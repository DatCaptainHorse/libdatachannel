/**
 * Copyright (c) 2020 Filip Klembara (in2core)
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#if RTC_ENABLE_MEDIA

#include "av1rtppacketizer.hpp"

#include "impl/internals.hpp"

namespace rtc {

OBUs AV1RtpPacketizer::splitMessage(binary_ptr message) {
	OBUs result;

	size_t index = 0;
	while (index < message->size()) {
		OBPOBUType obu_type;
		OBPFrameHeader frame_header;
		ptrdiff_t offset; // In bytes, weirdly enough..
		size_t obu_size;
		int temporal_id, spatial_id;
		OBPError err;

		int obu_extension_flag = (reinterpret_cast<uint8_t *>(message->data())[index] & 0x04) >> 2;

		// Get the next OBU
		int ret = obp_get_next_obu(reinterpret_cast<uint8_t *>(message->data()) + index,
		                           message->size() - index, &obu_type, &offset, &obu_size,
		                           &temporal_id, &spatial_id, &err);
		if (ret < 0) {
			LOG_WARNING << "Error parsing OBU header: " << err.error;
			break;
		}

		bool skip_this_obu = false;

		switch (obu_type) {
		case OBP_OBU_TEMPORAL_DELIMITER: {
			packetizerState->seenFrameHeader = 0;
			skip_this_obu = true;
			break;
		}
		case OBP_OBU_SEQUENCE_HEADER: {
			packetizerState->seenSeqHeader = true;
			// Zero out the sequence header
			packetizerState->seqHeader = {};
			ret = obp_parse_sequence_header(reinterpret_cast<uint8_t *>(message->data()) + index +
			                                    offset,
			                                obu_size, &packetizerState->seqHeader, &err);
			if (ret < 0) {
				LOG_ERROR << "Failed to parse sequence header: " << err.error;
				return result;
			}
			break;
		}
		case OBP_OBU_FRAME: {
			OBPTileGroup tile_group;
			// Zero out frame_header
			frame_header = {};
			if (!packetizerState->seenSeqHeader) {
				LOG_ERROR << "Encountered Frame OBU before Sequence Header OBU";
				return result;
			}
			ret = obp_parse_frame(reinterpret_cast<uint8_t *>(message->data()) + index + offset,
			                      obu_size, &packetizerState->seqHeader, &packetizerState->state,
			                      temporal_id, spatial_id, &frame_header, &tile_group,
			                      &packetizerState->seenFrameHeader, &err);
			if (ret < 0) {
				LOG_ERROR << "Failed to parse frame: " << err.error;
				return result;
			}
			break;
		}
		case OBP_OBU_REDUNDANT_FRAME_HEADER:
		case OBP_OBU_FRAME_HEADER: {
			// Zero out frame_header
			frame_header = {};
			if (!packetizerState->seenSeqHeader) {
				LOG_ERROR << "Encountered Frame Header OBU before Sequence Header OBU";
				return result;
			}
			ret = obp_parse_frame_header(
			    reinterpret_cast<uint8_t *>(message->data()) + index + offset, obu_size,
			    &packetizerState->seqHeader, &packetizerState->state, temporal_id, spatial_id,
			    &frame_header, &packetizerState->seenFrameHeader, &err);
			if (ret < 0) {
				LOG_ERROR << "Failed to parse frame header: " << err.error;
				return result;
			}
			break;
		}
		case OBP_OBU_TILE_LIST: {
			skip_this_obu = true;
			break;
		}
		default:
			skip_this_obu = true;
			break;
		}

		if (obu_size > 0 && !skip_this_obu) {
			if (packetizerState->last_temporal_id != -1 &&
			    temporal_id != packetizerState->last_temporal_id)
				packetizerState->temporal_id_changed = true;

			packetizerState->last_temporal_id = temporal_id;

			// Include OBU header as obuparse skips them (2/1 bytes)
			const size_t byte_offset_w_header = offset - (obu_extension_flag ? 2 : 1);
			const size_t obu_size_w_header = obu_size + (obu_extension_flag ? 2 : 1);
			const size_t byte_index = index / 8;

			const size_t maximumOBUSize = maximumFragmentSize * 8;

			// Split the OBU into fragments here
			size_t remaining_obu_size = obu_size_w_header;
			size_t current_obu_offset = 0;

			size_t start_position = byte_index + byte_offset_w_header;

			while (remaining_obu_size > 0) {
				size_t fragment_size = std::min(maximumOBUSize, remaining_obu_size);

				size_t end_position = start_position + fragment_size / 8;

				// If end_position would reach beyond the end of the message, set to end
				if (end_position > message->size())
					end_position = message->size();

				// Calculate size (in bytes) of the OBU payload needed
				const size_t obu_payload_size = end_position - start_position;

				// Create a binary object to hold the OBU payload
				binary obu_payload(obu_payload_size);

				std::copy(message->begin() + start_position, message->begin() + end_position,
				          obu_payload.begin());

				OBU::Fragmentation fragmentation = OBU::Fragmentation::None;
				if (remaining_obu_size > maximumOBUSize) {
					if (current_obu_offset == 0)
						fragmentation = OBU::Fragmentation::Start;
					else
						fragmentation = OBU::Fragmentation::Middle;
				} else if (current_obu_offset > 0 && remaining_obu_size <= maximumOBUSize)
					fragmentation = OBU::Fragmentation::End;

				std::shared_ptr<OBU> obu = std::make_shared<OBU>(
				    std::make_shared<binary>(obu_payload), temporal_id, spatial_id, fragmentation);

				// Add to the result
				result.push_back(obu);

				current_obu_offset += fragment_size;
				remaining_obu_size -= fragment_size;

				start_position = end_position;
			}
		}

		// Advance the index
		index += obu_size + (std::size_t)offset;
	}

	return result;
}

struct AV1AggregationHeader {
	uint8_t z : 1; // MUST be set to 1 if the first OBU element is an OBU fragment that is a
	               // continuation of an OBU fragment from the previous packet, and MUST be set to 0
	               // otherwise.
	uint8_t y : 1; // MUST be set to 1 if the last OBU element is an OBU fragment that will continue
	               // in the next packet, and MUST be set to 0 otherwise.
	uint8_t w : 2; // two bit field that describes the number of OBU elements in the packet. This
	               // field MUST be set equal to 0 or equal to the number of OBU elements contained
	               // in the packet.
	uint8_t n : 1; // MUST be set to 1 if the packet is the first packet of a coded video sequence,
	               // and MUST be set to 0 otherwise.
	uint8_t reserved : 3;
};

std::vector<binary_ptr> AV1RtpPacketizer::createPackets(const OBUs &obus) {
	std::vector<binary_ptr> result;

	// 1 packet for each OBU
	for (const auto &obu : obus) {
		// Create the packet
		binary_ptr packet = std::make_shared<binary>();

		// Add the AV1 Aggregation Header (1 byte)
		AV1AggregationHeader aggregationHeader = {};
		aggregationHeader.z = obu->fragmentation == OBU::Fragmentation::Middle ||
		                      obu->fragmentation == OBU::Fragmentation::End;
		aggregationHeader.y = obu->fragmentation == OBU::Fragmentation::Start ||
		                      obu->fragmentation == OBU::Fragmentation::Middle;
		aggregationHeader.w = 1;
		aggregationHeader.n = first_packet;

		first_packet = false;

		// Convert the header to a byte
		std::byte aggregationHeaderByte;
		memcpy(&aggregationHeaderByte, &aggregationHeader, sizeof(aggregationHeader));
		packet->push_back(aggregationHeaderByte);

		// Add the OBU payload
		packet->insert(packet->end(), obu->data->begin(), obu->data->end());

		// Add the packet to the result
		result.push_back(packet);
	}

	return result;
}

AV1RtpPacketizer::AV1RtpPacketizer(shared_ptr<RtpPacketizationConfig> rtpConfig,
                                   uint16_t maximumFragmentSize)
    : RtpPacketizer(rtpConfig), maximumFragmentSize(maximumFragmentSize), MediaHandlerRootElement(),
      packetizerState(std::make_shared<AV1RtpPacketizerState>()) {
	packetizerState->state = {};
}

ChainedOutgoingProduct
AV1RtpPacketizer::processOutgoingBinaryMessage(ChainedMessagesProduct messages,
                                               message_ptr control) {
	ChainedMessagesProduct packets = make_chained_messages_product();
	packets->reserve(messages->size());

	bool over_limit = false;
	packetizerState->remainingMTU = maximumFragmentSize;

	// Get the last chain's packetsToDeliver, deliver them first, rememeber to subtract
	// remainingSpaceForPayload
	if (!packetizerState->packetsToDeliver.empty()) {
		while (packetizerState->remainingMTU > 0 && !packetizerState->packetsToDeliver.empty()) {
			auto packet = packetizerState->packetsToDeliver.front();

			// Check if packet would go over limit
			if (packet->size() > packetizerState->remainingMTU || over_limit) {
				packetizerState->remainingMTU = 0;
				over_limit = true;
			}

			if (!over_limit) {
				packets->push_back(packetize(packet, packetizerState->temporal_id_changed));
				packetizerState->remainingMTU -= packet->size();
				packetizerState->packetsToDeliver.pop();
			}
		}
	}

	for (auto message : *messages) {
		auto obus = splitMessage(message);
		if (obus.empty())
			continue;

		auto packetsForMessage = createPackets(obus);
		for (auto packet : packetsForMessage) {
			// Check if packet would go over limit
			if (packet->size() > packetizerState->remainingMTU || over_limit) {
				packetizerState->packetsToDeliver.push(packet);
				packetizerState->remainingMTU = 0;
				over_limit = true;
			}

			if (!over_limit) {
				packets->push_back(packetize(packet, packetizerState->temporal_id_changed));
				packetizerState->remainingMTU -= packet->size();
			}
		}
	}

	// We must atleast return 1 packet, even if it goes out of the limit
	if (packets->empty()) {
		// Get first packet from packetsToDeliver
		if (!packetizerState->packetsToDeliver.empty()) {
			auto packet = packetizerState->packetsToDeliver.front();
			packets->push_back(packetize(packet, packetizerState->temporal_id_changed));
			packetizerState->packetsToDeliver.pop();
		}
	}

	packetizerState->temporal_id_changed = false;

	return {packets, control};
}

} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */
