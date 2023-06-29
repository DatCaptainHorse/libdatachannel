/**
 * Copyright (c) 2023 Kristian Ollikainen (DatCaptainHorse)
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#if RTC_ENABLE_MEDIA

#include "av1rtppacketizer.hpp"

#include "impl/internals.hpp"

namespace rtc {

std::vector<OBU> AV1RtpPacketizer::splitMessage(binary_ptr message) {
	std::vector<OBU> obus;
	OBPOBUType obuType;
	ptrdiff_t offset;
	size_t obuSize;
	int temporalId, spatialId;
	OBPError error;

	size_t index = 0;

	while (index < message->size()) {
		int obuExtFlag = (reinterpret_cast<uint8_t *>(message->data())[index] & 0x04) >> 2;

		int ret = obp_get_next_obu(reinterpret_cast<uint8_t *>(message->data() + index),
		                           message->size() - index, &obuType, &offset, &obuSize,
		                           &temporalId, &spatialId, &error);

		if (ret < 0) {
			LOG_ERROR << "Error parsing OBU header: " << error.error;
			break;
		}

		// If OBU size > 0
		if (obuSize > 0) {
			// Include OBU header as obuparse skips them (2/1 bytes)
			// Note - Chrome recognizes without this, needed?
			const size_t byte_offset_w_header = offset - (obuExtFlag ? 2 : 1);
			const size_t obu_size_w_header = obuSize + (obuExtFlag ? 2 : 1);
			const size_t byte_index = index / 8;

			const size_t maximumOBUSize = defaultMaximumFragmentSize;

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
				binary_ptr obu_payload = std::make_shared<binary>(obu_payload_size);

				std::copy(message->begin() + start_position, message->begin() + end_position,
				          obu_payload->begin());

				bool endFrag = false;
				if (current_obu_offset > 0 && remaining_obu_size <= maximumOBUSize)
					endFrag = true;

				// Add to the result
				obus.push_back(OBU(obuType, obu_payload, endFrag));

				current_obu_offset += fragment_size;
				remaining_obu_size -= fragment_size;

				start_position = end_position;
			}
		}

		// Move forward the buffer
		index += obuSize + (std::size_t)offset;
	}

	return obus;
}

// Note - Adding AV1 Aggregation Header had Chrome not recognize the stream as AV1, so I removed it

AV1RtpPacketizer::AV1RtpPacketizer(shared_ptr<RtpPacketizationConfig> rtpConfig)
    : RtpPacketizer(rtpConfig), MediaHandlerRootElement() {}

ChainedOutgoingProduct
AV1RtpPacketizer::processOutgoingBinaryMessage(ChainedMessagesProduct messages,
                                               message_ptr control) {
	ChainedMessagesProduct packets = make_chained_messages_product();

	for (auto &message : *messages) {
		// Split the message into OBUs
		std::vector<OBU> obus = splitMessage(message);

		if (!obus.empty()) {
			// Create RTP packets

			/*
			 "The RTP header Marker bit MUST be set equal to 0 if the packet is not the last packet
			 of the temporal unit, it SHOULD be set equal to 1 otherwise."

			 Note - Not sure if needed here, I did try just having it as "false" but Chrome stopped
			 recognizing the stream as AV1..
			 */
			for (std::size_t i = 0; i < obus.size(); i++)
				packets->push_back(packetize(obus[i].data, obus[i].endFragment));
		}
	}

	return {packets, control};
}

} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */
