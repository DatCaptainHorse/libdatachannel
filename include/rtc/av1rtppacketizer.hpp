/**
 * Copyright (c) 2020 Filip Klembara (in2core)
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef RTC_AV1_RTP_PACKETIZER_H
#define RTC_AV1_RTP_PACKETIZER_H

#if RTC_ENABLE_MEDIA

#include "mediahandlerrootelement.hpp"
#include "rtppacketizer.hpp"

#include <queue>

extern "C" {
#include "obuparse.h"
}

namespace rtc {

class OBU {
public:
	enum class Fragmentation { None, Start, Middle, End };

public:
	OBU(binary_ptr data, int temporal_id, int spatial_id, Fragmentation fragmentation)
	    : data(data), temporal_id(temporal_id), spatial_id(spatial_id),
	      fragmentation(fragmentation) {}

	binary_ptr data;
	int temporal_id;
	int spatial_id;

	Fragmentation fragmentation;
};

class OBUs : public std::vector<std::shared_ptr<OBU>> {};

struct AV1RtpPacketizerState {
	OBPSequenceHeader seqHeader;
	bool seenSeqHeader = false;
	int seenFrameHeader = 0;
	OBPState state;
	size_t remainingMTU;
	int last_temporal_id = -1;
	bool temporal_id_changed = false;
	std::queue<binary_ptr> packetsToDeliver;
};

/// RTP packetization of AV1 payload
class RTC_CPP_EXPORT AV1RtpPacketizer final : public RtpPacketizer, public MediaHandlerRootElement {
	OBUs splitMessage(binary_ptr message);
	const uint16_t maximumFragmentSize;
	std::shared_ptr<AV1RtpPacketizerState> packetizerState;

	bool first_packet = true;

	static const uint16_t defaultMaximumFragmentSize =
	    uint16_t(RTC_DEFAULT_MTU - 12 - 1 - 8 - 40); // SRTP/AV1AggrHdr/UDP/IPv6

public:
	/// Default clock rate for AV1 in RTP
	inline static const uint32_t defaultClockRate = 90 * 1000;

	/// Constructs AV1 payload packetizer with given RTP configuration.
	/// @note RTP configuration is used in packetization process which may change some configuration
	/// properties such as sequence number.
	/// @param rtpConfig  RTP configuration
	/// @param maximumFragmentSize maximum size of one OBU fragment
	AV1RtpPacketizer(shared_ptr<RtpPacketizationConfig> rtpConfig,
	                 uint16_t maximumFragmentSize = defaultMaximumFragmentSize);

	std::vector<binary_ptr> createPackets(const OBUs &obus);

	ChainedOutgoingProduct processOutgoingBinaryMessage(ChainedMessagesProduct messages,
	                                                    message_ptr control) override;
};

} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */

#endif /* RTC_AV1_RTP_PACKETIZER_H */
