/**
 * Copyright (c) 2023 Kristian Ollikainen (DatCaptainHorse)
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

struct OBU {
	OBPOBUType type;
	binary_ptr data;
	bool endFragment;

	OBU(OBPOBUType type, binary_ptr data, bool endFragment)
	    : type(type), data(data), endFragment(endFragment) {}
};

/// RTP packetization of AV1 payload
class RTC_CPP_EXPORT AV1RtpPacketizer final : public RtpPacketizer, public MediaHandlerRootElement {
public:
	/// Default clock rate for AV1 in RTP
	inline static const uint32_t defaultClockRate = 90 * 1000;
	inline static const uint16_t defaultMaximumFragmentSize =
	    uint16_t(RTC_DEFAULT_MTU - 12 - 8 - 40); // SRTP/UDP/IPv6

	std::vector<OBU> splitMessage(binary_ptr message);

	/// Constructs AV1 payload packetizer with given RTP configuration.
	/// @note RTP configuration is used in packetization process which may change some configuration
	/// properties such as sequence number.
	/// @param rtpConfig  RTP configuration
	AV1RtpPacketizer(shared_ptr<RtpPacketizationConfig> rtpConfig);

	ChainedOutgoingProduct processOutgoingBinaryMessage(ChainedMessagesProduct messages,
	                                                    message_ptr control) override;
};

} // namespace rtc

#endif /* RTC_ENABLE_MEDIA */

#endif /* RTC_AV1_RTP_PACKETIZER_H */
