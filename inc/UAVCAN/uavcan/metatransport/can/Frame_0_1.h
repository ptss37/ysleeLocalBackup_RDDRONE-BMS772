/*
 *
 * BSD 3-Clause License
 * 
 * Copyright (c) 2020, NXP Drone and Rover Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * UAVCAN data structure definition.
 *
 * AUTOGENERATED, DO NOT EDIT.
 *
 * Source File:
 * /home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/metatransport/can/Frame.0.1.uavcan
 *
 * Template:
 * StructureType.j2
 *
 * Generated at:  2020-11-02 13:23:07.796758 UTC
 * Is deprecated: no
 * Fixed port ID: None
 * Full name:     uavcan.metatransport.can.Frame
 * Version:       0.1
 *
 */

#ifndef UAVCAN_METATRANSPORT_CAN_FRAME
#define UAVCAN_METATRANSPORT_CAN_FRAME
#include <canard_dsdl.h>

#include <uavcan/time/SynchronizedTimestamp_1_0.h>
#include <uavcan/metatransport/can/Manifestation_0_1.h>

#define UAVCAN_METATRANSPORT_CAN_FRAME_MSG_SIZE 78



typedef struct uavcan_metatransport_can_frameType
{
	uavcan_time_synchronized_timestamp timestamp;
	uavcan_metatransport_can_manifestation manifestation;
} uavcan_metatransport_can_frame;

void uavcan_metatransport_can_frame_serializeToBuffer(uavcan_metatransport_can_frame* msg, uint8_t* const buffer, const size_t starting_bit)
{
    uavcan_time_synchronized_timestamp_serializeToBuffer(&msg->timestamp, buffer, starting_bit + 0);
    uavcan_metatransport_can_manifestation_serializeToBuffer(&msg->manifestation, buffer, starting_bit + 56);
}

void uavcan_metatransport_can_frame_deserializeFromBuffer(uavcan_metatransport_can_frame* msg, const uint8_t* const buffer, const size_t buf_size, const size_t starting_bit)
{

        
        uavcan_time_synchronized_timestamp_deserializeFromBuffer(&msg->timestamp, buffer, buf_size, starting_bit + 0);
        
        uavcan_metatransport_can_manifestation_deserializeFromBuffer(&msg->manifestation, buffer, buf_size, starting_bit + 56);

    return msg;
}

#endif // UAVCAN_METATRANSPORT_CAN_FRAME