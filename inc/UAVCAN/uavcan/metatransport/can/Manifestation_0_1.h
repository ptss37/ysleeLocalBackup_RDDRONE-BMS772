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
 * /home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/metatransport/can/Manifestation.0.1.uavcan
 *
 * Template:
 * UnionType.j2
 *
 * Generated at:  2020-11-02 13:23:07.889858 UTC
 * Is deprecated: no
 * Fixed port ID: None
 * Full name:     uavcan.metatransport.can.Manifestation
 * Version:       0.1
 *
 */

#ifndef UAVCAN_METATRANSPORT_CAN_MANIFESTATION
#define UAVCAN_METATRANSPORT_CAN_MANIFESTATION
#include <canard_dsdl.h>

#include <uavcan/metatransport/can/Error_0_1.h>
#include <uavcan/metatransport/can/DataFD_0_1.h>
#include <uavcan/metatransport/can/DataClassic_0_1.h>
#include <uavcan/metatransport/can/RTR_0_1.h>

#define UAVCAN_METATRANSPORT_CAN_MANIFESTATION_MSG_SIZE 93



typedef struct uavcan_metatransport_can_manifestationType
{
#error "TODO: UnionType
} uavcan_metatransport_can_manifestation;

void uavcan_metatransport_can_manifestation_serializeToBuffer(uavcan_metatransport_can_manifestation* msg, uint8_t* const buffer, const size_t starting_bit)
{
    uavcan_metatransport_can_error_serializeToBuffer(&msg->error, buffer, starting_bit + 0);
    uavcan_metatransport_can_data_fd_serializeToBuffer(&msg->data_fd, buffer, starting_bit + 32);
    uavcan_metatransport_can_data_classic_serializeToBuffer(&msg->data_classic, buffer, starting_bit + 592);
    uavcan_metatransport_can_rtr_serializeToBuffer(&msg->remote_transmission_request, buffer, starting_bit + 704);
}

void uavcan_metatransport_can_manifestation_deserializeFromBuffer(uavcan_metatransport_can_manifestation* msg, const uint8_t* const buffer, const size_t buf_size, const size_t starting_bit)
{

        
        uavcan_metatransport_can_error_deserializeFromBuffer(&msg->error, buffer, buf_size, starting_bit + 0);
        
        uavcan_metatransport_can_data_fd_deserializeFromBuffer(&msg->data_fd, buffer, buf_size, starting_bit + 32);
        
        uavcan_metatransport_can_data_classic_deserializeFromBuffer(&msg->data_classic, buffer, buf_size, starting_bit + 592);
        
        uavcan_metatransport_can_rtr_deserializeFromBuffer(&msg->remote_transmission_request, buffer, buf_size, starting_bit + 704);

    return msg;
}

#endif // UAVCAN_METATRANSPORT_CAN_MANIFESTATION