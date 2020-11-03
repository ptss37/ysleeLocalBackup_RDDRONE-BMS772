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
 * /home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/node/port/433.GetStatistics.0.1.uavcan
 *
 * Template:
 * ServiceType.j2
 *
 * Generated at:  2020-11-02 13:23:09.134228 UTC
 * Is deprecated: no
 * Fixed port ID: 433
 * Full name:     uavcan.node.port.GetStatistics
 * Version:       0.1
 *
 */

#ifndef UAVCAN_NODE_PORT_GETSTATISTICS
#define UAVCAN_NODE_PORT_GETSTATISTICS
#include <canard_dsdl.h>


#define UAVCAN_NODE_PORT_GET_STATISTICS_PORT_ID 433

#define UAVCAN_NODE_PORT_GET_STATISTICS_MSG_SIZE 18



typedef struct uavcan_node_port_get_statisticsType
{
	uavcan_node_port_get_statistics_request request;
	uavcan_node_port_get_statistics_response response;
} uavcan_node_port_get_statistics;

void uavcan_node_port_get_statistics_serializeToBuffer(uavcan_node_port_get_statistics* msg, uint8_t* const buffer, const size_t starting_bit)
{
    uavcan_node_port_get_statistics_request_serializeToBuffer(&msg->request, buffer, starting_bit + 0);
    uavcan_node_port_get_statistics_response_serializeToBuffer(&msg->response, buffer, starting_bit + 24);
}

void uavcan_node_port_get_statistics_deserializeFromBuffer(uavcan_node_port_get_statistics* msg, const uint8_t* const buffer, const size_t buf_size, const size_t starting_bit)
{

        
        uavcan_node_port_get_statistics_request_deserializeFromBuffer(&msg->request, buffer, buf_size, starting_bit + 0);
        
        uavcan_node_port_get_statistics_response_deserializeFromBuffer(&msg->response, buffer, buf_size, starting_bit + 24);

    return msg;
}

#endif // UAVCAN_NODE_PORT_GETSTATISTICS