/****************************************************************************
 * nxp_bms/BMS_v1/src/data.c
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
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <pthread.h>
#include <fcntl.h>

#include "data.h"
#include "cli.h"

#include "BMS_data_types.h"
#include "BMS_data_limits.h"


/****************************************************************************
 * Defines
 ****************************************************************************/
//#define SHOW_CELLV
//#define SHOW_TEMP

#define HANDLE_CHANGE_DEFAULT_PRIORITY    100
#define HANDLE_CHANGE_DEFAULT_STACK_SIZE  2048

/*#define set_MAX(typeT, par)                                   (typeT == UINT8VAL ? .parametersInfo[par].max.U8 = par##_MAX : \
                                                              (typeT == UINT16VAL ? .parametersInfo[par].max.U16 = par##_MAX : \
                                                              (.parametersInfo[par].max.FX10 = (int16_t)par##_MAX))) //(typeT != FLOATVAL ? (int16_t)par##_MAX :((int16_t)par##_MAX*FLOAT_MULTIPIER))
*/
/* #define SET_DEFAULT(typeT, maxOn, minOn, par, parVal_t) \
                                                               .parametersInfo[par].type                            = typeT, \
                                                               .parametersInfo[par].checkMax                        = maxOn, \
                                                               .parametersInfo[par].checkMin                        = minOn, \
                                                               .parametersInfo[par].max.U16                         = (typeT != FLOATVAL ? par##_MAX :((int16_t)((float)par##_MAX*FLOAT_MULTIPIER))), \
                                                               .parametersInfo[par].min.U16                         = (typeT != FLOATVAL ? par##_MIN :((int16_t)((float)par##_MIN*FLOAT_MULTIPIER))), \
                                                               .parametersInfo[par].parameterAdr                    = &s_parameters.parameters.parVal_t, \
                                                               .parameters.parVal_t                                 = par##_DEFAULT
*/
#define SET_DEFAULT2(typeT, maxOn, minOn, par, parVal_t) \
                                                              .type                            = typeT, \
                                                              .checkMax                        = maxOn, \
                                                              .checkMin                        = minOn, \
                                                              /*.max.U16                         = (typeT != FLOATVAL ? par##_MAX :((int16_t)((float)par##_MAX*FLOAT_MULTIPIER))), */\
                                                              .max.FX10                         = (typeT != FLOATVAL ? par##_MAX :((int16_t)((float)par##_MAX*FLOAT_MULTIPIER))), \
                                                              /*.min.U16                         = (typeT != FLOATVAL ? par##_MIN :((int16_t)((float)par##_MIN*FLOAT_MULTIPIER))), */\
                                                              .min.FX10                         = (typeT != FLOATVAL ? par##_MIN :((int16_t)((float)par##_MIN*FLOAT_MULTIPIER))), \
                                                              .parameterAdr                    = &s_parameters.parVal_t, \

/*#define SET_VALUE_IN_VAR(var, typeT, maxOn, minOn, par, parVal_t) \
                                                              var.parametersInfo[par].type                            = typeT; \
                                                              var.parametersInfo[par].checkMax                        = maxOn; \
                                                              var.parametersInfo[par].checkMin                        = minOn; \
                                                              var.parametersInfo[par].max.U16                         = (typeT != FLOATVAL ? par##_MAX :((int16_t)((float)par##_MAX*FLOAT_MULTIPIER))); \
                                                              var.parametersInfo[par].min.U16                         = (typeT != FLOATVAL ? par##_MIN :((int16_t)((float)par##_MIN*FLOAT_MULTIPIER))); \
                                                              var.parametersInfo[par].parameterAdr                    = &s_parameters.parameters.parVal_t; \
                                                              var.parameters.parVal_t                                 = par##_DEFAULT;
*/

//#define CHECK_NULL_ASSIGN_LENGHT(data)                        if(outLength != NULL) *outLength = sizeof(data);
//#define ASSIGN_OUT_TYPE(valueType)                            if(outData != NULL) (*(valueType*)outData = *(valueType*)lvRetValue);

#define CHECK_CHANGED(setting, valueType)                     (setting != (*(valueType*)inNewValue)) ? (lvChanged = true) : (lvChanged = false)
#define ASSIGN(setting, valueType)                            (setting = (*(valueType*)inNewValue));
#define CHECK_ASSIGN_BOTH(max, min, setting, valueType)       CHECK_CHANGED(setting, valueType); RANGE_OK(min, (*(valueType*)inNewValue), max) {ASSIGN(setting, valueType) lvRetValue = 0;} else{lvRetValue = -1;}
#define CHECK_ASSIGN_MAX(max, setting, valueType)             CHECK_CHANGED(setting, valueType); RANGE_OK_HIGH((*(valueType*)inNewValue), max) {ASSIGN(setting, valueType) lvRetValue = 0;} else{lvRetValue = -1;}
#define CHECK_ASSIGN_MIN(min, setting, valueType)             CHECK_CHANGED(setting, valueType); RANGE_OK_LOW(min, (*(valueType*)inNewValue))  {ASSIGN(setting, valueType) lvRetValue = 0;} else{lvRetValue = -1;}
#define CHECK_ASSIGN_NONE(setting, valueType)                 CHECK_CHANGED(setting, valueType); ASSIGN(setting, valueType) lvRetValue = 0; 

/****************************************************************************
 * Types
 ****************************************************************************/
//! @brief  enum to indicate what limit needs to be checked
typedef enum {
CHECK_NONE  = 0, //!< no limit needs to be checked
CHECK_LOW   = 1, //!< the lower limit needs to be checked 
CHECK_HIGH  = 2, //!< the upper limit needs to be checked
CHECK_BOTH  = 3  //!< both upper and lower limit needs to be checked
}checkLimit_t;


/****************************************************************************
 * private data
 ****************************************************************************/

// the callback functions
/*! @brief  callback function to handle the parameter change */
parameterChangeCallbackFunction       g_parameterChangeCallbackFunctionfp;

//! the data mutex
pthread_mutex_t dataLock;

//! to indicate data is initialized
bool gDataInitialized = false;

//! the eeprom mutex
pthread_mutex_t flashLock;
//! to indicate data is initialized
bool gFlashInitialized = false;

//! to indicate a configuration parameter has changed, and it should be saved to flashed 
static bool gSavableParameterChanged = false;

//make the struct containing all the data with the default values
//BMSparameters_t s_parameters;
/* =
{
  .parameters.basicVariables.batt_temperature                = C_BATT_DEFAULT,
  .parameters.basicVariables.batt_voltage                    = V_BATT_DEFAULT,
  .parameters.basicVariables.batt_current                    = I_BATT_DEFAULT,
  .parameters.basicVariables.avg_power_10sec                 = P_AVG_DEFAULT,
  .parameters.basicVariables.remaining_cap_wh                = A_REM_WH_DEFAULT, 
  .parameters.basicVariables.full_charge_cap_wh              = A_FULL_WH_DEFAULT,
  .parameters.basicVariables.hours_to_full_charge            = T_FULL_DEFAULT,
  .parameters.basicVariables.s_flags                    = S_FLAGS_DEFAULT,
  .parameters.basicVariables.state_of_health                 = S_HEALTH_PCT_DEFAULT,
  .parameters.basicVariables.state_of_charge                 = S_CHARGE_PCT_DEFAULT,
  .parameters.basicVariables.state_of_charge_stdev           = S_CHARGE_PCT_STDEV_DEFAULT,
  .parameters.basicVariables.battery_id                      = BATT_ID_DEFAULT,
  .parameters.basicVariables.model_instance_id               = MODEL_ID_DEFAULT,
  .parameters.basicVariables.model_name                      = MODEL_NAME_DEFAULT,
    
  .parameters.additionalVariables.cell1_voltage              = V_CELL1_DEFAULT,
  .parameters.additionalVariables.cell2_voltage              = V_CELL2_DEFAULT,
  .parameters.additionalVariables.cell3_voltage              = V_CELL3_DEFAULT,
  .parameters.additionalVariables.cell4_voltage              = V_CELL4_DEFAULT,
  .parameters.additionalVariables.cell5_voltage              = V_CELL5_DEFAULT,
  .parameters.additionalVariables.cell6_voltage              = V_CELL6_DEFAULT,
  .parameters.additionalVariables.AFE_temperature            = C_AFE_DEFAULT,
  .parameters.additionalVariables.T_temperature              = C_T_DEFAULT,
  .parameters.additionalVariables.R_temperature              = C_R_DEFAULT,
  .parameters.additionalVariables.N_charges                  = N_CHARGES_DEFAULT,
  .parameters.additionalVariables.N_chargesCmplt             = N_CHARGES_FULL_DEFAULT,
    
  .parameters.configurationVariables.N_cells                 = N_CELLS_DEFAULT,
  .parameters.configurationVariables.T_meas                  = T_MEAS_DEFAULT,
  .parameters.configurationVariables.T_cyclic                = t_CYCLIC_DEFAULT,
  .parameters.configurationVariables.IBatt_sleep_oc          = I_SLEEP_OC_DEFAULT,
  .parameters.configurationVariables.Cell_ov                 = V_CELL_OV_DEFAULT,
  .parameters.configurationVariables.Cell_uv                 = V_CELL_UV_DEFAULT,
  .parameters.configurationVariables.Cell_ot                 = C_CELL_OT_DEFAULT,
  .parameters.configurationVariables.Cell_ot_charge          = C_CELL_OT_CHARGE_DEFAULT,
  .parameters.configurationVariables.Cell_ut                 = C_CELL_UT_DEFAULT,
  .parameters.configurationVariables.Cell_ut_charge          = CELL_CHARGE_UT_DEFAULT,
  .parameters.configurationVariables.T_batt_timeout          = T_BMS_TIMEOUT_DEFAULT,
  .parameters.configurationVariables.T_fault_timeout         = T_FAULT_TIMEOUT_DEFAULT,
  .parameters.configurationVariables.T_batt_chrg_detect      = T_CHARGE_DETECT_DEFAULT,
  .parameters.configurationVariables.T_cb_delay              = T_CB_DELAY_DEFAULT,
  .parameters.configurationVariables.T_chrg_relax            = T_CHARGE_RELAX_DEFAULT,
  .parameters.configurationVariables.IBatt_chrg_complete     = I_CHARGE_FULL_DEFAULT,
  .parameters.configurationVariables.Cell_chrg_margin        = V_CELL_MARGIN_DEFAULT,

  .parameters.hardwareVariables.VBatt_min                    = VBATT_MIN_DEFAULT,
  .parameters.hardwareVariables.VBatt_max                    = VBATT_MAX_DEFAULT,
  .parameters.hardwareVariables.IBatt_maxp                   = IBATT_MAXP_DEFAULT,
  .parameters.hardwareVariables.IBatt_max                    = IBATT_MAX_DEFAULT,
  .parameters.hardwareVariables.IBatt_sc                     = IBATT_SC_DEFAULT,
  .parameters.hardwareVariables.T_IBatt_sc_blank             = T_IBATT_SC_BLANK_DEFAULT,
  .parameters.hardwareVariables.I_bal                        = I_BAL_DEFAULT, 

  .parametersInfo.type                                       = U8,
  .parametersInfo.checkMax                                   = true,
  .parametersInfo.checkMin                                   = true,
  .parametersInfo.max                                        = S_CHARGE_MAX,
  .parametersInfo.min                                        = S_CHARGE_MIN,
  .parametersInfo.parameterAdr                               = &s_parameters.parameters.basicVariables.state_of_charge
};*/

/* the struct containing all the data with the default values and the right parameter info
every type, minOn, maxOn, min, max, address and the default value is set
this struct is set in the setDefault function */
// BMSparameters_t s_parameters =
// {
//   //SET_DEFAULT(UINT8VAL,  true, true, S_HEALTH, basicVariables.state_of_health),
//   //SET_DEFAULT(UINT8VAL,  true, true, S_CHARGE, basicVariables.state_of_charge)

//   // assign the default values;
//   SET_DEFAULT(FLOATVAL,  true,  true,  C_BATT, basicVariables.C_batt),
//   SET_DEFAULT(FLOATVAL,  true,  true,  V_OUT, basicVariables.V_out),
//   SET_DEFAULT(FLOATVAL,  true,  true,  V_BATT, basicVariables.V_batt),
//   SET_DEFAULT(FLOATVAL,  true,  true,  I_BATT, basicVariables.I_batt),
//   SET_DEFAULT(FLOATVAL,  true,  true,  I_BATT_AVG, basicVariables.I_batt_avg),
//   //SET_DEFAULT(BOOLVAL,   false, false, S_OUT, basicVariables.output_status),
//   SET_DEFAULT(UINT8VAL,  true, false,  S_OUT, basicVariables.s_out),
//   SET_DEFAULT(FLOATVAL,  true,  true,  P_AVG, basicVariables.P_avg),
//   SET_DEFAULT(FLOATVAL,  true,  true,  E_USED, basicVariables.E_used),
//   SET_DEFAULT(FLOATVAL,  false, true,  A_REM, basicVariables.A_rem), 
//   SET_DEFAULT(FLOATVAL,  false, true,  A_FULL, basicVariables.A_full),
//   SET_DEFAULT(FLOATVAL,  false, true,  T_FULL, basicVariables.t_full),
//   SET_DEFAULT(UINT8VAL,  false, false, S_FLAGS, basicVariables.s_flags),
//   SET_DEFAULT(UINT8VAL,  true,  false, S_HEALTH, basicVariables.s_health),
//   SET_DEFAULT(UINT8VAL,  true,  false, S_CHARGE, basicVariables.s_charge),
//   //SET_DEFAULT(UINT8VAL,  true,  false, S_CHARGE_STDEV, basicVariables.state_of_charge_stdev),
//   SET_DEFAULT(UINT8VAL,  false, false, BATT_ID, basicVariables.batt_id),
//   SET_DEFAULT(INT32VAL,  false, false, MODEL_ID, basicVariables.model_id),
//   SET_DEFAULT(STRINGVAL, false, false, MODEL_NAME, basicVariables.model_name),
    
//   SET_DEFAULT(FLOATVAL,  true,  true,  V_CELL1, additionalVariables.V_cell1),
//   SET_DEFAULT(FLOATVAL,  true,  true,  V_CELL2, additionalVariables.V_cell2),
//   SET_DEFAULT(FLOATVAL,  true,  true,  V_CELL3, additionalVariables.V_cell3),
//   SET_DEFAULT(FLOATVAL,  true,  true,  V_CELL4, additionalVariables.V_cell4),
//   SET_DEFAULT(FLOATVAL,  true,  true,  V_CELL5, additionalVariables.V_cell5),
//   SET_DEFAULT(FLOATVAL,  true,  true,  V_CELL6, additionalVariables.V_cell6),
//   SET_DEFAULT(FLOATVAL,  true,  true,  C_AFE, additionalVariables.C_AFE),
//   SET_DEFAULT(FLOATVAL,  true,  true,  C_T, additionalVariables.C_T),
//   SET_DEFAULT(FLOATVAL,  true,  true,  C_R, additionalVariables.C_R),
//   SET_DEFAULT(UINT16VAL, false, false, N_CHARGES, additionalVariables.N_charges),
//   SET_DEFAULT(UINT16VAL, false, false, N_CHARGES_FULL, additionalVariables.N_charges_full),
    
//   SET_DEFAULT(UINT8VAL,  true,  true,  N_CELLS, configurationVariables.N_cells),
//   SET_DEFAULT(UINT16VAL, false, true,  T_MEAS, configurationVariables.t_meas),
//   SET_DEFAULT(UINT16VAL, false, true,  T_FTTI, configurationVariables.t_ftti),
//   SET_DEFAULT(UINT8VAL,  false, true,  T_CYCLIC, configurationVariables.t_cyclic),
//   SET_DEFAULT(UINT8VAL,  false, true,  I_SLEEP_OC, configurationVariables.I_sleep_oc),
//   SET_DEFAULT(FLOATVAL,  true,  true,  V_CELL_OV, configurationVariables.V_cell_ov),
//   SET_DEFAULT(FLOATVAL,  true,  true,  V_CELL_UV, configurationVariables.V_cell_uv),
//   SET_DEFAULT(FLOATVAL,  true,  true,  C_CELL_OT, configurationVariables.C_cell_ot),
//   SET_DEFAULT(FLOATVAL,  true,  true,  C_CELL_OT_CHARGE, configurationVariables.C_cell_ot_charge),
//   SET_DEFAULT(FLOATVAL,  true,  true,  C_CELL_UT, configurationVariables.C_cell_ut),
//   SET_DEFAULT(FLOATVAL,  true,  true,  C_CELL_UT_CHARGE, configurationVariables.C_cell_ut_charge),
//   SET_DEFAULT(FLOATVAL,  false, true,  A_FACTORY, configurationVariables.A_factory),
//   SET_DEFAULT(UINT16VAL, false, true,  T_BMS_TIMEOUT, configurationVariables.t_bms_timeout),
//   SET_DEFAULT(UINT8VAL,  true,  true,  T_FAULT_TIMEOUT, configurationVariables.t_fault_timeout),
//   SET_DEFAULT(UINT8VAL,  false, true,  T_CHARGE_DETECT, configurationVariables.t_charge_detect),
//   SET_DEFAULT(UINT8VAL,  false, true,  T_CB_DELAY, configurationVariables.t_cb_delay),
//   SET_DEFAULT(UINT16VAL, false, true,  T_CHARGE_RELAX, configurationVariables.t_charge_relax),
//   SET_DEFAULT(UINT16VAL, false, true,  I_CHARGE_FULL, configurationVariables.I_charge_full),
//   SET_DEFAULT(FLOATVAL,  false, true,  I_CHARGE_MAX, configurationVariables.I_charge_max),
//   SET_DEFAULT(FLOATVAL,  false, true,  I_OUT_MAX, configurationVariables.I_out_max),
//   SET_DEFAULT(UINT8VAL,  false, true,  V_CELL_MARGIN, configurationVariables.V_cell_margin),
//   SET_DEFAULT(INT32VAL,  false, true,  T_OCV_CYCLIC0, configurationVariables.t_ocv_cyclic0),
//   SET_DEFAULT(INT32VAL,  false, true,  T_OCV_CYCLIC1, configurationVariables.t_ocv_cyclic1),
//   SET_DEFAULT(FLOATVAL,  true,  true,  C_PCB_UT, configurationVariables.C_pcb_ut),
//   SET_DEFAULT(FLOATVAL,  true,  true,  C_PCB_OT, configurationVariables.C_pcb_ot),
//   SET_DEFAULT(FLOATVAL,  true,  true,  V_STORAGE, configurationVariables.V_storage),
//   SET_DEFAULT(FLOATVAL,  true,  true,  OCV_SLOPE, configurationVariables.ocv_slope),
//   SET_DEFAULT(UINT8VAL,  true,  false, BATT_EOL, configurationVariables.batt_eol),
//   SET_DEFAULT(UINT8VAL,  true,  true,  SENSOR_ENABLE, configurationVariables.sensor_enable),
//   SET_DEFAULT(UINT8VAL,  true,  true,  SELF_DISCHARGE_ENABLE, configurationVariables.self_discharge_enable),
//   SET_DEFAULT(UINT8VAL,  true,  true,  UAVCAN_NODE_STATIC_ID, configurationVariables.Uavcan_node_static_id),
//   SET_DEFAULT(UINT16VAL, true,  true,  UAVCAN_SUBJECT_ID, configurationVariables.Uavcan_subject_id),
//   SET_DEFAULT(UINT8VAL,  true,  true,  UAVCAN_FD_MODE, configurationVariables.Uavcan_fd_mode),
//   SET_DEFAULT(INT32VAL,  true,  true,  UAVCAN_BITRATE, configurationVariables.Uavcan_bitrate),
//   SET_DEFAULT(INT32VAL,  true,  true,  UAVCAN_FD_BITRATE, configurationVariables.Uavcan_fd_bitrate),

//   SET_DEFAULT(UINT8VAL,  true, true,   V_MIN, hardwareVariables.V_min),
//   SET_DEFAULT(UINT8VAL,  true, true,   V_MAX, hardwareVariables.V_max),
//   SET_DEFAULT(UINT8VAL,  true, true,   I_PEAK, hardwareVariables.I_peak),
//   SET_DEFAULT(UINT8VAL,  true, true,   I_MAX, hardwareVariables.I_max),
//   SET_DEFAULT(UINT16VAL, true, true,   I_SHORT, hardwareVariables.I_short),
//   SET_DEFAULT(UINT8VAL,  false, true,  T_SHORT, hardwareVariables.t_short),
//   SET_DEFAULT(UINT8VAL,  false, false, I_BAL, hardwareVariables.I_bal)
// };

/*! the struct containing all the data with the default values and the right parameter info
every type, minOn, maxOn, min, max, address and the default value is set
this struct is set in the setDefault function */
BMSParameterValues_t s_parameters = 
{
  .basicVariables.C_batt                          = C_BATT_DEFAULT, 
  .basicVariables.V_out                           = V_OUT_DEFAULT, 
  .basicVariables.V_batt                          = V_BATT_DEFAULT, 
  .basicVariables.I_batt                          = I_BATT_DEFAULT, 
  .basicVariables.I_batt_avg                      = I_BATT_AVG_DEFAULT, 
  .basicVariables.s_out                           = S_OUT_DEFAULT, 
  .basicVariables.P_avg                           = P_AVG_DEFAULT, 
  .basicVariables.E_used                          = E_USED_DEFAULT, 
  .basicVariables.A_rem                           = A_REM_DEFAULT, 
  .basicVariables.A_full                          = A_FULL_DEFAULT, 
  .basicVariables.t_full                          = T_FULL_DEFAULT, 
  .basicVariables.s_flags                         = S_FLAGS_DEFAULT, 
  .basicVariables.s_health                        = S_HEALTH_DEFAULT, 
  .basicVariables.s_charge                        = S_CHARGE_DEFAULT, 
  .basicVariables.batt_id                         = BATT_ID_DEFAULT, 
  .basicVariables.model_id                        = MODEL_ID_DEFAULT, 
  .basicVariables.model_name                      = MODEL_NAME_DEFAULT, 

  .additionalVariables.V_cell1                    = V_CELL1_DEFAULT, 
  .additionalVariables.V_cell2                    = V_CELL2_DEFAULT, 
  .additionalVariables.V_cell3                    = V_CELL3_DEFAULT, 
  .additionalVariables.V_cell4                    = V_CELL4_DEFAULT, 
  .additionalVariables.V_cell5                    = V_CELL5_DEFAULT, 
  .additionalVariables.V_cell6                    = V_CELL6_DEFAULT, 
  .additionalVariables.C_AFE                      = C_AFE_DEFAULT, 
  .additionalVariables.C_T                        = C_T_DEFAULT, 
  .additionalVariables.C_R                        = C_R_DEFAULT, 
  .additionalVariables.N_charges                  = N_CHARGES_DEFAULT, 
  .additionalVariables.N_charges_full             = N_CHARGES_FULL_DEFAULT, 

  .configurationVariables.N_cells                 = N_CELLS_DEFAULT, 
  .configurationVariables.t_meas                  = T_MEAS_DEFAULT, 
  .configurationVariables.t_ftti                  = T_FTTI_DEFAULT, 
  .configurationVariables.t_cyclic                = T_CYCLIC_DEFAULT, 
  .configurationVariables.I_sleep_oc              = I_SLEEP_OC_DEFAULT, 
  .configurationVariables.V_cell_ov               = V_CELL_OV_DEFAULT, 
  .configurationVariables.V_cell_uv               = V_CELL_UV_DEFAULT, 
  .configurationVariables.C_cell_ot               = C_CELL_OT_DEFAULT, 
  .configurationVariables.C_cell_ot_charge        = C_CELL_OT_CHARGE_DEFAULT, 
  .configurationVariables.C_cell_ut               = C_CELL_UT_DEFAULT, 
  .configurationVariables.C_cell_ut_charge        = C_CELL_UT_CHARGE_DEFAULT, 
  .configurationVariables.A_factory               = A_FACTORY_DEFAULT, 
  .configurationVariables.t_bms_timeout           = T_BMS_TIMEOUT_DEFAULT, 
  .configurationVariables.t_fault_timeout         = T_FAULT_TIMEOUT_DEFAULT, 
  .configurationVariables.t_charge_detect         = T_CHARGE_DETECT_DEFAULT, 
  .configurationVariables.t_cb_delay              = T_CB_DELAY_DEFAULT, 
  .configurationVariables.t_charge_relax          = T_CHARGE_RELAX_DEFAULT, 
  .configurationVariables.I_charge_full           = I_CHARGE_FULL_DEFAULT, 
  .configurationVariables.I_charge_max            = I_CHARGE_MAX_DEFAULT, 
  .configurationVariables.I_out_max               = I_OUT_MAX_DEFAULT, 
  .configurationVariables.V_cell_margin           = V_CELL_MARGIN_DEFAULT, 
  .configurationVariables.t_ocv_cyclic0           = T_OCV_CYCLIC0_DEFAULT, 
  .configurationVariables.t_ocv_cyclic1           = T_OCV_CYCLIC1_DEFAULT, 
  .configurationVariables.C_pcb_ut                = C_PCB_UT_DEFAULT, 
  .configurationVariables.C_pcb_ot                = C_PCB_OT_DEFAULT, 
  .configurationVariables.V_storage               = V_STORAGE_DEFAULT, 
  .configurationVariables.ocv_slope               = OCV_SLOPE_DEFAULT, 
  .configurationVariables.batt_eol                = BATT_EOL_DEFAULT, 
  .configurationVariables.sensor_enable           = SENSOR_ENABLE_DEFAULT, 
  .configurationVariables.self_discharge_enable   = SELF_DISCHARGE_ENABLE_DEFAULT, 
  .configurationVariables.Uavcan_node_static_id   = UAVCAN_NODE_STATIC_ID_DEFAULT, 
  .configurationVariables.Uavcan_subject_id       = UAVCAN_SUBJECT_ID_DEFAULT, 
  .configurationVariables.Uavcan_fd_mode          = UAVCAN_FD_MODE_DEFAULT, 
  .configurationVariables.Uavcan_bitrate          = UAVCAN_BITRATE_DEFAULT, 
  .configurationVariables.Uavcan_fd_bitrate       = UAVCAN_FD_BITRATE_DEFAULT, 

  .hardwareVariables.V_min                        = V_MIN_DEFAULT, 
  .hardwareVariables.V_max                        = V_MAX_DEFAULT, 
  .hardwareVariables.I_peak                       = I_PEAK_DEFAULT, 
  .hardwareVariables.I_max                        = I_MAX_DEFAULT, 
  .hardwareVariables.I_short                      = I_SHORT_DEFAULT, 
  .hardwareVariables.t_short                      = T_SHORT_DEFAULT, 
  .hardwareVariables.I_bal                        = I_BAL_DEFAULT 
};

const BMSparametersInfo_t s_parametersInfo[NONE] = 
{
   { SET_DEFAULT2(FLOATVAL,  true,  true,  C_BATT, basicVariables.C_batt) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  V_OUT, basicVariables.V_out) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  V_BATT, basicVariables.V_batt) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  I_BATT, basicVariables.I_batt) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  I_BATT_AVG, basicVariables.I_batt_avg) },
   { SET_DEFAULT2(UINT8VAL,  true, false,  S_OUT, basicVariables.s_out) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  P_AVG, basicVariables.P_avg) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  E_USED, basicVariables.E_used) },
   { SET_DEFAULT2(FLOATVAL,  false, true,  A_REM, basicVariables.A_rem) },
   { SET_DEFAULT2(FLOATVAL,  false, true,  A_FULL, basicVariables.A_full) },
   { SET_DEFAULT2(FLOATVAL,  false, true,  T_FULL, basicVariables.t_full) },
   { SET_DEFAULT2(UINT8VAL,  false, false, S_FLAGS, basicVariables.s_flags) },
   { SET_DEFAULT2(UINT8VAL,  true,  false, S_HEALTH, basicVariables.s_health) },
   { SET_DEFAULT2(UINT8VAL,  true,  false, S_CHARGE, basicVariables.s_charge) },
   { SET_DEFAULT2(UINT8VAL,  false, false, BATT_ID, basicVariables.batt_id) },
   { SET_DEFAULT2(INT32VAL,  false, false, MODEL_ID, basicVariables.model_id) },
   { SET_DEFAULT2(STRINGVAL, false, false, MODEL_NAME, basicVariables.model_name) },

   { SET_DEFAULT2(FLOATVAL,  true,  true,  V_CELL1, additionalVariables.V_cell1) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  V_CELL2, additionalVariables.V_cell2) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  V_CELL3, additionalVariables.V_cell3) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  V_CELL4, additionalVariables.V_cell4) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  V_CELL5, additionalVariables.V_cell5) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  V_CELL6, additionalVariables.V_cell6) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  C_AFE, additionalVariables.C_AFE) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  C_T, additionalVariables.C_T) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  C_R, additionalVariables.C_R) },
   { SET_DEFAULT2(UINT16VAL, false, false, N_CHARGES, additionalVariables.N_charges) },
   { SET_DEFAULT2(UINT16VAL, false, false, N_CHARGES_FULL, additionalVariables.N_charges_full) },

   { SET_DEFAULT2(UINT8VAL,  true,  true,  N_CELLS, configurationVariables.N_cells) },
   { SET_DEFAULT2(UINT16VAL, false, true,  T_MEAS, configurationVariables.t_meas) },
   { SET_DEFAULT2(UINT16VAL, false, true,  T_FTTI, configurationVariables.t_ftti) },
   { SET_DEFAULT2(UINT8VAL,  false, true,  T_CYCLIC, configurationVariables.t_cyclic) },
   { SET_DEFAULT2(UINT8VAL,  false, true,  I_SLEEP_OC, configurationVariables.I_sleep_oc) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  V_CELL_OV, configurationVariables.V_cell_ov) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  V_CELL_UV, configurationVariables.V_cell_uv) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  C_CELL_OT, configurationVariables.C_cell_ot) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  C_CELL_OT_CHARGE, configurationVariables.C_cell_ot_charge) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  C_CELL_UT, configurationVariables.C_cell_ut) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  C_CELL_UT_CHARGE, configurationVariables.C_cell_ut_charge) },
   { SET_DEFAULT2(FLOATVAL,  false, true,  A_FACTORY, configurationVariables.A_factory) },
   { SET_DEFAULT2(UINT16VAL, false, true,  T_BMS_TIMEOUT, configurationVariables.t_bms_timeout) },
   { SET_DEFAULT2(UINT8VAL,  true,  true,  T_FAULT_TIMEOUT, configurationVariables.t_fault_timeout) },
   { SET_DEFAULT2(UINT8VAL,  false, true,  T_CHARGE_DETECT, configurationVariables.t_charge_detect) },
   { SET_DEFAULT2(UINT8VAL,  false, true,  T_CB_DELAY, configurationVariables.t_cb_delay) },
   { SET_DEFAULT2(UINT16VAL, false, true,  T_CHARGE_RELAX, configurationVariables.t_charge_relax) },
   { SET_DEFAULT2(UINT16VAL, false, true,  I_CHARGE_FULL, configurationVariables.I_charge_full) },
   { SET_DEFAULT2(FLOATVAL,  false, true,  I_CHARGE_MAX, configurationVariables.I_charge_max) },
   { SET_DEFAULT2(FLOATVAL,  false, true,  I_OUT_MAX, configurationVariables.I_out_max) },
   { SET_DEFAULT2(UINT8VAL,  false, true,  V_CELL_MARGIN, configurationVariables.V_cell_margin) },
   { SET_DEFAULT2(INT32VAL,  false, true,  T_OCV_CYCLIC0, configurationVariables.t_ocv_cyclic0) },
   { SET_DEFAULT2(INT32VAL,  false, true,  T_OCV_CYCLIC1, configurationVariables.t_ocv_cyclic1) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  C_PCB_UT, configurationVariables.C_pcb_ut) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  C_PCB_OT, configurationVariables.C_pcb_ot) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  V_STORAGE, configurationVariables.V_storage) },
   { SET_DEFAULT2(FLOATVAL,  true,  true,  OCV_SLOPE, configurationVariables.ocv_slope) },
   { SET_DEFAULT2(UINT8VAL,  true,  false, BATT_EOL, configurationVariables.batt_eol) },
   { SET_DEFAULT2(UINT8VAL,  true,  true,  SENSOR_ENABLE, configurationVariables.sensor_enable) },
   { SET_DEFAULT2(UINT8VAL,  true,  true,  SELF_DISCHARGE_ENABLE, configurationVariables.self_discharge_enable) },
   { SET_DEFAULT2(UINT8VAL,  true,  true,  UAVCAN_NODE_STATIC_ID, configurationVariables.Uavcan_node_static_id) },
   { SET_DEFAULT2(UINT16VAL, true,  true,  UAVCAN_SUBJECT_ID, configurationVariables.Uavcan_subject_id) },
   { SET_DEFAULT2(UINT8VAL,  true,  true,  UAVCAN_FD_MODE, configurationVariables.Uavcan_fd_mode) },
   { SET_DEFAULT2(INT32VAL,  true,  true,  UAVCAN_BITRATE, configurationVariables.Uavcan_bitrate) },
   { SET_DEFAULT2(INT32VAL,  true,  true,  UAVCAN_FD_BITRATE, configurationVariables.Uavcan_fd_bitrate) },

   { SET_DEFAULT2(UINT8VAL,  true, true,   V_MIN, hardwareVariables.V_min) },
   { SET_DEFAULT2(UINT8VAL,  true, true,   V_MAX, hardwareVariables.V_max) },
   { SET_DEFAULT2(UINT16VAL, true, true,   I_PEAK, hardwareVariables.I_peak) },
   { SET_DEFAULT2(UINT8VAL,  true, true,   I_MAX, hardwareVariables.I_max) },
   { SET_DEFAULT2(UINT16VAL, true, true,   I_SHORT, hardwareVariables.I_short) },
   { SET_DEFAULT2(UINT8VAL,  false, true,  T_SHORT, hardwareVariables.t_short) },
   { SET_DEFAULT2(UINT8VAL,  false, false, I_BAL, hardwareVariables.I_bal) }
};

//! these are the units for each parameter in the right sequence
const char *parameterUnits[] = 
{
  "C",
  "V",
  "V",
  "A",
  "A",
  "-",
  "W",
  "Wh",
  "Ah",
  "Ah",
  "h",
  "-",
  "%",
  "%",
  "-",
  "-",
  "-",
  "V",
  "V",
  "V",
  "V",
  "V",
  "V",
  "C",
  "C",
  "C",
  "-",
  "-",
  "-",
  "ms",
  "ms",
  "s",
  "mA",
  "V",
  "V",
  "C",
  "C",
  "C",
  "C",
  "Ah",
  "s",
  "s",
  "s",
  "s",
  "s",
  "mA",
  "A",
  "A",
  "mV",
  "s",
  "s",
  "C",
  "C",
  "V",
  "V/A.min",
  "%",
  "-",
  "-",
  "-",
  "-",
  "-",
  "bit/s",
  "bit/s",
  "V",
  "V",
  "A",
  "A",
  "A",
  "us",
  "mA"
};

// test_t test = 
// { 
//    .parametersInfo.type                                       = U8,
//   .parametersInfo.checkMax                                   = true,
//   .parametersInfo.checkMin                                   = true,
//   .parametersInfo.max                                        = S_CHARGE_MAX,
//   .parametersInfo.min                                        = S_CHARGE_MIN,
//   .parametersInfo.parameterAdr                               = &s_parameters.parameters.basicVariables.state_of_charge
// };


/****************************************************************************
 * private Functions
 ****************************************************************************/
/*!
 * @brief     this function will set the default values of the BMS to the data struct
 *      
 * @param     none
 *
 * @return    none
 */
void data_setDefault(void);

/****************************************************************************
 * public Functions
 ****************************************************************************/

/*!
 * @brief     this function is needed to use the data_setParameter and data_getParameter
 *        it will initialize the mutex and return the outcome of the pthread_mutex_init function
 *      
 * @param     p_parameterChangeCallbackFunction the address of the function to start a task to handle a parameter change
 *
 * @return    If successful, the function will return zero (OK). Otherwise, an error number will be returned to indicate the error:
 * @example   if(data_initialize())
 *        {
 *          // do something with the error
 *        }
 */
int data_initialize(parameterChangeCallbackFunction p_parameterChangeCallbackFunction)
{
  int lvRetValue = !gDataInitialized;

  if(!gDataInitialized)
  {
    cli_printf("SELF-TEST START: FLASH\n");

    // connect the callback function
    g_parameterChangeCallbackFunctionfp = p_parameterChangeCallbackFunction;

  	// initialize the mutex lock, otherwise it will not work
  	lvRetValue = pthread_mutex_init(&dataLock, NULL);

    // set the default values
    //data_setDefault();
    //cli_printf("data struct size: %d\n", sizeof(s_parameters)/sizeof(uint8_t));
    //cli_printf("parameters struct size: %d\n", sizeof(s_parameters.parameters)/sizeof(uint8_t));

    if(lvRetValue != 0)
    {
      cli_printf("data_initialize ERROR: couldn't init data mutex!\n");
      return lvRetValue;
    }

     // initialize flash mutex
    lvRetValue = pthread_mutex_init(&flashLock, NULL);

    // check for errors
    if(lvRetValue != 0)
    {
      cli_printf("data_initialize ERROR: couldn't init flash mutex!\n");
      return lvRetValue;
    }

    // set the value to true
    gFlashInitialized = true;

    // set the variable true
    gDataInitialized = true;

    // try to load the parameters, it will go wrong the first time and load the default ones
    if(data_loadParameters())
    {
      cli_printf("nothing/wrong saved!\n");
    }

    cli_printf("SELF-TEST PASS:  FLASH\n");
  }

  return lvRetValue;
}

/*!
 * @brief     function to the type a certain parameter from the data struct in data.c
 *            this function could be used before the data_setParameter or data_getParameter function
 *     
 * @param     parameterKind the parameter of which the type should be returned, from the parameterKind enum in BMS_data_types.h
 *
 * @retval    the type of the parameter as a valueType_t enum value, defined in BMS_data_types.h
 *            it will return STRINGVAL if the parameterKind was NONE
 *             
 * @example   valueType_t paramType;
 *            switch(paramType) 
 *            {
 *              case FLOATVAL: // so somthing with floating points values
 *              break;
 *              case STRINGVAL: // do somthing with string values
 *              break;
 *              case default: // something with int32_t values (uint16 and uint8 can be set and get with int32)
 *              break;
 *            }
 *        
 */
valueType_t data_getType(parameterKind_t parameterKind)
{
  // use the most not used value as an error
  valueType_t lvRetValue = STRINGVAL;

  // check if not NONE
  if (parameterKind != NONE)
  {
    // get the type 
    lvRetValue = s_parametersInfo[parameterKind].type;
  }

  // return the value
  return lvRetValue;
}

/*!
 * @brief       function to get a certain parameter from the data struct in data.c
 *              this function could be used after the data_setParameter function
 *              there are 2 ways to get the parameter, either with the outData parameter 
 *              or with the return value of the function. other may be NULL
 *              the outLenght parameter is used get the lenght of the data in bytes, this may be NULL
 *              data_initializeData should have been called once
 *     
 * @param       parameterKind the parameter value it wants, from the parameterKind enum in BMS_data_types.h
 * @param       outData pointer to the value that needs to become the parameter value, could be int32_t, float or char*
 * @param       outLenght pointer to the value that needs to become the lenght of the data (in bytes) 
 *              only used with characters, otherwise it may be NULL
 *
 * @retval      a void pointer to the value 
 * @example     int32_t lvGetParam; 
 *              uint8_t *pReadUint8_tData = NULL;
 *              pReadUint8_tData = data_getParameter(S_CHARGE, &lvGetParam, NULL);
 *              if(pReadUint8_tData != NULL)
 *              {
 *                // do something with value
 *              }
 *              else
 *              {
 *                // something went wrong!
 *              }
 *              other example:
 *              char  *GetString = malloc(sizeof(char[32]));
 *              uint16_t *Lenght = malloc(sizeof(uint16_t));
 *              data_getParameter(MODEL_NAME, GetString, Lenght);
 *              free(GetString);
 *              free(Lenght); 
 */
void* data_getParameter(parameterKind_t parameterKind, void* outData, uint16_t* outLength)
{

  // make a return variable
  void* lvRetValue = NULL;

  // check if wrong input
  if(parameterKind == NONE)
  {
    cli_printf("ERROR: wrong input!\n");
    return lvRetValue;
  }

  // check if initialized
  if(!gDataInitialized)
  {
    cli_printf("ERROR: initialize mutex!\n");
    return lvRetValue;
  }

  // lock the mutex(with error check)
  if ((pthread_mutex_lock(&dataLock)) != 0)
  {
    cli_printf("ERROR: pthread_mutex_lock failed\n");
    return lvRetValue;
  }

  // this switch will check which type it is
  // it will assign the lenght of the data 
  // it will set the pointer of lvRetValue to the right data of the s_parameters struct
  // it will assign outData with the value of lvRetValue
  switch(s_parametersInfo[parameterKind].type)
  {
    // in case it is a floatvalue
    case FLOATVAL:  
                    lvRetValue = (float*)s_parametersInfo[parameterKind].parameterAdr;
                    if(outLength != NULL) *outLength = sizeof(*lvRetValue);
                    if(outData != NULL) (*(float*)outData = *(float*)lvRetValue);
    break;
    // in case it is a uint8_t value
    case UINT8VAL:  lvRetValue = (uint8_t*)s_parametersInfo[parameterKind].parameterAdr;
                    if(outLength != NULL) *outLength = sizeof(*lvRetValue);
                    if(outData != NULL) (*(uint8_t*)outData = *(uint8_t*)lvRetValue);
                   
    break;
    // in case it is a bool value
    // case BOOLVAL:   lvRetValue = (bool*)s_parametersInfo[parameterKind].parameterAdr;
    //                 if(outLength != NULL) *outLength = sizeof(*lvRetValue);
    //                 if(outData != NULL) (*(bool*)outData = *(bool*)lvRetValue);
                   
    // break;
    // in case it is a uint16_t value
    case UINT16VAL: lvRetValue = (uint16_t*)s_parametersInfo[parameterKind].parameterAdr;
                    if(outLength != NULL) *outLength = sizeof(*lvRetValue);
                    if(outData != NULL) (*(uint16_t*)outData = *(uint16_t*)lvRetValue);
                    

    break;
    // in case it is a int32_t value
    case INT32VAL:  lvRetValue = (int32_t*)s_parametersInfo[parameterKind].parameterAdr;
                    if(outLength != NULL) *outLength = sizeof(*lvRetValue);
                    if(outData != NULL) (*(int32_t*)outData = *(int32_t*)lvRetValue);
                    

    break;
    // in case it is a string value
    case STRINGVAL: lvRetValue = &s_parameters.basicVariables.model_name;
                    if(outLength != NULL) *outLength = strlen(s_parameters.basicVariables.model_name);
                    if(outData != NULL) strncpy(((char*)outData), s_parameters.basicVariables.model_name, *outLength); 
                    
    break;
    // just in case
    default:
    break;
  }                            

  // unlock the mutex after writing is done
  if ((pthread_mutex_unlock(&dataLock)) != 0)
  {
    cli_printf("ERROR: pthread_mutex_unlock failed\n");
    return NULL;
  } 

  return lvRetValue;
}


/*!
 * @brief     function to set a certain parameter in the data struct
 *            after this, the value can be read with the data_setParameter funciton
 *            this function only sets the new value if it is within the range 
 *            of the parameter as discribed in BMS_data_limits.h
 *            it is possible to signal if a parameter has changed (work in progress!)
 *            data_initialize should have been called once     
 * 
 * @param     parameterKind the setting it wants to set from the parameterKind enum in BMS_data_types.h
 * @param     inNewValue a pointer to the value it will be

 *
 * @retval    is -1 when something went wrong, 0 when it went right
 * @example:  int32_t newValue = 3; or float newValue = 3.3
 *            if(data_setParameter(S_CHARGE, &newValue))
 *            {
 *              // it went wrong!
 *            }
 *  
 *            for model name:
 *            char newModelName[] = "New model name\0";
 *            if(data_setParameter(MODEL_NAME, newModelName))
 *            {
 *              // it went wrong!
 *            }
 */
int data_setParameter(parameterKind_t parameterKind, void* inNewValue)
{
  // the return varaiable
  int lvRetValue = -1;
  //int lvErrcode;

  //static char * dataChangeArg[] = {"98", NULL};
  //char *dataChange = "99";
  //char *theData = "0000000000000";
  checkLimit_t lvCheckLimit;

  // variable to check if the variable has changed
  bool lvChanged = false;
  
  // check for a void pointer and right input
  if(inNewValue == NULL || parameterKind == NONE)
  {
    cli_printf("ERROR: wrong input!\n");
    return lvRetValue;
  }

  // check if initialized
  if(!gDataInitialized)
  {
    return lvRetValue;
  }

  // lock the mutex(with error check)
  if ((pthread_mutex_lock(&dataLock)) != 0)
  {
    cli_printf("ERROR: pthread_mutex_lock failed\n");
    return lvRetValue;
  }  

  // make the enum for the switch
  lvCheckLimit = (checkLimit_t)((s_parametersInfo[parameterKind].checkMax << 1) | 
    (s_parametersInfo[parameterKind].checkMin));

  // this switch will check if the parameter is within its limits as stated in "BMS_data_limits.h"
  // if is is within the limits it will assign the value to the s_parameters struct
  // and it will set lvRetValue to 0, meaning it went right
  // before setting the parameter it will check if the old en new parameters are different, if so lvChanged = true;
  switch(s_parametersInfo[parameterKind].type)
  {
    // in case it is a floatvalue
    case FLOATVAL:  
                    // if((*(s_parameters.parametersInfo[parameterKind].parameterAdr) != (*(float*)inNewValue)))
                    //   (lvChanged = true); 
                    // else 
                    //   (lvChanged = false);

                    // check if there is a limit on it 
                    // this switch will check what check needs to be done with the limit 
                    // it will assign the right value
                    // it will set lvChanged high if the data is different
                    // it will set lvRetValue if succeeded
                    switch(lvCheckLimit)
                    {
                      // if the limit check need to be done on both high and low limit
                      case CHECK_BOTH:  CHECK_ASSIGN_BOTH(s_parametersInfo[parameterKind].max.FX10/(float)FLOAT_MULTIPIER,
                                        s_parametersInfo[parameterKind].min.FX10/(float)FLOAT_MULTIPIER,
                                        *(float*)(s_parametersInfo[parameterKind].parameterAdr), float)
                      break;
                      // if the limit check need to be done on low limit
                      case CHECK_LOW:   CHECK_ASSIGN_MIN(s_parametersInfo[parameterKind].min.FX10/(float)FLOAT_MULTIPIER, 
                                        *(float*)(s_parametersInfo[parameterKind].parameterAdr), float)
                      break;
                      // if the limit check need to be done on high limit
                      case CHECK_HIGH:  CHECK_ASSIGN_MAX(s_parametersInfo[parameterKind].max.FX10/(float)FLOAT_MULTIPIER, 
                                        *(float*)(s_parametersInfo[parameterKind].parameterAdr), float)
                      break;
                      // if the limit check shouldn't be done
                      case CHECK_NONE:  CHECK_ASSIGN_NONE(*(float*)(s_parametersInfo[parameterKind].parameterAdr), float)
                      break;
                    }

                    // // set the parameter
                    // (*(s_parameters.parametersInfo[parameterKind].parameterAdr) = (*(float*)inNewValue));
                    // lvRetValue = 0;
    break;
    // in case it is a uint8_t value
    case UINT8VAL:  // check if there is a limit on it 
                    // this switch will check what check needs to be done with the limit 
                    // it will assign the right value
                    // it will set lvChanged high if the data is different
                    // it will set lvRetValue if succeeded
                    switch(lvCheckLimit)
                    {
                      // if the limit check need to be done on both high and low limit
                      case CHECK_BOTH:  CHECK_ASSIGN_BOTH(s_parametersInfo[parameterKind].max.U8,
                                        s_parametersInfo[parameterKind].min.U8,
                                        *(uint8_t*)(s_parametersInfo[parameterKind].parameterAdr), uint8_t)
                      break;
                      // if the limit check need to be done on low limit
                      case CHECK_LOW:   CHECK_ASSIGN_MIN(s_parametersInfo[parameterKind].min.U8, 
                                        *(uint8_t*)(s_parametersInfo[parameterKind].parameterAdr), uint8_t)
                      break;
                      // if the limit check need to be done on high limit
                      case CHECK_HIGH:  CHECK_ASSIGN_MAX(s_parametersInfo[parameterKind].max.U8, 
                                        *(uint8_t*)(s_parametersInfo[parameterKind].parameterAdr), uint8_t)
                      break;
                      // if the limit check shouldn't be done
                      case CHECK_NONE:  CHECK_ASSIGN_NONE(*(uint8_t*)(s_parametersInfo[parameterKind].parameterAdr), uint8_t)
                      break;
                    }
    break;
        // in case it is a bool value
    // case BOOLVAL:  // check if there is a limit on it 
    //                 // this switch will check what check needs to be done with the limit 
    //                 // it will assign the right value
    //                 // it will set lvChanged high if the data is different
    //                 // it will set lvRetValue if succeeded
    //                 // switch(lvCheckLimit)
    //                 // {
    //                 //   // if the limit check need to be done on both high and low limit
    //                 //   case CHECK_BOTH:  CHECK_ASSIGN_BOTH(s_parametersInfo[parameterKind].max.BOL,
    //                 //                     s_parametersInfo[parameterKind].min.BOL,
    //                 //                     *(bool*)(s_parametersInfo[parameterKind].parameterAdr), bool)
    //                 //   break;
    //                 //   // if the limit check need to be done on low limit
    //                 //   case CHECK_LOW:   CHECK_ASSIGN_MIN(s_parametersInfo[parameterKind].min.U8, 
    //                 //                     *(bool*)(s_parametersInfo[parameterKind].parameterAdr), bool)
    //                 //   break;
    //                 //   // if the limit check need to be done on high limit
    //                 //   case CHECK_HIGH:  CHECK_ASSIGN_MAX(s_parametersInfo[parameterKind].max.U8, 
    //                 //                     *(bool*)(s_parametersInfo[parameterKind].parameterAdr), bool)
    //                 //   break;
    //                 //   // if the limit check shouldn't be done
    //                 //   case CHECK_NONE:  CHECK_ASSIGN_NONE(*(bool*)(s_parametersInfo[parameterKind].parameterAdr), bool)
    //                 //   break;
    //                 // }
    //                 // if the limit check shouldn't be done
    //                 CHECK_ASSIGN_NONE(*(bool*)(s_parametersInfo[parameterKind].parameterAdr), bool)
    // break;
    // in case it is a uint16_t value
    case UINT16VAL: // check if there is a limit on it 
                    // this switch will check what check needs to be done with the limit 
                    // it will assign the right value
                    // it will set lvChanged high if the data is different
                    // it will set lvRetValue if succeeded
                    switch(lvCheckLimit)
                    {
                      // if the limit check need to be done on both high and low limit
                      case CHECK_BOTH:  CHECK_ASSIGN_BOTH(s_parametersInfo[parameterKind].max.U16,
                                        s_parametersInfo[parameterKind].min.U16,
                                        *(uint16_t*)(s_parametersInfo[parameterKind].parameterAdr), uint16_t)
                      break;
                      // if the limit check need to be done on low limit
                      case CHECK_LOW:   CHECK_ASSIGN_MIN(s_parametersInfo[parameterKind].min.U16, 
                                        *(uint16_t*)(s_parametersInfo[parameterKind].parameterAdr), uint16_t)
                      break;
                      // if the limit check need to be done on high limit
                      case CHECK_HIGH:  CHECK_ASSIGN_MAX(s_parametersInfo[parameterKind].max.U16, 
                                        *(uint16_t*)(s_parametersInfo[parameterKind].parameterAdr), uint16_t)
                      break;
                      // if the limit check shouldn't be done
                      case CHECK_NONE:  CHECK_ASSIGN_NONE(*(uint16_t*)(s_parametersInfo[parameterKind].parameterAdr), uint16_t)
                      break;
                    }

    break;
    // in case it is a int32_t value
    case INT32VAL:  // check if there is a limit on it 
                    // this switch will check what check needs to be done with the limit 
                    // it will assign the right value
                    // it will set lvChanged high if the data is different
                    // it will set lvRetValue if succeeded
                    switch(lvCheckLimit)
                    {
                      // if the limit check need to be done on both high and low limit
                      case CHECK_BOTH:  CHECK_ASSIGN_BOTH(s_parametersInfo[parameterKind].max.FX10,
                                        s_parametersInfo[parameterKind].min.FX10,
                                        *(int32_t*)(s_parametersInfo[parameterKind].parameterAdr), int32_t)
                      break;
                      // if the limit check need to be done on low limit
                      case CHECK_LOW:   CHECK_ASSIGN_MIN(s_parametersInfo[parameterKind].min.FX10, 
                                        *(int32_t*)(s_parametersInfo[parameterKind].parameterAdr), int32_t)
                      break;
                      // if the limit check need to be done on high limit
                      case CHECK_HIGH:  CHECK_ASSIGN_MAX(s_parametersInfo[parameterKind].max.FX10, 
                                        *(int32_t*)(s_parametersInfo[parameterKind].parameterAdr), int32_t)
                      break;
                      // if the limit check shouldn't be done
                      case CHECK_NONE:  CHECK_ASSIGN_NONE(*(int32_t*)(s_parametersInfo[parameterKind].parameterAdr), int32_t)
                      break;
                    }

    break;
    // in case it is a string value
    case STRINGVAL:   if(strcmp(s_parameters.basicVariables.model_name, ((char*)inNewValue))) 
                        lvChanged = true;       
                      strncpy(s_parameters.basicVariables.model_name, ((char*)inNewValue), MODEL_NAME_MAX_CHARS); 
                      lvRetValue = 0;
    break;
    // just in case
    default:
    break;
  }

  // check if a parameter has changed
  if(lvChanged && lvRetValue == 0)
  {
    // call the callback function
    g_parameterChangeCallbackFunctionfp(parameterKind, inNewValue);

    // check which parameter has been changed if it needs to be saved
    if((!gSavableParameterChanged) && ((parameterKind == A_FULL) || (parameterKind == S_FLAGS) || (parameterKind == S_HEALTH)|| 
      (BATT_ID <= parameterKind && parameterKind <= MODEL_NAME) || (N_CHARGES <= parameterKind)))
    {
      //cli_printf("this parameter changed: %d", parameterKind);
      // set the parameter
      gSavableParameterChanged = true;
    }
  }

  // unlock the mutex after writing is done
  if ((pthread_mutex_unlock(&dataLock)) != 0)
  {
    lvRetValue = -1;
    cli_printf("ERROR: pthread_mutex_unlock failed\n");
    return lvRetValue;
  } 

  // return the value
  return lvRetValue;
}

/*!
 * @brief     function to lock the mutex (make sure that no other tasks/threads can acces/change the data)
 *            make sure the mutex is unlocked again with data_unlockMutex
 *            should always be used with data_unlockMutex
 *     
 * @param     none
 *
 * @retval    0 if succeeded, otherwise the error of pthread_mutex_lock
 *        
 * @example   uint8_t stateOfCharge; 
 *            data_lockMutex();
 *            stateOfCharge = *(data_getAdr(S_CHARGE));
 *            data_unlockMutex();
 *        
 */
int data_lockMutex(void)
{
  // lock the data mutex and return
  return pthread_mutex_lock(&dataLock);
}

/*!
 * @brief   function to unlock the mutex (make sure that other tasks/threads can acces/change the data)
 *          make sure the mutex is locked with the data_lockMutex function before calling this funciton
 *          should always be used with the data_lockMutex
 *     
 * @param   none
 *
 * @retval  0 if succeeded, otherwise the error of pthread_mutex_unlock
 *        
 * @example uint8_t stateOfCharge; 
 *          data_lockMutex();
 *          stateOfCharge = *(data_getAdr(S_CHARGE));
 *          data_unlockMutex();
 *        
 */
int data_unlockMutex(void)
{
  // unlock the data mutex and return
  return pthread_mutex_unlock(&dataLock);
}

/*!
 * @brief   function to get the address of a certain parameter from the data struct in data.c
 *          this function is faster than the get or set function
 *          this function should be used with the data_lockMutex() and data_unlockMutex() functions
 * 
 *     
 * @param   parameterKind the parameter of which the address should be returned, from the parameterKind enum in BMS_data_types.h
 *
 * @retval  an void pointer addres to the value
 * @warning be sure to lock te mutex before this function and unlock it after. 
 *          shouldn't be used to set a variable!
 *          use with caution, a variable could be changed with this function but there are no limit checks
 *          there is alno no check if the varaible has changed (the signal for other functions)
 *        
 * @example uint8_t stateOfCharge; 
 *          data_lockMutex();
 *          stateOfCharge = *(data_getAdr(S_CHARGE));
 *          data_unlockMutex();
 *        
 */
void* data_getAdr(parameterKind_t parameterKind)
{
  // if it is not NONE
  if(parameterKind != NONE)
  {
    // return the parameter address
    return s_parametersInfo[parameterKind].parameterAdr;
  }
  // else
  else 
  {
    // return NULL
    return NULL;
  }
}

/*!
 * @brief   function to get the unit as a strings of a certain parameter * 
 *     
 * @param   parameterKind the parameter of which the unit should be returned, 
 *          from the parameterKind enum in BMS_data_types.h
 *
 * @retval  an char pointer addres to the value
 *        
 */
char* data_getUnit(parameterKind_t parameterKind)
{
  // check if it is not in range
  if(parameterKind >= NONE)
  {
    // return the "-"
    return (char*)parameterUnits[5];
  }
  // if in range
  else
  {
    // return the unit
    return (char*)parameterUnits[parameterKind];
  }
}

/*!
 * @brief   function to set a bit in the status flags (s_flags)
 * @note    if the flags are S_FLAGS_UKNOWN, clear the s_flags first.
 *     
 * @param     bit the bit that needs to change in the s_flags variable (0-7) use the STATUS_*_BIT defines for it
 * @param     value the new value of this bit as a bool
 *
 * @retval    0 if succeeded, otherwise the error of pthread_mutex_unlock
 *        
 * @example   if(data_statusFlagBit(STATUS_OVERLOAD_BIT, 1))
 *        {
 *          // do something with the error
 *        }
 *        
 */
int data_statusFlagBit(uint8_t bit, bool value)
{
  int lvRetValue = 1;

  // check the input
  if(bit > STATUS_HIGHEST_BIT)
  {
    // return and output to user
    cli_printf("data_statusFlagBit ERROR: input bit: %d > %d\n", bit, STATUS_HIGHEST_BIT);
    return lvRetValue;
  } 

  // check if initialized
  if(!gDataInitialized)
  {
    return lvRetValue;
  }

  // lock the mutex(with error check)
  if ((pthread_mutex_lock(&dataLock)) != 0)
  {
    cli_printf("ERROR: pthread_mutex_lock failed\n");
    return lvRetValue;
  }  

  // check if the flags need to be cleared (if it was unknown)
  if(s_parameters.basicVariables.s_flags == S_FLAGS_UKNOWN)
  {
    // reset the flags
    s_parameters.basicVariables.s_flags = 0;
  }

  // TODO reset the STATUS_ASK_PARS_BIT when the service request is send or reset is send

  // check if setting the bit or clearing it
  if(value)
  {
    // set the new bit
    s_parameters.basicVariables.s_flags |= 1 << bit;
  }
  else
  {
    // clear the bit
    s_parameters.basicVariables.s_flags &= ~(1 << bit);
  }

  // unlock the mutex after writing is done
  if ((pthread_mutex_unlock(&dataLock)) != 0)
  {
    lvRetValue = -1;
    cli_printf("ERROR: pthread_mutex_unlock failed\n");
    return lvRetValue;
  }  

  // return
  return lvRetValue;
}

/*!
 * @brief     function to save the parameters in flash
 * @note      The eeeprom is used for this (4KB)
 * @note       Multi-thread protected
 * 
 * @retval    0 if it went OK, negative otherwise         
 */
int data_saveParameters(void)
{
  int lvRetValue = 0;
  int fd, writtenBytes, i; 
  uint32_t CRC = 1;
  uint8_t *CRCCalc;
  const uint32_t parSize = sizeof(s_parameters)/sizeof(int8_t);

  // check if initialized
  if(!gFlashInitialized)
  {
    // output to user
    cli_printf("data_saveParameters ERROR: not initialized!\n");

    lvRetValue -= 1;

    // return erro
    return lvRetValue;
  }

  // lock the datalock
  pthread_mutex_lock(&dataLock);

  // check if it needs to save anything 
  if(!gSavableParameterChanged)
  {
    // output to user
    cli_printf("data_saveParameters: parameters didn't change, nothing will be saved\n");

    lvRetValue -= 0;

    // unlock the datalock
    pthread_mutex_unlock(&dataLock);

    // return erro
    return lvRetValue;
  }

  // unlock the datalock
  pthread_mutex_unlock(&dataLock);

  // lock the mutex
  pthread_mutex_lock(&flashLock);

  // Open the eeprom device
  fd = open("/dev/eeeprom0", O_WRONLY);

  // lock the mutex(with error check)
  if ((pthread_mutex_lock(&dataLock)) != 0)
  {
    cli_printf("ERROR: pthread_mutex_lock failed\n");

    // close the filedescriptor
    close(fd);

    // unlock the mutex
    pthread_mutex_unlock(&flashLock);

    lvRetValue -= 2;

    // return
    return lvRetValue;
  }  

  // write the struct
  writtenBytes = write(fd, &s_parameters, parSize);

  //cli_printf("writtenBytes: %d\n", writtenBytes);

  // check for error
  if(writtenBytes != parSize)
  {
    // output to user
    cli_printf("data_saveParameters ERROR: could not write parameters! %d != %d\n", writtenBytes, parSize);

    lvRetValue -= 4;

    // return erro
    lvRetValue--;
  }
  else
  {
    // reset the save variable, because new variable have been saved 
    gSavableParameterChanged = false;
  }

  // set the crcclat to the beginning of the parameters
  CRCCalc = (uint8_t*)(&s_parameters);

  // calculate CRC
  for(i = 0; i < parSize; i++)
  {
    // sum all the values
    CRC += *CRCCalc;

    // increment the address of the pointer
    CRCCalc++;
  }

  // unlock the mutex(with error check)
  if ((pthread_mutex_unlock(&dataLock)) != 0)
  {
    cli_printf("ERROR: pthread_mutex_unlock failed\n");

    // close the filedescriptor
    close(fd);

    // unlock the mutex
    pthread_mutex_unlock(&flashLock);

    lvRetValue -= 8;

    // return
    return lvRetValue;
  }  

  //cli_printf("CRC: %d\n", CRC);

  // write the CRC
  writtenBytes = write(fd, &CRC, sizeof(CRC)/sizeof(uint8_t));

  // check for error
  if(writtenBytes != sizeof(CRC)/sizeof(uint8_t))
  {
    // output to user
    cli_printf("data_saveParameters ERROR: could not write CRC! %d != %d\n", writtenBytes, 
      sizeof(CRC)/sizeof(uint8_t));

    // return error
    lvRetValue -= 16;

    // reset eeprom?
  }

  // close the filedescriptor
  close(fd);

  // unlock the mutex
  pthread_mutex_unlock(&flashLock);

  // return to user
  return lvRetValue;
}

/*!
 * @brief     function to load the parameters from flash
 * @note      The eeeprom is used for this (4KB)
 * @note      Multi-thread protected
 * 
 * @retval    0 if it went OK, negative otherwise         
 */
int data_loadParameters(void)
{
  int lvRetValue = 0;

  int fd, i, readBytes;
  uint32_t CRCR = 1, CRCW = 1;
  uint8_t *CRCCalc;
  const uint32_t parSize = sizeof(s_parameters)/sizeof(uint8_t);
  BMSParameterValues_t oldParameters;

  // check if initialized
  if(!gFlashInitialized)
  {
    // output to user
    cli_printf("data_saveParameters ERROR: not initialized!\n");

    lvRetValue -= 1;

    // return erro
    return lvRetValue;
  }

  // lock the mutex
  pthread_mutex_lock(&flashLock);

  // make read only
  fd = open("/dev/eeeprom0", O_RDONLY);

  // lock the mutex(with error check)
  if ((pthread_mutex_lock(&dataLock)) != 0)
  {
    cli_printf("ERROR: pthread_mutex_lock failed\n");

     // close file descriptor
    close(fd);

    // unlock the mutex
    pthread_mutex_unlock(&flashLock);

    lvRetValue -= 2;

    // return to user
    return lvRetValue;
  }  

  // save the old values
  oldParameters = s_parameters;

  // read the paramters 
  readBytes = read(fd, &s_parameters, parSize);

  //cli_printf("readBytes %d\n", readBytes);

  // check if the bytes to read is ok
  if(readBytes != parSize)
  {
    // output to user
    cli_printf("data_saveParameters ERROR: could not read parameters!\n");

    // return erro
    lvRetValue -= 8;

    // reset eeprom and save default values in call
  }

  // // check the CRC
  readBytes = read(fd, &CRCR, sizeof(CRCR)/sizeof(uint8_t));

  //cli_printf("CRCR: %d readBytes %d\n", CRCR, readBytes);

  // check if the bytes to read is ok
  if(readBytes != sizeof(CRCR)/sizeof(uint8_t))
  {
    // output to user
    cli_printf("data_saveParameters ERROR: could not read CRC!\n");

    // return erro
    lvRetValue -= 16;
    // reset eeprom and save default values in call
  }

  CRCCalc = (uint8_t*)&s_parameters;

  // calculate CRC
  for(i = 0; i < parSize; i++)
  {
    // sum all the values
    CRCW += *CRCCalc;

    // increment the address of the pointer
    CRCCalc++;
  }

  //cli_printf("CRCW: %d\n", CRCW);

  // check CRC
  if(CRCR != CRCW)
  {
    // output to user
    cli_printf("CRC of saved data doesn't match!\n");

    // return erro
    lvRetValue -= 32;

    cli_printf("Setting old values!\n");

    // save the old values
    s_parameters = oldParameters;
  }

  // unlock the mutex(with error check)
  if ((pthread_mutex_unlock(&dataLock)) != 0)
  {
    cli_printf("ERROR: pthread_mutex_unlock failed\n");

     // close file descriptor
    close(fd);

    // unlock the mutex
    pthread_mutex_unlock(&flashLock);

    lvRetValue -= 4;

    // return to user
    return lvRetValue;
  }  

  // close file descriptor
  close(fd);

  // unlock the mutex
  pthread_mutex_unlock(&flashLock);

  // return to user
  return lvRetValue;
}


/*!
 * @brief     this function will set the default values of the BMS to the data struct
 * @note      Multi-thread protected
 *      
 * @param     none
 *
 * @return    0 if succeeded, negative otherwise
 */
int data_setDefaultParameters(void)
{
  int lvRetValue = -1;
  int32_t intVariable;
  float floatVariable; 
  char charVariable[] = MODEL_NAME_DEFAULT;
  parameterKind_t parameter;
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = BATT_ID_DEFAULT;
  parameter = BATT_ID;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                          
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = MODEL_ID_DEFAULT;
  parameter = MODEL_ID;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                         
  
  //set the new value and set it in the data struct with the set function to make sure it is handled
  //intVariable = MODEL_NAME_DEFAULT;
  parameter = MODEL_NAME;
  lvRetValue = data_setParameter(parameter, charVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                   
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = N_CELLS_DEFAULT;
  parameter = N_CELLS;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                  
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = T_MEAS_DEFAULT;
  parameter = T_MEAS;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                   
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = T_FTTI_DEFAULT;
  parameter = T_FTTI;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                   
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = T_CYCLIC_DEFAULT;
  parameter = T_CYCLIC;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                 
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = I_SLEEP_OC_DEFAULT;
  parameter = I_SLEEP_OC;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);               
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  floatVariable = V_CELL_OV_DEFAULT;
  parameter = V_CELL_OV;
  lvRetValue = data_setParameter(parameter, &floatVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  floatVariable = V_CELL_UV_DEFAULT;
  parameter = V_CELL_UV;
  lvRetValue = data_setParameter(parameter, &floatVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  floatVariable = C_CELL_OT_DEFAULT;
  parameter = C_CELL_OT;
  lvRetValue = data_setParameter(parameter, &floatVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  floatVariable = C_CELL_OT_CHARGE_DEFAULT;
  parameter = C_CELL_OT_CHARGE;
  lvRetValue = data_setParameter(parameter, &floatVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);         
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  floatVariable = C_CELL_UT_DEFAULT;
  parameter = C_CELL_UT;
  lvRetValue = data_setParameter(parameter, &floatVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  floatVariable = C_CELL_UT_CHARGE_DEFAULT;
  parameter = C_CELL_UT_CHARGE;
  lvRetValue = data_setParameter(parameter, &floatVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);         
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  floatVariable = A_FACTORY_DEFAULT;
  parameter = A_FACTORY;
  lvRetValue = data_setParameter(parameter, &floatVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = T_BMS_TIMEOUT_DEFAULT;
  parameter = T_BMS_TIMEOUT;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);            
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = T_FAULT_TIMEOUT_DEFAULT;
  parameter = T_FAULT_TIMEOUT;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);          
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = T_CHARGE_DETECT_DEFAULT;
  parameter = T_CHARGE_DETECT;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);          
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = T_CB_DELAY_DEFAULT;
  parameter = T_CB_DELAY;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);               
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = T_CHARGE_RELAX_DEFAULT;
  parameter = T_CHARGE_RELAX;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);           
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = I_CHARGE_FULL_DEFAULT;
  parameter = I_CHARGE_FULL;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);            
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  floatVariable = I_CHARGE_MAX_DEFAULT;
  parameter = I_CHARGE_MAX;
  lvRetValue = data_setParameter(parameter, &floatVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);             
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  floatVariable = I_OUT_MAX_DEFAULT;
  parameter = I_OUT_MAX;
  lvRetValue = data_setParameter(parameter, &floatVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = V_CELL_MARGIN_DEFAULT;
  parameter = V_CELL_MARGIN;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);            
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = T_OCV_CYCLIC0_DEFAULT;
  parameter = T_OCV_CYCLIC0;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);            
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = T_OCV_CYCLIC1_DEFAULT;
  parameter = T_OCV_CYCLIC1;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);            
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  floatVariable = C_PCB_UT_DEFAULT;
  parameter = C_PCB_UT;
  lvRetValue = data_setParameter(parameter, &floatVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                 
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  floatVariable = C_PCB_OT_DEFAULT;
  parameter = C_PCB_OT;
  lvRetValue = data_setParameter(parameter, &floatVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                 
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  floatVariable = V_STORAGE_DEFAULT;
  parameter = V_STORAGE;
  lvRetValue = data_setParameter(parameter, &floatVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  floatVariable = OCV_SLOPE_DEFAULT;
  parameter = OCV_SLOPE;
  lvRetValue = data_setParameter(parameter, &floatVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = BATT_EOL_DEFAULT;
  parameter = BATT_EOL;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                 
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = SENSOR_ENABLE_DEFAULT;
  parameter = SENSOR_ENABLE;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);            
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = SELF_DISCHARGE_ENABLE_DEFAULT;
  parameter = SELF_DISCHARGE_ENABLE;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);    
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = UAVCAN_NODE_STATIC_ID_DEFAULT;
  parameter = UAVCAN_NODE_STATIC_ID;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);    
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = UAVCAN_SUBJECT_ID_DEFAULT;
  parameter = UAVCAN_SUBJECT_ID;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);        
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = UAVCAN_FD_MODE_DEFAULT;
  parameter = UAVCAN_FD_MODE;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);           
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = UAVCAN_BITRATE_DEFAULT;
  parameter = UAVCAN_BITRATE;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);           
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = UAVCAN_FD_BITRATE_DEFAULT;
  parameter = UAVCAN_FD_BITRATE;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);        
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = V_MIN_DEFAULT;
  parameter = V_MIN;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                         
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = V_MAX_DEFAULT;
  parameter = V_MAX;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                         
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = I_PEAK_DEFAULT;
  parameter = I_PEAK;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                        
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = I_MAX_DEFAULT;
  parameter = I_MAX;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                         
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = I_SHORT_DEFAULT;
  parameter = I_SHORT;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                       
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = T_SHORT_DEFAULT;
  parameter = T_SHORT;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                       
  
  // set the new value and set it in the data struct with the set function to make sure it is handled
  intVariable = I_BAL_DEFAULT;
  parameter = I_BAL;
  lvRetValue = data_setParameter(parameter, &intVariable);
  if(lvRetValue)
  {
    cli_printf("cli ERROR: couldn't set default variable! par: %d error: %d\n", parameter, lvRetValue);
  }
  // sleep for a short time to make sure it is handeld
  usleep(1);                         

  // set the return value
  lvRetValue = 0;

  // return to the user
  return lvRetValue;

}



// EOF

 