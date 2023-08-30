/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "toolhead_laser.h"

#include "../common/config.h"
#include "../common/debug.h"
#include "../gcode/M1028.h"

#include "../snapmaker.h"

// marlin headers
#include "src/core/macros.h"
#include "src/core/boards.h"
#include "Configuration.h"
#include "src/pins/pins.h"
#include "src/module/motion.h"
#include "src/module/planner.h"
#include "rotary_module.h"
#include "src/module/stepper.h"
#include "../service/system.h"
#include "toolhead_3dp.h"


#define LASER_CLOSE_FAN_DELAY     (300)
#define LASER_DISABLE_DELAY       (2)

#define TimSetPwm(n)  Tim1SetCCR4(n)
#define TimGetPwm()  Tim1GetCCR4()


ToolHeadLaser laser_1_6_w(MODULE_DEVICE_ID_1_6_W_LASER);
ToolHeadLaser laser_10w(MODULE_DEVICE_ID_10W_LASER);
ToolHeadLaser laser_20w(MODULE_DEVICE_ID_20W_LASER);
ToolHeadLaser laser_40w(MODULE_DEVICE_ID_40W_LASER);
ToolHeadLaser *laser = &laser_1_6_w;
ToolHeadLaser laser_array[LASER_AGING_INDEX_MAX] = {MODULE_DEVICE_ID_40W_LASER, MODULE_DEVICE_ID_40W_LASER, MODULE_DEVICE_ID_40W_LASER, MODULE_DEVICE_ID_40W_LASER,
                                                      MODULE_DEVICE_ID_40W_LASER, MODULE_DEVICE_ID_40W_LASER};

// int8_t laser_dir_pin[LASER_AGING_INDEX_MAX] = {E0_DIR_PIN, E1_DIR_PIN, X_DIR_PIN, Y_DIR_PIN, Z_DIR_PIN, B_DIR_PIN};
uint8_t  laser_init_total = 0;
uint8_t  rec_error[LASER_AGING_INDEX_MAX] = {0, 0, 0, 0, 0, 0};
uint8_t  mc_port[LASER_AGING_INDEX_MAX] = {0, 0, 0, 0, 0, 0};
uint32_t laser_open_time = 0;
uint32_t start_tick  = 3;
char port_info_str[LASER_AGING_INDEX_MAX][5] = {{"E0"}, {"E1"}, {"X "}, {"Y "}, {"Z "}, {"B "}};

static void CallbackAckReportSecurityIndex1(CanStdDataFrame_t &cmd);
static void CallbackAckReportSecurityIndex2(CanStdDataFrame_t &cmd);
static void CallbackAckReportSecurityIndex3(CanStdDataFrame_t &cmd);
static void CallbackAckReportSecurityIndex4(CanStdDataFrame_t &cmd);
static void CallbackAckReportSecurityIndex5(CanStdDataFrame_t &cmd);
static void CallbackAckReportSecurityIndex6(CanStdDataFrame_t &cmd);
static void CallbackAckReportFireSensorRawDataIndex1(CanStdDataFrame_t &cmd);
static void CallbackAckReportFireSensorRawDataIndex2(CanStdDataFrame_t &cmd);
static void CallbackAckReportFireSensorRawDataIndex3(CanStdDataFrame_t &cmd);
static void CallbackAckReportFireSensorRawDataIndex4(CanStdDataFrame_t &cmd);
static void CallbackAckReportFireSensorRawDataIndex5(CanStdDataFrame_t &cmd);
static void CallbackAckReportFireSensorRawDataIndex6(CanStdDataFrame_t &cmd);


static void (*call_back_report_security_array[LASER_AGING_INDEX_MAX])(CanStdDataFrame_t &cmd) = {
  CallbackAckReportSecurityIndex1,
  CallbackAckReportSecurityIndex2,
  CallbackAckReportSecurityIndex3,
  CallbackAckReportSecurityIndex4,
  CallbackAckReportSecurityIndex5,
  CallbackAckReportSecurityIndex6
};

static void (*call_back_report_fire_sensor_rawdata_array[LASER_AGING_INDEX_MAX])(CanStdDataFrame_t &cmd) = {
  CallbackAckReportFireSensorRawDataIndex1,
  CallbackAckReportFireSensorRawDataIndex2,
  CallbackAckReportFireSensorRawDataIndex3,
  CallbackAckReportFireSensorRawDataIndex4,
  CallbackAckReportFireSensorRawDataIndex5,
  CallbackAckReportFireSensorRawDataIndex6
};

extern void Tim1SetCCR4(uint16_t pwm);
extern uint16_t Tim1GetCCR4();
extern void Tim1PwmInit(unsigned short prescaler, unsigned short period);

static __attribute__((section(".data"))) uint8_t power_table_1_6W[]= {
  0,
  20,22,24,26,28,30,31,33,35,37,39,41,43,45,47,49,51,53,54,56,58,60,63,65,67,69,71,73,75,77,79,82,84,86,88,90,93,95,97,
  100,102,103,106,109,111,113,116,119,121,123,125,128,130,133,135,138,140,143,145,148,150,153,156,158,161,164,166,169,
  171,174,177,179,182,185,187,190,192,196,198,200,203,205,208,210,211,214,217,218,221,224,226,228,231,234,236,240,242,
  247,251,255
};

static __attribute__((section(".data"))) uint8_t power_table_10W[]= {
  0, 20, 27, 29, 32, 35, 37, 40, 42, 45,
  47, 49, 51, 54, 56, 59, 61, 63, 65, 68,
  70, 72, 75, 77, 79, 82, 84, 87, 90, 92,
  94, 97, 99, 101, 103, 106, 108, 110, 112, 115,
  117, 120, 122, 124, 126, 128, 131, 133, 135, 138,
  140, 142, 144, 147, 149, 151, 153, 156, 158, 161,
  163, 166, 168, 171, 173, 176, 178, 180, 182, 185,
  188, 190, 192, 193, 195, 198, 200, 202, 204, 207,
  209, 212, 214, 216, 218, 221, 224, 226, 228, 230,
  233, 235, 239, 241, 242, 245, 247, 250, 252, 254,
  255
};

static __attribute__((section(".data"))) uint8_t power_table_20W[]= {
  0, 20, 27, 29, 32, 35, 37, 40, 42, 45,
  47, 49, 51, 54, 56, 59, 61, 63, 65, 68,
  70, 72, 75, 77, 79, 82, 84, 87, 90, 92,
  94, 97, 99, 101, 103, 106, 108, 110, 112, 115,
  117, 120, 122, 124, 126, 128, 131, 133, 135, 138,
  140, 142, 144, 147, 149, 151, 153, 156, 158, 161,
  163, 166, 168, 171, 173, 176, 178, 180, 182, 185,
  188, 190, 192, 193, 195, 198, 200, 202, 204, 207,
  209, 212, 214, 216, 218, 221, 224, 226, 228, 230,
  233, 235, 239, 241, 242, 245, 247, 250, 252, 254,
  255
};

static __attribute__((section(".data"))) uint8_t power_table_40W[]= {
  0, 20, 27, 29, 32, 35, 37, 40, 42, 45,
  47, 49, 51, 54, 56, 59, 61, 63, 65, 68,
  70, 72, 75, 77, 79, 82, 84, 87, 90, 92,
  94, 97, 99, 101, 103, 106, 108, 110, 112, 115,
  117, 120, 122, 124, 126, 128, 131, 133, 135, 138,
  140, 142, 144, 147, 149, 151, 153, 156, 158, 161,
  163, 166, 168, 171, 173, 176, 178, 180, 182, 185,
  188, 190, 192, 193, 195, 198, 200, 202, 204, 207,
  209, 212, 214, 216, 218, 221, 224, 226, 228, 230,
  233, 235, 239, 241, 242, 245, 247, 250, 252, 254,
  255
};

// static void CallbackAckLaserFocus(CanStdDataFrame_t &cmd) {
//   laser->focus(cmd.data[0]<<8 | cmd.data[1]);
// }

// static void CallbackAckReportSecurity(CanStdDataFrame_t &cmd) {
//   laser->security_status_ = cmd.data[0];
//   laser->pitch_ = (cmd.data[1] << 8) | cmd.data[2];
//   laser->roll_ = (cmd.data[3] << 8) | cmd.data[4];
//   laser->laser_temperature_ = cmd.data[5];
//   laser->imu_temperature_ = (int8_t)cmd.data[6];
//   laser->fire_sensor_trigger_ = cmd.data[7];

//   laser->need_to_tell_hmi_ = true;

//   rec_error |= laser->security_status_;

//   if (laser->security_status_ != 0) {
//     laser->need_to_turnoff_laser_ = true;

//     if (systemservice.GetCurrentStage() == SYSTAGE_WORK || systemservice.GetCurrentStage() == SYSTAGE_PAUSE) {
//       quickstop.Trigger(QS_SOURCE_SECURITY, true);
//     } else if (laser->laser_status_ == LASER_ENABLE) {
//       laser->TurnOff();
//     }
//   }
// }

static void CallbackAckReportSecurityIndex1(CanStdDataFrame_t &cmd) {
  uint8_t index_ = 0;
  laser_array[index_].security_status_ = cmd.data[0];
  laser_array[index_].pitch_ = (cmd.data[1] << 8) | cmd.data[2];
  laser_array[index_].roll_ = (cmd.data[3] << 8) | cmd.data[4];
  laser_array[index_].laser_temperature_ = cmd.data[5];
  laser_array[index_].imu_temperature_ = (int8_t)cmd.data[6];
  laser_array[index_].fire_sensor_trigger_ = cmd.data[7];

  if (laser_init_total == index_ + 1)
    laser->need_to_tell_hmi_ = true;

  rec_error[index_] |= laser_array[index_].security_status_;

  if (laser_array[index_].laser_temperature_ < 0 || laser_array[index_].imu_temperature_ < 0) {
    rec_error[index_] |= (1 << 6);
  }

  // LOG_I("module%d: mac:0x%x security_status_: 0x%x imu_temp: %d, laser_temp: %d\n", index_, laser_array[index_].mac(),laser_array[index_].security_status_, laser_array[index_].imu_temperature_, laser_array[index_].laser_temperature_);

  // if (laser_array[index_].security_status_ != 0) {
  //   laser->need_to_turnoff_laser_ = true;

  //   if (systemservice.GetCurrentStage() == SYSTAGE_WORK || systemservice.GetCurrentStage() == SYSTAGE_PAUSE) {
  //     quickstop.Trigger(QS_SOURCE_SECURITY, true);
  //   } else if (laser->laser_status_ == LASER_ENABLE) {
  //     laser->TurnOff();
  //   }
  // }
}

static void CallbackAckReportSecurityIndex2(CanStdDataFrame_t &cmd) {
  uint8_t index_ = 1;
  laser_array[index_].security_status_ = cmd.data[0];
  laser_array[index_].pitch_ = (cmd.data[1] << 8) | cmd.data[2];
  laser_array[index_].roll_ = (cmd.data[3] << 8) | cmd.data[4];
  laser_array[index_].laser_temperature_ = cmd.data[5];
  laser_array[index_].imu_temperature_ = (int8_t)cmd.data[6];
  laser_array[index_].fire_sensor_trigger_ = cmd.data[7];

  if (laser_init_total == index_ + 1)
    laser->need_to_tell_hmi_ = true;

  rec_error[index_] |= laser_array[index_].security_status_;

  if (laser_array[index_].laser_temperature_ < 0 || laser_array[index_].imu_temperature_ < 0) {
    rec_error[index_] |= (1 << 6);
  }

  // LOG_I("module%d: mac:0x%x security_status_: 0x%x imu_temp: %d, laser_temp: %d\n", index_, laser_array[index_].mac(),laser_array[index_].security_status_, laser_array[index_].imu_temperature_, laser_array[index_].laser_temperature_);
}

static void CallbackAckReportSecurityIndex3(CanStdDataFrame_t &cmd) {
  uint8_t index_ = 2;
  laser_array[index_].security_status_ = cmd.data[0];
  laser_array[index_].pitch_ = (cmd.data[1] << 8) | cmd.data[2];
  laser_array[index_].roll_ = (cmd.data[3] << 8) | cmd.data[4];
  laser_array[index_].laser_temperature_ = cmd.data[5];
  laser_array[index_].imu_temperature_ = (int8_t)cmd.data[6];
  laser_array[index_].fire_sensor_trigger_ = cmd.data[7];

  if (laser_init_total == index_ + 1)
    laser->need_to_tell_hmi_ = true;

  rec_error[index_] |= laser_array[index_].security_status_;

  if (laser_array[index_].laser_temperature_ < 0 || laser_array[index_].imu_temperature_ < 0) {
    rec_error[index_] |= (1 << 6);
  }

  // LOG_I("module%d: mac:0x%x security_status_: 0x%x imu_temp: %d, laser_temp: %d\n", index_, laser_array[index_].mac(),laser_array[index_].security_status_, laser_array[index_].imu_temperature_, laser_array[index_].laser_temperature_);
}

static void CallbackAckReportSecurityIndex4(CanStdDataFrame_t &cmd) {
  uint8_t index_ = 3;
  laser_array[index_].security_status_ = cmd.data[0];
  laser_array[index_].pitch_ = (cmd.data[1] << 8) | cmd.data[2];
  laser_array[index_].roll_ = (cmd.data[3] << 8) | cmd.data[4];
  laser_array[index_].laser_temperature_ = cmd.data[5];
  laser_array[index_].imu_temperature_ = (int8_t)cmd.data[6];
  laser_array[index_].fire_sensor_trigger_ = cmd.data[7];

  if (laser_init_total == index_ + 1)
    laser->need_to_tell_hmi_ = true;

  rec_error[index_] |= laser_array[index_].security_status_;

  if (laser_array[index_].laser_temperature_ < 0 || laser_array[index_].imu_temperature_ < 0) {
    rec_error[index_] |= (1 << 6);
  }

  // LOG_I("module%d: mac:0x%x security_status_: 0x%x imu_temp: %d, laser_temp: %d\n", index_, laser_array[index_].mac(),laser_array[index_].security_status_, laser_array[index_].imu_temperature_, laser_array[index_].laser_temperature_);
}

static void CallbackAckReportSecurityIndex5(CanStdDataFrame_t &cmd) {
  uint8_t index_ = 4;
  laser_array[index_].security_status_ = cmd.data[0];
  laser_array[index_].pitch_ = (cmd.data[1] << 8) | cmd.data[2];
  laser_array[index_].roll_ = (cmd.data[3] << 8) | cmd.data[4];
  laser_array[index_].laser_temperature_ = cmd.data[5];
  laser_array[index_].imu_temperature_ = (int8_t)cmd.data[6];
  laser_array[index_].fire_sensor_trigger_ = cmd.data[7];

  if (laser_init_total == index_ + 1)
    laser->need_to_tell_hmi_ = true;

  rec_error[index_] |= laser_array[index_].security_status_;

  if (laser_array[index_].laser_temperature_ < 0 || laser_array[index_].imu_temperature_ < 0) {
    rec_error[index_] |= (1 << 6);
  }

  // LOG_I("module%d: mac:0x%x security_status_: 0x%x imu_temp: %d, laser_temp: %d\n", index_, laser_array[index_].mac(),laser_array[index_].security_status_, laser_array[index_].imu_temperature_, laser_array[index_].laser_temperature_);
}

static void CallbackAckReportSecurityIndex6(CanStdDataFrame_t &cmd) {
  uint8_t index_ = 5;
  laser_array[index_].security_status_ = cmd.data[0];
  laser_array[index_].pitch_ = (cmd.data[1] << 8) | cmd.data[2];
  laser_array[index_].roll_ = (cmd.data[3] << 8) | cmd.data[4];
  laser_array[index_].laser_temperature_ = cmd.data[5];
  laser_array[index_].imu_temperature_ = (int8_t)cmd.data[6];
  laser_array[index_].fire_sensor_trigger_ = cmd.data[7];

  if (laser_init_total == index_ + 1)
    laser->need_to_tell_hmi_ = true;

  rec_error[index_] |= laser_array[index_].security_status_;

  if (laser_array[index_].laser_temperature_ < 0 || laser_array[index_].imu_temperature_ < 0) {
    rec_error[index_] |= (1 << 6);
  }

  // LOG_I("module%d: mac:0x%x security_status_: 0x%x imu_temp: %d, laser_temp: %d\n", index_, laser_array[index_].mac(),laser_array[index_].security_status_, laser_array[index_].imu_temperature_, laser_array[index_].laser_temperature_);
}

// static void CallbackAckGetCrossLight(CanStdDataFrame_t &cmd) {
//   laser->cross_light_state_ = cmd.data[0];
//   laser->cross_light_state_update_ = true;
// }

// static void CallbackAckGetFireSensorSensitivity(CanStdDataFrame_t &cmd) {
//   laser->fire_sensor_trigger_value_ = cmd.data[0] | (cmd.data[1] << 8);
//   laser->fire_sensor_sensitivity_update_ = true;
// }

// static void CallbackAckGetCrossLightOffset(CanStdDataFrame_t &cmd) {
//   laser->crosslight_offset_x = *(float *)(&cmd.data[0]);
//   laser->crosslight_offset_y = *(float *)(&cmd.data[4]);
//   laser->crosslight_offset_update_ = true;
// }

// static void CallbackAckReportFireSensorRawData(CanStdDataFrame_t &cmd) {
//   laser->fire_sensor_rawdata_ = cmd.data[0] | (cmd.data[1]<<8);
//   LOG_I("frd: %d\n", laser->fire_sensor_rawdata_);
// }


static void CallbackAckReportFireSensorRawDataIndex1(CanStdDataFrame_t &cmd) {
  laser_array[0].fire_sensor_rawdata_ = cmd.data[0] | (cmd.data[1]<<8);
  LOG_I("frd0: %d\n", laser_array[0].fire_sensor_rawdata_);
}

static void CallbackAckReportFireSensorRawDataIndex2(CanStdDataFrame_t &cmd) {
  laser_array[1].fire_sensor_rawdata_ = cmd.data[0] | (cmd.data[1]<<8);
  LOG_I("frd1: %d\n", laser_array[0].fire_sensor_rawdata_);
}

static void CallbackAckReportFireSensorRawDataIndex3(CanStdDataFrame_t &cmd) {
  laser_array[2].fire_sensor_rawdata_ = cmd.data[0] | (cmd.data[1]<<8);
  LOG_I("frd2: %d\n", laser_array[0].fire_sensor_rawdata_);
}

static void CallbackAckReportFireSensorRawDataIndex4(CanStdDataFrame_t &cmd) {
  laser_array[3].fire_sensor_rawdata_ = cmd.data[0] | (cmd.data[1]<<8);
  LOG_I("frd3: %d\n", laser_array[0].fire_sensor_rawdata_);
}

static void CallbackAckReportFireSensorRawDataIndex5(CanStdDataFrame_t &cmd) {
  laser_array[4].fire_sensor_rawdata_ = cmd.data[0] | (cmd.data[1]<<8);
  LOG_I("frd4: %d\n", laser_array[0].fire_sensor_rawdata_);
}

static void CallbackAckReportFireSensorRawDataIndex6(CanStdDataFrame_t &cmd) {
  laser_array[5].fire_sensor_rawdata_ = cmd.data[0] | (cmd.data[1]<<8);
  LOG_I("frd5: %d\n", laser_array[0].fire_sensor_rawdata_);
}


void ToolHeadLaser::SetLaserOutputPinSta(bool sta, uint8_t index) {
  int8_t laser_power_pin[LASER_AGING_INDEX_MAX] = {E0_STEP_PIN, E1_STEP_PIN, X_STEP_PIN, Y_STEP_PIN, Z_STEP_PIN, B_STEP_PIN};
  if (index < LASER_AGING_INDEX_MAX) {
    OUT_WRITE(laser_power_pin[index], sta);
  }
  else {
    for (uint8_t i = 0; i < LASER_AGING_INDEX_MAX; i++)
      OUT_WRITE(laser_power_pin[i], sta);
  }
}

void ToolHeadLaser::SetLaserDirCheckPinSta(bool sta, uint8_t index) {
  int8_t laser_dir_pin[LASER_AGING_INDEX_MAX] = {E0_DIR_PIN, E1_DIR_PIN, X_DIR_PIN, Y_DIR_PIN, Z_DIR_PIN, B_DIR_PIN};
  if (index < LASER_AGING_INDEX_MAX) {
    OUT_WRITE(laser_dir_pin[index], sta);
  }
  else {
    for (uint8_t i = 0; i < LASER_AGING_INDEX_MAX; i++)
      OUT_WRITE(laser_dir_pin[i], sta);
  }
}


ErrCode ToolHeadLaser::Init(MAC_t &mac, uint8_t mac_index) {
  ErrCode ret;
  CanExtCmd_t cmd;
  uint8_t     func_buffer[2*32+2];

  Function_t    function;
  message_id_t  message_id[32];
  uint16_t get_dev_id = MODULE_GET_DEVICE_ID(mac.val);
  // uint8_t port_index = 0;

  LOG_I("\tGot toolhead Laser!!!  dev_id: %d, mac_index: %d\n", get_dev_id, mac_index);

  // if (axis_to_port[E_AXIS] != PORT_8PIN_1) {
  //   LOG_E("toolhead Laser failed: Please use the <M1029 E1> set E port\n");
  //   return E_HARDWARE;
  // }

  // ret = ModuleBase::InitModule8p(mac, E0_DIR_PIN, 0);
  // if (ret != E_SUCCESS)
  //   return ret;

  // we have configured Laser in same port
  // if (mac_index_ != MODULE_MAC_INDEX_INVALID)
  //   return E_SAME_STATE;

  if (laser_init_total >= LASER_AGING_INDEX_MAX) {
    LOG_E("laser_init_total: %d, init failed\n", laser_init_total);
    return E_SAME_STATE;
  }

  if (!laser_init_total) {
    SetLaserOutputPinSta(true);
  }

  vTaskDelay(10);

  int8_t laser_dir_pin[LASER_AGING_INDEX_MAX] = {E0_DIR_PIN, E1_DIR_PIN, X_DIR_PIN, Y_DIR_PIN, Z_DIR_PIN, B_DIR_PIN};
  uint8_t tmp_port_index = 0xFF;
  for (uint8_t i = 0; i < LASER_AGING_INDEX_MAX; i++) {
    SetLaserDirCheckPinSta(LOW);
    if (mc_port[i] == 0) {
      ret = ModuleBase::InitModule8p(mac, laser_dir_pin[i], 0);
      if (ret == E_SUCCESS) {
        mc_port[i] = 1;
        tmp_port_index = i;
        LOG_I("!!!!!!!!find port_index: %d!!!!!!!!!\n", tmp_port_index);
        break;
      }
    }
  }

  if (tmp_port_index == 0xFF) {
    LOG_E("laser init: dir test fail!!!\n");
    // return E_HARDWARE;
  }

  laser_array[laser_init_total].device_id(get_dev_id);
  laser_array[laser_init_total].port_index_ = tmp_port_index;
  LOG_I("laser mode%d init, mac: 0x%x, dev_id: %d\n", laser_init_total, mac.val, get_dev_id);

  cmd.mac    = mac;
  cmd.data   = func_buffer;
  cmd.length = 1;

  cmd.data[MODULE_EXT_CMD_INDEX_ID] = MODULE_EXT_CMD_GET_FUNCID_REQ;

  // try to get function ids from module
  if (canhost.SendExtCmdSync(cmd, 500, 2) != E_SUCCESS)
    return E_FAILURE;

  function.channel   = mac.bits.channel;
  function.mac_index = mac_index;
  function.sub_index = laser_init_total;
  function.priority  = MODULE_FUNC_PRIORITY_DEFAULT;

  for (int i = 0; i < cmd.data[MODULE_EXT_CMD_INDEX_DATA]; i++) {
    function.id = (cmd.data[i*2 + 2]<<8 | cmd.data[i*2 + 3]);
    /*if (function.id == MODULE_FUNC_GET_LASER_FOCUS) {
      message_id[i]     = canhost.RegisterFunction(function, CallbackAckLaserFocus);
      msg_id_get_focus_ = message_id[i];
    }
    else */if (function.id == MODULE_FUNC_REPORT_SECURITY_STATUS) {
      message_id[i] = canhost.RegisterFunction(function, call_back_report_security_array[laser_init_total]);
    }
    // else if (function.id == MODULE_FUNC_GET_CROSSLIGHT_STATE) {
    //   message_id[i] = canhost.RegisterFunction(function, CallbackAckGetCrossLight);
    // }
    // else if (function.id == MODULE_FUNC_GET_FIRE_SENSOR_SENSITIVITY) {
    //   message_id[i] = canhost.RegisterFunction(function, CallbackAckGetFireSensorSensitivity);
    // }
    // else if (function.id == MODULE_FUNC_GET_CROSSLIGHT_OFFSET) {
    //   message_id[i] = canhost.RegisterFunction(function, CallbackAckGetCrossLightOffset);
    // }
    else if (function.id == MODULE_FUNC_REPORT_FIRE_SENSOR_RAWDATA) {
      message_id[i] = canhost.RegisterFunction(function, call_back_report_fire_sensor_rawdata_array[laser_init_total]);
    }
    else {
      message_id[i] = canhost.RegisterFunction(function, NULL);
    }
  }

  // register function ids to can host, it will assign message id
  // for (int i = 0; i < cmd.data[MODULE_EXT_CMD_INDEX_DATA]; i++) {
  //   function.id = (cmd.data[i*2 + 2]<<8 | cmd.data[i*2 + 3]);
  //   if (function.id == MODULE_FUNC_GET_LASER_FOCUS) {
  //     message_id[i]     = canhost.RegisterFunction(function, CallbackAckLaserFocus);
  //     msg_id_get_focus_ = message_id[i];
  //   }
  //   else if (function.id == MODULE_FUNC_REPORT_SECURITY_STATUS) {
  //     message_id[i] = canhost.RegisterFunction(function, CallbackAckReportSecurity);
  //   }
  //   else if (function.id == MODULE_FUNC_GET_CROSSLIGHT_STATE) {
  //     message_id[i] = canhost.RegisterFunction(function, CallbackAckGetCrossLight);
  //   }
  //   else if (function.id == MODULE_FUNC_GET_FIRE_SENSOR_SENSITIVITY) {
  //     message_id[i] = canhost.RegisterFunction(function, CallbackAckGetFireSensorSensitivity);
  //   }
  //   else if (function.id == MODULE_FUNC_GET_CROSSLIGHT_OFFSET) {
  //     message_id[i] = canhost.RegisterFunction(function, CallbackAckGetCrossLightOffset);
  //   }
  //   else if (function.id == MODULE_FUNC_REPORT_FIRE_SENSOR_RAWDATA) {
  //     message_id[i] = canhost.RegisterFunction(function, CallbackAckReportFireSensorRawData);
  //   }
  //   else {
  //     message_id[i] = canhost.RegisterFunction(function, NULL);
  //   }

  //   if (function.id == MODULE_FUNC_SET_FAN1)
  //     msg_id_set_fan_ = message_id[i];
  // }

  // register function ids to can host, it will assign message id
  // for (int i = 0; i < cmd.data[MODULE_EXT_CMD_INDEX_DATA]; i++) {
  //   function.id = (cmd.data[i*2 + 2]<<8 | cmd.data[i*2 + 3]);
  //   if (function.id == MODULE_FUNC_GET_LASER_FOCUS) {
  //     message_id[i]     = canhost.RegisterFunction(function, CallbackAckLaserFocus);
  //     msg_id_get_focus_ = message_id[i];
  //   }
  //   else if (function.id == MODULE_FUNC_REPORT_SECURITY_STATUS) {
  //     message_id[i] = canhost.RegisterFunction(function, CallbackAckReportSecurity);
  //   }
  //   else if (function.id == MODULE_FUNC_GET_CROSSLIGHT_STATE) {
  //     message_id[i] = canhost.RegisterFunction(function, CallbackAckGetCrossLight);
  //   }
  //   else if (function.id == MODULE_FUNC_GET_FIRE_SENSOR_SENSITIVITY) {
  //     message_id[i] = canhost.RegisterFunction(function, CallbackAckGetFireSensorSensitivity);
  //   }
  //   else if (function.id == MODULE_FUNC_GET_CROSSLIGHT_OFFSET) {
  //     message_id[i] = canhost.RegisterFunction(function, CallbackAckGetCrossLightOffset);
  //   }
  //   else if (function.id == MODULE_FUNC_REPORT_FIRE_SENSOR_RAWDATA) {
  //     message_id[i] = canhost.RegisterFunction(function, CallbackAckReportFireSensorRawData);
  //   }
  //   else {
  //     message_id[i] = canhost.RegisterFunction(function, NULL);
  //   }

  //   if (function.id == MODULE_FUNC_SET_FAN1)
  //     msg_id_set_fan_ = message_id[i];
  // }

  ret = canhost.BindMessageID(cmd, message_id);

  if (ret != E_SUCCESS) {
    LOG_E("laser init BindMessageID fail\n");
    return ret;
  }

  // if (MODULE_DEVICE_ID_10W_LASER == device_id() || MODULE_DEVICE_ID_1_6_W_LASER == device_id()) {
  //   Tim1PwmInit(1862, 255);     // 1.6w & 10w, no modification of the old laser control frequency for the time being  // 250Hz
  //   esp32_.Init(&MSerial3, EXECUTOR_SERIAL_IRQ_PRIORITY);
  // }
  // else {
  //   Tim1PwmInit(93, 255);       // 5kHz
  // }

  // mac_index_ = mac_index;
  // state_     = TOOLHEAD_LASER_STATE_OFF;

  laser_array[laser_init_total].mac_index_ = mac_index;
  laser_array[laser_init_total].state_ = state_;

  // laser = this;
  // set toolhead
  // power_table_ = power_table_1_6W;
  // if (laser->device_id_ == MODULE_DEVICE_ID_1_6_W_LASER) {
  //   power_table_ = power_table_1_6W;
  //   SetToolhead(MODULE_TOOLHEAD_LASER);
  // } else if (laser->device_id_ == MODULE_DEVICE_ID_10W_LASER) {
  //   power_table_ = power_table_10W;
  //   SetToolhead(MODULE_TOOLHEAD_LASER_10W);
  // }
  // else if (laser->device_id_ == MODULE_DEVICE_ID_20W_LASER) {
  //   power_table_ = power_table_20W;
  //   SetToolhead(MODULE_TOOLHEAD_LASER_20W);
  // }
  // else if (laser->device_id_ == MODULE_DEVICE_ID_40W_LASER) {
  //   power_table_ = power_table_40W;
  //   SetToolhead(MODULE_TOOLHEAD_LASER_40W);
  // }

  laser_array[laser_init_total].power_table_ = power_table_1_6W;
  if (laser->device_id_ == MODULE_DEVICE_ID_1_6_W_LASER) {
    laser_array[laser_init_total].power_table_ = power_table_1_6W;
    SetToolhead(MODULE_TOOLHEAD_LASER);
  } else if (laser->device_id_ == MODULE_DEVICE_ID_10W_LASER) {
    laser_array[laser_init_total].power_table_ = power_table_10W;
    SetToolhead(MODULE_TOOLHEAD_LASER_10W);
  }
  else if (laser->device_id_ == MODULE_DEVICE_ID_20W_LASER) {
    laser_array[laser_init_total].power_table_ = power_table_20W;
    SetToolhead(MODULE_TOOLHEAD_LASER_20W);
  }
  else if (laser->device_id_ == MODULE_DEVICE_ID_40W_LASER) {
    laser_array[laser_init_total].power_table_ = power_table_40W;
    SetToolhead(MODULE_TOOLHEAD_LASER_40W);
  }

  laser = &laser_array[laser_init_total];
  laser_array[laser_init_total].module_index_ = laser_init_total;
  laser_init_total++;

  return E_SUCCESS;
}

void ToolHeadLaser::TurnoffLaserIfNeeded() {
  bool is_disable_laser = (laser_status_ == LASER_WAIT_DISABLE) && (++laser_tick_ > LASER_DISABLE_DELAY);
  if (laser->need_to_turnoff_laser_) {
    // The module is exception and needs to be turn off
    laser->need_to_turnoff_laser_ = false;
    TurnOff();
    is_disable_laser = true;
  }
  if (is_disable_laser) {
    LaserControl(0);
  }
}

void ToolHeadLaser::PrintInfo(void) {
  if (laser->device_id_ == MODULE_DEVICE_ID_20W_LASER || laser->device_id_ == MODULE_DEVICE_ID_40W_LASER) {
    float x_offset, y_offset;
    GetCrossLightOffsetCAN(x_offset, y_offset);
  }

  LOG_I("power limit pwm %d\n", power_limit_pwm_);
  LOG_I("power_pwm_ %d\n", power_pwm_);
  LOG_I("power_val_ %f\n", power_val_);
  LOG_I("security_status_ %d\n", security_status_);
  LOG_I("laser_temperature_ %d\n", laser_temperature_);
  LOG_I("imu_temperature_ %d\n", imu_temperature_);
  LOG_I("fire_sensor_trigger_: %s\n", fire_sensor_trigger_ ? "TRIGGER" : "NORMAL");
  LOG_I("fire_sensor_trigger_adc_value_ %d\n", fire_sensor_trigger_value_);
  LOG_I("crosslight_offset_x %f, crosslight_offset_y %f\n", crosslight_offset_x, crosslight_offset_y);
  LOG_I("half_power_mode_: %s\n", half_power_mode_ ? "OPEN" : "CLOSE");
}

uint16_t ToolHeadLaser::tim_pwm() {
  return TimGetPwm();
}
void ToolHeadLaser::tim_pwm(uint16_t pwm) {
  TimSetPwm(pwm);
}

void ToolHeadLaser::TurnOn() {
  if (state_ == TOOLHEAD_LASER_STATE_OFFLINE)
    return;
  return;
  if ((laser->device_id_ == MODULE_DEVICE_ID_10W_LASER || laser->device_id_ == MODULE_DEVICE_ID_20W_LASER || \
      laser->device_id_ == MODULE_DEVICE_ID_40W_LASER) && laser->security_status_ != 0) {
    return;
  }

  if (laser->device_id_ == MODULE_DEVICE_ID_10W_LASER || laser->device_id_ == MODULE_DEVICE_ID_20W_LASER || \
      laser->device_id_ == MODULE_DEVICE_ID_40W_LASER) {
    if (laser_status_ == LASER_DISABLE) {
      // Send the enable command only when the power is disabled
      LaserControl(1);
    }
    // Stop waiting disable
    laser_status_ = LASER_ENABLE;
  }
  state_ = TOOLHEAD_LASER_STATE_ON;
  CheckFan(power_pwm_);
  tim_pwm(power_pwm_);
}

void ToolHeadLaser::PwmCtrlDirectly(uint8_t duty) {
  tim_pwm(duty);
}

void ToolHeadLaser::TurnOff() {
  CloseAllLaser();
  return;
  if (state_ == TOOLHEAD_LASER_STATE_OFFLINE)
    return;

  if (laser->device_id_ == MODULE_DEVICE_ID_10W_LASER || laser->device_id_ == MODULE_DEVICE_ID_20W_LASER || \
      laser->device_id_ == MODULE_DEVICE_ID_40W_LASER) {
    if (laser_status_ != LASER_DISABLE) {
      laser_status_ = LASER_WAIT_DISABLE;
      laser_tick_ = 0;
    }
  }
  laser->InlineDisable();
  state_ = TOOLHEAD_LASER_STATE_OFF;
  CheckFan(0);
  tim_pwm(0);
}

void ToolHeadLaser::SetOutput(float power) {
  return;
  if ((laser->device_id_ == MODULE_DEVICE_ID_10W_LASER || laser->device_id_ == MODULE_DEVICE_ID_20W_LASER || \
      laser->device_id_ == MODULE_DEVICE_ID_40W_LASER) && laser->security_status_ != 0) {
    return;
  }

  SetPower(power);
  TurnOn();
}


void ToolHeadLaser::SetPower(float power) {
  int   integer;
  float decimal;

  if (state_ == TOOLHEAD_LASER_STATE_OFFLINE)
    return;

  // limit to valid range of the table
  LIMIT(power, 0.0f, 100.0f);
  power_val_ = power;

  integer = (int)power;
  decimal = power - integer;

  // even if random data is read by exceeding the table data, it will be discarded when multiplying by 0
  power_pwm_ = (uint16_t)(power_table_[integer] + (power_table_[integer + 1] - power_table_[integer]) * decimal);

  if (power_pwm_ > power_limit_pwm_)
    power_pwm_ = power_limit_pwm_;
}


void ToolHeadLaser::SetPowerLimit(float limit) {
  float cur_power = power_val_;

  if (limit > TOOLHEAD_LASER_POWER_NORMAL_LIMIT)
    limit = TOOLHEAD_LASER_POWER_NORMAL_LIMIT;

  // Compute limit PWM value
  power_limit_pwm_ = 255;
  SetPower(limit);
  power_limit_pwm_ = power_pwm_;

  LOG_I("laser limit: &u\n", power_limit_pwm_);

  SetPower(cur_power);

  // check if we need to change current output
  if (state_ == TOOLHEAD_LASER_STATE_ON)
    TurnOn();
}

void ToolHeadLaser::SetFanPower(uint8_t power) {
  // CanStdMesgCmd_t cmd;
  // uint8_t         buffer[2];
  // buffer[0]  = 0;
  // buffer[1]  = power;
  // cmd.id     = msg_id_set_fan_;
  // cmd.data   = buffer;
  // cmd.length = 2;

  CanStdFuncCmd_t cmd;
  uint8_t msg_len = 0;
  uint8_t can_buffer[2] = {0};

  can_buffer[msg_len++] = 0;
  can_buffer[msg_len++] = power;
  cmd.id        = MODULE_FUNC_SET_FAN1;
  cmd.data      = can_buffer;
  cmd.length    = msg_len;

  for (uint8_t i = 0; i < laser_init_total; i++) {
    canhost.SendStdCmd(cmd, i);
  }
}

void ToolHeadLaser::CheckFan(uint16_t pwm) {
  switch (fan_state_) {
  case TOOLHEAD_LASER_FAN_STATE_OPEN:
    if (pwm == 0) {
      fan_state_ = TOOLHEAD_LASER_FAN_STATE_TO_BE_CLOSED;
      fan_tick_  = 0;
    }
    break;

  case TOOLHEAD_LASER_FAN_STATE_TO_BE_CLOSED:
    if (pwm > 0) {
      fan_state_ = TOOLHEAD_LASER_FAN_STATE_OPEN;
      fan_tick_  = 0;
    }
    break;

  case TOOLHEAD_LASER_FAN_STATE_CLOSED:
    if (pwm > 0) {
      fan_state_ = TOOLHEAD_LASER_FAN_STATE_OPEN;
      SetFanPower(255);
    }
    break;
  }
}

void ToolHeadLaser::TryCloseFan() {
  if (state_ == TOOLHEAD_LASER_STATE_OFFLINE)
    return;

  if (fan_state_ == TOOLHEAD_LASER_FAN_STATE_TO_BE_CLOSED) {
    if (++fan_tick_ > LASER_CLOSE_FAN_DELAY) {
      fan_tick_  = 0;
      fan_state_ = TOOLHEAD_LASER_FAN_STATE_CLOSED;
      SetFanPower(0);
    }
  }
}

ErrCode ToolHeadLaser::SetCrossLightCAN(bool sw) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[1];

  buffer[0]     = !!sw;
  cmd.id        = MODULE_FUNC_SET_CROSSLIGHT;
  cmd.data      = buffer;
  cmd.length    = 1;

  LOG_I("Set Cross Light: %s\n", sw ? "ON" : "OFF");

  return canhost.SendStdCmd(cmd);
}

ErrCode ToolHeadLaser::GetCrossLightCAN(bool &sw) {
  CanStdFuncCmd_t cmd;

  cmd.id        = MODULE_FUNC_GET_CROSSLIGHT_STATE;
  cmd.length    = 0;
  cross_light_state_update_ = false;
  ErrCode ret = canhost.SendStdCmd(cmd);
  if (E_SUCCESS != ret) {
    LOG_E("Get Cross Light send msg failed, ret: %d\n", ret);
    return ret;
  }

  int32_t max_try = 500;
  while(!cross_light_state_update_ && max_try--) vTaskDelay(pdMS_TO_TICKS(1));
  if (max_try >= 0) {
    sw = cross_light_state_;
    LOG_I("Get Cross Light: %s\n", sw ? "ON" : "OFF");
    return E_SUCCESS;
  }
  else {
    LOG_E("Get Cross Light time out\n");
    return E_TIMEOUT;
  }
}

ErrCode ToolHeadLaser::SetFireSensorSensitivityCAN(uint16 sen, bool is_save) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[8];

  LOG_I("Set FireSensorSensitivity adc value: %d\n", sen);

  if (sen > FIRE_DETECT_TRIGGER_LIMIT_ADC_VALUE)
    sen = FIRE_DETECT_TRIGGER_DISABLE_ADC_VALUE;

  buffer[0]     = sen & 0xFF;
  buffer[1]     = (sen >> 8) & 0xFF;
  buffer[2]     = is_save;
  cmd.id        = MODULE_FUNC_SET_FIRE_SENSOR_SENSITIVITY;
  cmd.data      = buffer;
  cmd.length    = 3;

  return canhost.SendStdCmd(cmd);
}
ErrCode ToolHeadLaser::GetFireSensorSensitivityCAN(uint16 &sen) {
  CanStdFuncCmd_t cmd;

  cmd.id        = MODULE_FUNC_GET_FIRE_SENSOR_SENSITIVITY;
  cmd.length    = 0;
  fire_sensor_sensitivity_update_ = false;
  ErrCode ret = canhost.SendStdCmd(cmd);
  if (E_SUCCESS != ret) {
    LOG_E("Get FireSensorSensitivity send msg failed, ret: %d\n", ret);
    return ret;
  }

  int32_t max_try = 500;
  while(!fire_sensor_sensitivity_update_ && max_try--) vTaskDelay(pdMS_TO_TICKS(1));
  if (max_try >= 0) {
    sen = fire_sensor_trigger_value_;
    LOG_I("Get FireSensorSensitivity adc value: %d\n", sen);
    return E_SUCCESS;
  }
  else {
    LOG_E("Get FireSensorSensitivity adc value time out\n");
    return E_TIMEOUT;
  }
}

ErrCode ToolHeadLaser::SetFireSensorReportTime(uint16 itv) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[2];

  LOG_I("Set FireSensorReportTime %dms\n", itv);

  buffer[0]     = itv & 0xFF;
  buffer[1]     = (itv>>8) & 0xFF;
  cmd.id        = MODULE_FUNC_SET_FIRE_SENSOR_REPORT_TIME;
  cmd.data      = buffer;
  cmd.length    = 2;

  return canhost.SendStdCmd(cmd);
}

ErrCode ToolHeadLaser::SetCrossLightOffsetCAN(float x, float y) {
  CanStdFuncCmd_t cmd;
  uint8_t buffer[8];

  LOG_I("Set CrossLightOffset, X: %f, Y: %f\n", x, y);

  *(float *)(&buffer[0]) = x;
  *(float *)(&buffer[4]) = y;
  cmd.id        = MODULE_FUNC_SET_CROSSLIGHT_OFFSET;
  cmd.data      = buffer;
  cmd.length    = 8;

  return canhost.SendStdCmd(cmd);
}


ErrCode ToolHeadLaser::GetCrossLightOffsetCAN(float &x, float &y) {
  CanStdFuncCmd_t cmd;

  cmd.id        = MODULE_FUNC_GET_CROSSLIGHT_OFFSET;
  cmd.length    = 0;
  crosslight_offset_update_ = false;
  ErrCode ret = canhost.SendStdCmd(cmd);
  if (E_SUCCESS != ret) {
    LOG_E("Get CrossLightOffset send msg failed\n");
    return ret;
  }

  int32_t max_try = 500;
  while(!crosslight_offset_update_ && max_try--) vTaskDelay(pdMS_TO_TICKS(1));
  if (max_try >= 0) {
    x = crosslight_offset_x;
    y = crosslight_offset_y;
    LOG_I("Get CrossLightOffset X: %f, Y: %f\n", x, y);
  }
  else {
    LOG_E("Get CrossLightOffset time out\n");
    return E_TIMEOUT;
  }

  if (CheckCrossLightOffset(crosslight_offset_x, crosslight_offset_y))
    return E_FAILURE;
  else
    return E_SUCCESS;
}

ErrCode ToolHeadLaser::CheckCrossLightOffset(float x_offset, float y_offset) {
  // TODO: use of more rigorous validity checks
  if ((x_offset == INVALID_OFFSET || y_offset == INVALID_OFFSET))
    return E_FAILURE;
  return E_SUCCESS;
}


ErrCode ToolHeadLaser::LoadFocus() {
  CanStdMesgCmd_t cmd;
  uint8_t data[1];
  if (rotaryModule.status() != ROTATE_OFFLINE) {
    data[0] = 1;
  } else {
    data[0] = 0;
  }
  cmd.id     = msg_id_get_focus_;
  cmd.length = 1;
  cmd.data = data;
  return canhost.SendStdCmd(cmd);
}


ErrCode ToolHeadLaser::GetFocus(SSTP_Event_t &event) {
  uint8_t  buff[5];

  LoadFocus();
  vTaskDelay(pdMS_TO_TICKS(20));

  LOG_I("SC get Focus: %.2f mm\n", focus_);
  uint16_t focus = focus_ * 1000;

  buff[0] = 0;

  buff[1] = 0;
  buff[2] = 0;
  buff[3] = (uint8_t)(focus>>8);
  buff[4] = (uint8_t)focus;

  event.length = 5;
  event.data   = buff;

  return hmi.Send(event);
}


ErrCode ToolHeadLaser::SetFocus(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  CanStdFuncCmd_t cmd;

  uint8_t  buff[3];
  uint32_t focus;

  if (event.length < 4) {
    LOG_E("Must specify Focus!\n");
    event.length = 1;
    event.data = &err;
    return hmi.Send(event);
  }

  PDU_TO_LOCAL_WORD(focus, event.data);

  LOG_I("SC set Focus: %.2f\n", focus / 1000.0f);

  // length and data is picked up, can be changed
  event.length = 1;
  event.data = &err;

  if (focus > TOOLHEAD_LASER_CAMERA_FOCUS_MAX) {
    LOG_E("new focus[%u] is out of range!\n", focus);
    return hmi.Send(event);
  }

  buff[0] = (uint8_t)(focus >> 8);
  buff[1] = (uint8_t)(focus);
  if (rotaryModule.status() != ROTATE_OFFLINE) {
    buff[2] = 1;
  } else {
    buff[2] = 0;
  }

  cmd.id     = MODULE_FUNC_SET_LASER_FOCUS;
  cmd.data   = buff;
  cmd.length = 3;

  err = canhost.SendStdCmd(cmd, 0);

  if (err == E_SUCCESS)
    LoadFocus();
  LOG_I("SC new focal length: %.3f\n", focus_);

  return hmi.Send(event);
}


ErrCode ToolHeadLaser::DoManualFocusing(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  float pos[XYZ];

  float max_z_speed;

  LOG_I("SC req manual focusing, limit: %d\n", power_limit_pwm_);

  if (!all_axes_homed()) {
    LOG_E("Machine is not be homed!\n");
    goto out;
  }

  if (!IsOnline()) {
    LOG_E("Laser is offline!\n");
    goto out;
  }

  if (event.length < 12) {
    LOG_E("need to specify position!\n");
    goto out;
  }

  planner.synchronize();

  max_z_speed = planner.settings.max_feedrate_mm_s[Z_AXIS];

  PDU_TO_LOCAL_WORD(pos[X_AXIS], event.data);
  PDU_TO_LOCAL_WORD(pos[Y_AXIS], event.data+4);
  PDU_TO_LOCAL_WORD(pos[Z_AXIS], event.data+8);
  LOG_I("Laser will move to (%.2f, %.2f, %.2f)\n", pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS]);

  planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

  // Move to the Certain point
  do_blocking_move_to_logical_xy(pos[X_AXIS], pos[Y_AXIS], speed_in_calibration[X_AXIS]);

  // Move to the Z
  do_blocking_move_to_logical_z( pos[Z_AXIS], speed_in_calibration[Z_AXIS]);

  planner.synchronize();

  planner.settings.max_feedrate_mm_s[Z_AXIS] = max_z_speed;

out:
  event.length = 1;
  event.data   = &err;
  return hmi.Send(event);
}


ErrCode ToolHeadLaser::DoAutoFocusing(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  uint8_t Count = 21;
  float z_interval = 0.5;

  float start_pos[XYZ];

  int   i = 0;
  float next_x, next_y, next_z;
  float line_space = 2;
  float line_len_short = 5;
  float line_len_long = 10;

  if (MODULE_DEVICE_ID_20W_LASER == device_id() || MODULE_DEVICE_ID_40W_LASER == device_id()) {
    LOG_E("Laser 20W or 40W do not support auto focusing\n");
    goto out;
  }

  LOG_I("SC req auto focusing, limit: %u\n", power_limit_pwm_);

  if (!all_axes_homed()) {
    LOG_E("Machine is not be homed!\n");
    goto out;
  }

  if (!IsOnline()) {
    LOG_E("Laser is offline!\n");
    goto out;
  }

  if (event.length == 4) {
    PDU_TO_LOCAL_WORD(z_interval, event.data);
    z_interval /= 1000;
    LOG_I("SC specify Z interval: %.2f\n", z_interval);
  }

  planner.synchronize();

  LOOP_XYZ(i) {
    start_pos[i] = current_position[i];
  }

  next_x = start_pos[X_AXIS] - (int)(Count / 2) * 2;
  next_y = start_pos[Y_AXIS];
  next_z = start_pos[Z_AXIS] - ((float)(Count - 1) / 2.0 * z_interval);

  // too low
  if(next_z <= 10) {
    LOG_E("start Z height is too low: %.2f\n", next_z);
    goto out;
  }

  // Move to next Z
  move_to_limited_z(next_z, 20.0f);

  // Draw 10 Line
  do {
    // Move to the start point
    move_to_limited_xy(next_x, next_y, speed_in_calibration[X_AXIS]);
    planner.synchronize();

    // Laser on
    SetOutput(laser_pwr_in_cali);

    // Draw Line
    if((i % 5) == 0)
      move_to_limited_xy(next_x, next_y + line_len_long, speed_in_draw_ruler);
    else
      move_to_limited_xy(next_x, next_y + line_len_short, speed_in_draw_ruler);

    planner.synchronize();

    // Laser off
    SetOutput(0);

    // Move up Z increase
    if(i != (Count - 1))
      move_to_limited_z(current_position[Z_AXIS] + z_interval, 20.0f);

    next_x = next_x + line_space;
    i++;
  } while(i < Count);

  planner.synchronize();

  // Move to beginning
  move_to_limited_z(start_pos[Z_AXIS], 20.0f);
  move_to_limited_xy(start_pos[X_AXIS], start_pos[Y_AXIS], 20.0f);
  planner.synchronize();

  err = E_SUCCESS;

out:
  event.data   = &err;
  event.length = 1;
  return hmi.Send(event);
}



/**
 * ReadBlueToothName:Read BT Name
 * para Name:The pointer to the Name buffer
 * return:0 for read success, 1 for unname, 2 for timeout
 */
ErrCode ToolHeadLaser::ReadBluetoothInfo(LaserCameraCommand cmd, uint8_t *out, uint16_t &length) {
  SSTP_Event_t  event = {cmd, 0, 0, NULL};

  ErrCode  ret = E_SUCCESS;

  if (MODULE_DEVICE_ID_20W_LASER == device_id() || MODULE_DEVICE_ID_40W_LASER == device_id()) {
    LOG_E("Read BT info failed, Laser 20W or 40W do not have any bluetooth\n");
    return E_FAILURE;
  }

  for (int i = 1; i < 4; i++) {
    esp32_.FlushInput();

    esp32_.Send(event);
    esp32_.FlushOutput();
    vTaskDelay(pdMS_TO_TICKS(200 * i));

    ret = esp32_.CheckoutCmd(out, length);
    if (ret != E_SUCCESS) {
      LOG_E("failed to read BT info - %u!\n", ret);
    }
    else {
      goto out;
    }
  }

  return ret;

out:
  if (length < 2) {
    LOG_E("failed to read BT info - %u!\n", 110);
    return E_NO_RESRC;
  }

  if (out[0] != (cmd + 1)) {
    LOG_E("failed to read BT info - %u!\n", 111);
    return E_INVALID_DATA;
  }

  if (out[1] != 0) {
    LOG_E("failed to read BT info - %u!\n", 112);
    return E_INVALID_DATA;
  }

  return E_SUCCESS;
}


/**
 * SetBluetoothName:Set BT name
 * para Name:The name of the BT
 * ret  None
 */
ErrCode ToolHeadLaser::SetBluetoothInfo(LaserCameraCommand cmd, uint8_t *info, uint16_t length) {
  SSTP_Event_t  event = {cmd, 0};

  uint8_t  buffer[72];
  ErrCode  ret;

  if (MODULE_DEVICE_ID_20W_LASER == device_id() || MODULE_DEVICE_ID_40W_LASER == device_id()) {
    LOG_E("Set BT info failed, Laser 20W or 40W do not have any blutooth\n");
    return E_INVALID_CMD;
  }

  event.length = length;
  event.data   = info;

  for (int i = 1; i < 4; i++) {
    esp32_.FlushInput();

    esp32_.Send(event);
    esp32_.FlushOutput();
    vTaskDelay(pdMS_TO_TICKS(200 * i));

    ret = esp32_.CheckoutCmd(buffer, length);
    if (ret != E_SUCCESS) {
      LOG_E("failed to set BT info - %u!\n", ret);
    }
    else {
      goto out;
    }
  }

  return ret;

out:
  if (buffer[0] != (cmd + 1)) {
    LOG_E("failed to read BT info - %u!\n", 111);
    return E_INVALID_CMD;
  }

  if (buffer[1] != 0) {
    LOG_E("failed to read BT info - %u!\n", 112);
    return E_INVALID_DATA;
  }

  return E_SUCCESS;
}


ErrCode ToolHeadLaser::SetCameraBtName(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  if (MODULE_DEVICE_ID_20W_LASER == device_id() || MODULE_DEVICE_ID_40W_LASER == device_id()) {
    LOG_E("Set BT Name failed, Laser 20W or 40W do not have any blutooth\n");
    err = E_INVALID_CMD;
    event.data   = &err;
    event.length = 1;
    return hmi.Send(event);
  }

  LOG_I("SC set BT Name: %s\n", event.data);

  err = SetBluetoothInfo(M_SET_BT_NAME, event.data, event.length);

  event.data   = &err;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode ToolHeadLaser::GetCameraBtName(SSTP_Event_t &event) {
  uint8_t buffer[40] = {0};
  ErrCode ret;

  if (MODULE_DEVICE_ID_20W_LASER == device_id() || MODULE_DEVICE_ID_40W_LASER == device_id()) {
    LOG_E("Req BT Name failed, Laser 20W or 40W do not have any blutooth\n");
    ErrCode err = E_INVALID_CMD;
    event.data   = &err;
    event.length = 1;
    return hmi.Send(event);
  }

  ret = ReadBluetoothInfo(M_REPORT_BT_NAME, buffer, event.length);

  LOG_I("SC req BT Name: %s\n", buffer + 2);


  if (ret != E_SUCCESS) {
    event.length = 1;
    event.data   = &ret;
  }
  else {
    /* when ReadBluetoothInfo() save content to buffer[],
     * buffer[0] is ACK command id, buffer[1] is ACK code from module,
     * buffer[2: end] is actual information we need.
     * Because we also need to return ACK to to HMI, so
     * event.data = buffer + 1; and length increase one
     */
    event.data   = buffer + 1;
    event.length -= 1;
  }

  return hmi.Send(event);
}


ErrCode ToolHeadLaser::GetCameraBtMAC(SSTP_Event_t &event) {
  uint8_t buffer[16] = {0};
  ErrCode ret;

  if (MODULE_DEVICE_ID_20W_LASER == device_id() || MODULE_DEVICE_ID_40W_LASER == device_id()) {
    LOG_E("Get BT MAC failed, Laser 20W or 40W do not have any blutooth\n");
    ErrCode err = E_INVALID_CMD;
    event.data   = &err;
    event.length = 1;
    return hmi.Send(event);
  }

  LOG_I("SC get BT MAC\n");

  ret = ReadBluetoothInfo(M_REPORT_BT_MAC, buffer, event.length);

  if (ret != E_SUCCESS) {
    event.data   = &ret;
    event.length = 1;
  }
  else {
    /* when ReadBluetoothInfo() save content to buffer[],
     * buffer[0] is ACK command id, buffer[1] is ACK code from module,
     * buffer[2: 2+length] is actual information we need.
     * Because we also need to return ACK to to HMI, so
     * event.data = buffer + 1; and length increase one
     */
    event.data   = buffer + 1;
    event.length -= 1;

    LOG_I("\t0x%08X, 0x%08X\n", *(uint32_t *)(buffer), *(uint32_t *)(buffer + 4));
  }

  return hmi.Send(event);
}

/**
 * ReadBlueToothName:Read BT versions
 * return:0 for read success, 1 for unname, 2 for timeout
 */
ErrCode ToolHeadLaser::ReadBluetoothVer() {
  uint8_t  buff[72];
  uint16_t size;
  ErrCode ret = E_SUCCESS;

  if (MODULE_DEVICE_ID_20W_LASER == device_id() || MODULE_DEVICE_ID_40W_LASER == device_id()) {
    LOG_E("Read BT version failed, Laser 20W or 40W do not have any blutooth\n");
    return E_FAILURE;
  }

  ret = ReadBluetoothInfo(M_REPORT_VERSIONS, buff, size);

  if (ret != E_SUCCESS) {
    LOG_E("failed to read BT version - %u!\n", ret);
    return 2;
  }

  LOG_I("BT version: %s\n", buff + 2);
  return 0;
}

/**
 * SetCameraLight:set camera light status
 * para state:1-open 0-close
 */
void ToolHeadLaser::SetCameraLight(uint8_t state) {
  SSTP_Event_t  event = {M_SET_CAMERA_LIGHT, 0, 0, NULL};
  uint8_t buff[1];

  if (MODULE_DEVICE_ID_20W_LASER == device_id() || MODULE_DEVICE_ID_40W_LASER == device_id()) {
    LOG_E("Set Laser Camera light failed, Laser 20W or 40W do not have blutooth\n");
    return;
  }

  buff[0] = !state;
  event.data = buff;
  event.length = 1;
  SetBluetoothInfo(M_SET_CAMERA_LIGHT, event.data, event.length);
  LOG_I("set Laser Camera light:%d!\n", state);
}

ErrCode ToolHeadLaser::SetAutoFocusLight(SSTP_Event_t &event) {
  CanStdFuncCmd_t cmd;
  uint8_t can_buffer[1];

  if (MODULE_DEVICE_ID_20W_LASER == device_id() || MODULE_DEVICE_ID_40W_LASER == device_id()) {
    LOG_E("Laser 20W or 40W do not have any focuslight\n");
    uint8_t err = E_INVALID_CMD;
    event.data   = &err;
    event.length = 1;
    return hmi.Send(event);
  }

  switch (event.data[0]) {
  case 0:
    can_buffer[0] = 0;
    break;

  case 1:
    can_buffer[0] = 1;
    break;

  default:
    return E_PARAM;
  }

  cmd.id        = MODULE_FUNC_SET_AUTOFOCUS_LIGHT;
  cmd.data      = can_buffer;
  cmd.length    = 1;


  uint8_t buff[1];

  if (canhost.SendStdCmdSync(cmd, 2000) == E_SUCCESS) {
    buff[0] = 0;
  } else {
    buff[0] = 1;
  }

  SSTP_Event_t event_hmi = {EID_SETTING_ACK, SETTINGS_OPC_SET_AUTOFOCUS_LIGHT};
  event_hmi.length = 1;
  event_hmi.data = buff;

  return hmi.Send(event_hmi);
}

ErrCode ToolHeadLaser::GetSecurityStatus(SSTP_Event_t &event) {
  CanStdFuncCmd_t cmd;

  cmd.id        = MODULE_FUNC_REPORT_SECURITY_STATUS;
  cmd.data      = NULL;
  cmd.length    = 0;
  return canhost.SendStdCmd(cmd);
}

ErrCode ToolHeadLaser::GetSecurityStatus(void) {
  CanStdFuncCmd_t cmd;

  cmd.id        = MODULE_FUNC_REPORT_SECURITY_STATUS;
  cmd.data      = NULL;
  cmd.length    = 0;

  for (uint8_t i = 0; i < laser_init_total; i++) {
    canhost.SendStdCmd(cmd, i);
  }
  return E_SUCCESS;
}

ErrCode ToolHeadLaser::SendSecurityStatus() {
  SSTP_Event_t event = {EID_SYS_CTRL_ACK, SYSCTL_OPC_SECURITY_STATUS};
  uint8_t buff[7];

  buff[0] = laser->security_status_;
  buff[1] = laser->laser_temperature_;
  buff[2] = laser->roll_ >> 8;
  buff[3] = laser->roll_ & 0xff;
  buff[4] = laser->pitch_ >> 8;
  buff[5] = laser->pitch_ & 0xff;
  buff[6] = laser->fire_sensor_trigger_;

  event.length = 7;
  event.data = buff;

  return hmi.Send(event);
}

ErrCode ToolHeadLaser::SendPauseStatus() {
  SSTP_Event_t event = {EID_SYS_CTRL_ACK, SYSCTL_OPC_PAUSE};

  event.length = 0;
  event.data = NULL;

  return hmi.Send(event);
}

ErrCode ToolHeadLaser::SetOnlineSyncId(SSTP_Event_t &event) {
  CanStdFuncCmd_t cmd;
  uint8_t can_buffer[5];

  can_buffer[0] = 1;
  can_buffer[1] = event.data[0];
  can_buffer[2] = event.data[1];
  can_buffer[3] = event.data[2];
  can_buffer[4] = event.data[3];
  cmd.id        = MODULE_FUNC_ONLINE_SYNC;
  cmd.data      = can_buffer;
  cmd.length    = 5;

  canhost.SendStdCmd(cmd);

  SSTP_Event_t event_hmi = {EID_SETTING_ACK, SETTINGS_OPC_SET_ONLINE_SYNC_ID};
  uint8_t buff[1] = {0};
  event_hmi.length = 1;
  event_hmi.data = buff;

  return hmi.Send(event_hmi);
}

ErrCode ToolHeadLaser::GetOnlineSyncId(SSTP_Event_t &event) {
  CanStdFuncCmd_t cmd;
  uint8_t can_buffer[1];

  can_buffer[0] = 0;
  cmd.id        = MODULE_FUNC_ONLINE_SYNC;
  cmd.data      = can_buffer;
  cmd.length    = 1;

  ErrCode ret;
  ret = canhost.SendStdCmdSync(cmd, 2000);
  if (ret != E_SUCCESS) {
    return ret;
  }

  LOG_I("get online sync id: 0x%x\n", (cmd.data[0] | cmd.data[1] << 8 | cmd.data[2] << 16 | cmd.data[3] << 24));

  SSTP_Event_t event_tmp = {EID_SETTING_ACK, SETTINGS_OPC_GET_ONLINE_SYNC_ID};
  uint8_t buff[4];

  buff[0] = cmd.data[0];
  buff[1] = cmd.data[1];
  buff[2] = cmd.data[2];
  buff[3] = cmd.data[3];

  event_tmp.length = 4;
  event_tmp.data = buff;

  hmi.Send(event_tmp);

  return E_SUCCESS;
}

ErrCode ToolHeadLaser::SetProtectTemp(SSTP_Event_t &event) {
  CanStdFuncCmd_t cmd;
  int8_t can_buffer[2];

  can_buffer[0] = event.data[0];
  can_buffer[1] = event.data[1];
  cmd.id        = MODULE_FUNC_SET_PROTECT_TEMP;
  cmd.data      = (uint8_t *)can_buffer;
  cmd.length    = 2;

  return canhost.SendStdCmd(cmd);
}

ErrCode ToolHeadLaser::LaserControl(uint8_t state) {
  // CanStdFuncCmd_t cmd;
  // uint8_t can_buffer[1];

  // can_buffer[0] = state;
  // cmd.id        = MODULE_FUNC_LASER_CTRL;
  // cmd.data      = can_buffer;
  // cmd.length    = 1;

  // ErrCode ret;
  // ret = canhost.SendStdCmdSync(cmd, 2000);
  // if (ret != E_SUCCESS) {
  //   return ret;
  // }
  // laser_status_ = state ? LASER_ENABLE : LASER_DISABLE;
  return E_SUCCESS;
}

ErrCode ToolHeadLaser::LaserBranchCtrl(bool onoff) {
  CanStdFuncCmd_t cmd;
  uint8_t can_buffer[8];

  can_buffer[0] = onoff;
  cmd.id        = MODULE_FUNC_LASER_BRANCH_CTRL;
  cmd.data      = can_buffer;
  cmd.length    = 1;

  ErrCode ret;
  ret = canhost.SendStdCmdSync(cmd, 2000);
  if (ret != E_SUCCESS) {
    return ret;
  }
  half_power_mode_ = onoff;
  return E_SUCCESS;
}

ErrCode ToolHeadLaser::LaserGetHWVersion() {
  CanStdFuncCmd_t cmd;
  uint8_t buff[1] = {0};

  cmd.id        = MODULE_FUNC_GET_HW_VERSION;
  cmd.data      = buff;
  cmd.length    = 1;

  ErrCode ret;
  ret = canhost.SendStdCmdSync(cmd, 2000);
  if (ret != E_SUCCESS) {
    return ret;
  }

  SSTP_Event_t event_tmp = {EID_SETTING_ACK, SETTINGS_OPC_GET_LASER_HW_VERSION};
  buff[0] = cmd.data[0];
  event_tmp.length = 1;
  event_tmp.data = buff;
  hmi.Send(event_tmp);

  return E_SUCCESS;
}

ErrCode ToolHeadLaser::SetCrossLight(SSTP_Event_t &event) {
  uint8_t buff[1];

  if (event.length < 1) {
    LOG_E("Hmi SetCrossLight parameter error\n");
    buff[0] = E_PARAM;
  }
  else {
    buff[0] = SetCrossLightCAN(!!event.data[0]);
  }

  SSTP_Event_t event_hmi = {EID_SETTING_ACK, SETTINGS_OPC_SET_CROSSLIGHT};
  event_hmi.length = 1;
  event_hmi.data = buff;

  return hmi.Send(event_hmi);
}

ErrCode ToolHeadLaser::GetCrossLight(SSTP_Event_t &event) {
  bool sw;
  uint8_t buff[2];

  SSTP_Event_t event_tmp = {EID_SETTING_ACK, SETTINGS_OPC_GET_CROSSLIGHT};
  buff[0] = GetCrossLightCAN(sw);
  buff[1] = sw;
  event_tmp.length = 2;
  event_tmp.data = buff;

  return hmi.Send(event_tmp);
}

ErrCode ToolHeadLaser::SetFireSensorSensitivity(SSTP_Event_t &event) {
  uint8_t buff[1];

  if (event.length < 2) {
    LOG_E("Hmi SetFireSensorSensitivity parameter error\n");
    buff[0] = E_PARAM;
  }
  else {
    buff[0] = SetFireSensorSensitivityCAN(event.data[0] | (event.data[1] << 8));
  }

  SSTP_Event_t event_hmi = {EID_SETTING_ACK, SETTINGS_OPC_SET_FIRE_SENSOR_SENSITIVITY};
  event_hmi.length = 1;
  event_hmi.data = buff;

  return hmi.Send(event_hmi);
}

ErrCode ToolHeadLaser::GetFireSensorSensitivity(SSTP_Event_t &event) {
  uint16_t fss;
  uint8_t buff[8];

  SSTP_Event_t event_tmp = {EID_SETTING_ACK, SETTINGS_OPC_GET_FIRE_SENSOR_SENSITIVITY};
  buff[0] = GetFireSensorSensitivityCAN(fss);
  buff[1] = fss & 0xFF;
  buff[2] = (fss >> 8) & 0xFF;
  event_tmp.length = 3;
  event_tmp.data = buff;

  return hmi.Send(event_tmp);
}

ErrCode ToolHeadLaser::SetFireSensorReportTime(SSTP_Event_t &event) {
  uint8_t buff[1];

  if (event.length < 1) {
    LOG_E("Hmi SetFireSensorReportTime parameter error\n");
    buff[0] = E_PARAM;
  }
  else {
    buff[0] = SetFireSensorReportTime(event.data[0] | (event.data[1]<<8));
  }

  SSTP_Event_t event_hmi = {EID_SETTING_ACK, SETTINGS_OPC_SET_FIRE_SENSOR_REPORT_TIME};
  event_hmi.length = 1;
  event_hmi.data = buff;
  return hmi.Send(event_hmi);
}

ErrCode ToolHeadLaser::SetCrosslightOffset(SSTP_Event_t &event) {
  uint8_t buff[1];

  if (event.length < 8) {
    LOG_E("Hmi SetCrosslightOffset parameter error\n");
    buff[0] = E_PARAM;
  }
  else {
    int32_t ix = *(int32_t *)(&event.data[0]), iy = *(int32_t *)(&event.data[4]);
    float fx = (float)ix / 1000.0, fy = (float)iy / 1000.0;
    buff[0] = SetCrossLightOffsetCAN(fx, fy);
  }

  SSTP_Event_t event_hmi = {EID_SETTING_ACK, SETTINGS_OPC_SET_CROSSLIGHT_OFFSET};
  event_hmi.length = 1;
  event_hmi.data = buff;
  return hmi.Send(event_hmi);
}

ErrCode ToolHeadLaser::GetCrosslightOffset(SSTP_Event_t &event) {
  float fx, fy;
  uint8_t buff[9];

  SSTP_Event_t event_tmp = {EID_SETTING_ACK, SETTINGS_OPC_GET_CROSSLIGHT_OFFSET};
  buff[0] = GetCrossLightOffsetCAN(fx, fy);
  int32_t *x = (int32_t *)&buff[1], *y = (int32_t *)&buff[1+4];
  *x = (int32_t)(fx * 1000), *y = (int32_t)(fy * 1000);
  event_tmp.length = 9;
  event_tmp.data = buff;
  return hmi.Send(event_tmp);
}

void ToolHeadLaser::TellSecurityStatus() {
  SendSecurityStatus();

  LOG_I("laser open %dh %dm %ds\n", laser_open_time / (3600), (laser_open_time % (3600) / 60), (laser_open_time % (3600) % 60));
  for (uint8_t i = 0; i < laser_init_total; i++) {
    LOG_I("module%d: port_index: %s mac: 0x%x, dev_id: %d, rec_error: 0x%x, security_status_: 0x%x imu_temp: %d, laser_temp: %d\n",
            i, laser_array[i].port_index_ == 0xFF ? "unknow" : port_info_str[laser_array[i].port_index_], laser_array[i].mac(), laser_array[i].device_id(), rec_error[i], laser_array[i].security_status_, laser_array[i].imu_temperature_, laser_array[i].laser_temperature_);
  }
  LOG_I("\n\n");
  // SERIAL_ECHOLNPAIR("Laser temp: ", laser->laser_temperature_, ", imu temp: ", laser->imu_temperature_, ", roll: ", laser->roll_, ", pitch: ", laser->pitch_);
}

uint8_t ToolHeadLaser::LaserGetPwmPinState() {
  CanStdFuncCmd_t cmd;
  uint8_t can_buffer[1] = {0};

  cmd.id        = MODULE_FUNC_REPORT_PIN_STATE;
  cmd.data      = can_buffer;
  cmd.length    = 1;

  if (canhost.SendStdCmdSync(cmd, 2000) != E_SUCCESS) {
    return 0xff;
  }

  return cmd.data[0];
}

void ToolHeadLaser::LaserConfirmPinState() {
  CanStdFuncCmd_t cmd;
  uint8_t can_buffer[1] = {0};

  cmd.id        = MODULE_FUNC_CONFIRM_PIN_STATE;
  cmd.data      = can_buffer;
  cmd.length    = 1;

  for (uint8_t i = 0; i < laser_init_total; i++) {
    canhost.SendStdCmd(cmd, i);
  }
}

ErrCode ToolHeadLaser::LaserControlEx(uint8_t state) {
  CanStdFuncCmd_t cmd;
  uint8_t can_buffer[1];

  can_buffer[0] = state;
  cmd.id        = MODULE_FUNC_LASER_CTRL;
  cmd.data      = can_buffer;
  cmd.length    = 1;

  ErrCode ret;
  for (uint8_t i = 0; i < laser_init_total; i++) {
    if (state) {
      if (!laser_array[i].security_status_ && !rec_error[i] && laser_array[i].port_index_ != 0xFF)
        can_buffer[0] = 1;
      else
        can_buffer[0] = 0;
    }
    ret = canhost.SendStdCmdSync(cmd, 2000, 1, i);
    // ret = canhost.SendStdCmd(cmd, i);
    LOG_I("LaserControlEx: modeule index: %d, ret: %d, power: %d\n", i, ret, can_buffer[0]);
  }
  return E_SUCCESS;
}

void ToolHeadLaser::OpenAllLaser(void) {
  LaserControlEx(true);
  SetLaserOutputPinSta(LOW);
}

void ToolHeadLaser::CloseAllLaser(void) {
  SetLaserOutputPinSta(HIGH);
  LaserControlEx(false);
}

void ToolHeadLaser::Process() {
  if (mac_index_ == MODULE_MAC_INDEX_INVALID)
    return;

  if (++timer_in_process_ < 100) return;
  timer_in_process_ = 0;

  // if (laser->device_id_ == MODULE_DEVICE_ID_10W_LASER || laser->device_id_ == MODULE_DEVICE_ID_20W_LASER ||
  //     laser->device_id_ == MODULE_DEVICE_ID_40W_LASER) {
    if (laser_pwm_pin_checked_ == false) {
      // PwmCtrlDirectly(255);
      // pwm_pin_pulldown_state_ = LaserGetPwmPinState();
      // PwmCtrlDirectly(0);
      // pwm_pin_pullup_state_ = LaserGetPwmPinState();
      // if (pwm_pin_pulldown_state_ == 0 && pwm_pin_pullup_state_ == 1) {
        LaserConfirmPinState();
      // }
      laser_pwm_pin_checked_ = true;
    }

    if (need_to_tell_hmi_) {
      need_to_tell_hmi_ = false;
      TellSecurityStatus();
    }

    if (laser_open_time != 0xFFFFFFFF) {
      laser_open_time++;
      GetSecurityStatus();
      // LOG_I("laser open %dh %dm %ds, ", laser_open_time / (3600), (laser_open_time % (3600) / 60), (laser_open_time % (3600) % 60));
      // LOG_I("rec_error: 0x%x, security_status_: 0x%x, laser_temp: %d, imu_tmp: %d, fire_err : %d\n", rec_error,
      //       security_status_, laser_temperature_, imu_temperature_, fire_sensor_trigger_);
    }

    if (start_tick > 0) {
      LOG_I("****start_tick: %d*****\n", start_tick);
      start_tick--;
      if (start_tick == 0) {
        laser_open_time = 0;
        OpenAllLaser();
        SetFanPower(255);
        // if (!security_status_)
        //   SetOutput(100);
        // else
          // LOG_W("****security_status_: 0x%x, open laser fail*****\n", security_status_);
      }
    }

    // TurnoffLaserIfNeeded();
  // }

  // TryCloseFan();
}


void ToolHeadLaser::InlineDisable() {
  planner.laser_inline.status.isEnabled = false;
}


void ToolHeadLaser::SetOutputInline(float power) {
  CheckFan(power);

  SetPower(power);
  planner.laser_inline.power = power_pwm_;

  if (power_pwm_ > 0)
    planner.laser_inline.status.isEnabled = true;
  else
    planner.laser_inline.status.isEnabled = false;

  if ((laser->device_id_ == MODULE_DEVICE_ID_10W_LASER || laser->device_id_ == MODULE_DEVICE_ID_20W_LASER ||
      laser->device_id_ == MODULE_DEVICE_ID_40W_LASER) && power_pwm_ > 0) {
    if (laser_status_ == LASER_DISABLE) {
      // Send the enable command only when the power is disabled
      LaserControl(1);
    }
    // Stop waiting disable
    laser_status_ = LASER_ENABLE;
  }
}


void ToolHeadLaser::TurnOn_ISR(uint16_t power_pwm) {
  if (power_pwm > power_limit_pwm_)
    power_pwm = power_limit_pwm_;

  if (power_pwm > 0)
    state_ = TOOLHEAD_LASER_STATE_ON;
  else
    state_ = TOOLHEAD_LASER_STATE_OFF;

  TimSetPwm(power_pwm);
}