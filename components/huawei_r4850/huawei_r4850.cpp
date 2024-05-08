#include "huawei_r4850.h"
#include "esphome/core/application.h"
#include "esphome/core/base_automation.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

namespace esphome {
namespace huawei_r4850 {

static const char *const TAG = "huawei_r4850";

static const uint32_t CAN_ID_REQUEST = 0x108040FE;
static const uint32_t CAN_ID_DATA = 0x1081407F;
static const uint32_t CAN_ID_SET = 0x108180FE;

static const uint8_t R48xx_DATA_INPUT_POWER = 0x70;
static const uint8_t R48xx_DATA_INPUT_FREQ = 0x71;
static const uint8_t R48xx_DATA_INPUT_CURRENT = 0x72;
static const uint8_t R48xx_DATA_OUTPUT_POWER = 0x73;
static const uint8_t R48xx_DATA_EFFICIENCY = 0x74;
static const uint8_t R48xx_DATA_OUTPUT_VOLTAGE = 0x75;
static const uint8_t R48xx_DATA_OUTPUT_CURRENT_MAX = 0x76;
static const uint8_t R48xx_DATA_INPUT_VOLTAGE = 0x78;
static const uint8_t R48xx_DATA_OUTPUT_TEMPERATURE = 0x7F;
static const uint8_t R48xx_DATA_INPUT_TEMPERATURE = 0x80;
static const uint8_t R48xx_DATA_OUTPUT_CURRENT = 0x81;
static const uint8_t R48xx_DATA_OUTPUT_CURRENT1 = 0x82;

HuaweiR4850Component::HuaweiR4850Component(canbus::Canbus *canbus) { this->canbus = canbus; }

void HuaweiR4850Component::setup() {
  Automation<std::vector<uint8_t>, uint32_t, bool> *automation;
  LambdaAction<std::vector<uint8_t>, uint32_t, bool> *lambdaaction;
  canbus::CanbusTrigger *canbus_canbustrigger;
  //================================================== hinzugefügt==========================================
  this->power_sensor_->add_on_state_callback([this](float state) {
    if (std::isnan(state))
      return;

    this->power_demand_ = this->calculate_power_demand_((int16_t) ceilf(state), this->last_power_demand_);
    ESP_LOGD(TAG, "'%s': New calculated demand: %d / last demand: %d", "modbusgerät", this->power_demand_,
             this->last_power_demand_);
    this->last_power_demand_received_ = millis();
  });
  //================================================== hinzugefügt==========================================

  canbus_canbustrigger = new canbus::CanbusTrigger(this->canbus, 0, 0, true);
  canbus_canbustrigger->set_component_source("canbus");
  App.register_component(canbus_canbustrigger);
  automation = new Automation<std::vector<uint8_t>, uint32_t, bool>(canbus_canbustrigger);
  auto cb = [=](std::vector<uint8_t> x, uint32_t can_id, bool remote_transmission_request) -> void {
    this->on_frame(can_id, remote_transmission_request, x);
  };
  lambdaaction = new LambdaAction<std::vector<uint8_t>, uint32_t, bool>(cb);
  automation->add_actions({lambdaaction});
}

void HuaweiR4850Component::update() {
  ESP_LOGD(TAG, "Sending request message");
  std::vector<uint8_t> data = {0, 0, 0, 0, 0, 0, 0, 0};
  this->canbus->send_data(CAN_ID_REQUEST, true, data);

  // no new value for 5* intervall -> set sensors to NAN)
  if (millis() - lastUpdate_ > this->update_interval_ * 5) {
    this->publish_sensor_state_(this->input_power_sensor_, NAN);
    this->publish_sensor_state_(this->input_voltage_sensor_, NAN);
    this->publish_sensor_state_(this->input_current_sensor_, NAN);
    this->publish_sensor_state_(this->input_temp_sensor_, NAN);
    this->publish_sensor_state_(this->input_frequency_sensor_, NAN);
    this->publish_sensor_state_(this->output_power_sensor_, NAN);
    this->publish_sensor_state_(this->output_current_sensor_, NAN);
    this->publish_sensor_state_(this->output_voltage_sensor_, NAN);
    this->publish_sensor_state_(this->output_temp_sensor_, NAN);
    this->publish_sensor_state_(this->efficiency_sensor_, NAN);
    this->publish_number_state_(this->max_output_current_number_, NAN);
  }
}

//================================================== hinzugefügt==========================================

int16_t HuaweiR4850Component::calculate_power_demand_(int16_t consumption, uint16_t last_power_demand) {
  if (this->power_demand_calculation_ == POWER_DEMAND_CALCULATION_NEGATIVE_MEASUREMENTS_REQUIRED) {
    return this->calculate_power_demand_negative_measurements_(consumption, last_power_demand);
  }

  if (this->power_demand_calculation_ == POWER_DEMAND_CALCULATION_RESTART_ON_CROSSING_ZERO) {
    return this->calculate_power_demand_restart_on_crossing_zero_(consumption, last_power_demand);
  }

  return this->calculate_power_demand_oem_(consumption);
}

int16_t HuaweiR4850Component::calculate_power_demand_negative_measurements_(int16_t consumption,
                                                                            uint16_t last_power_demand) {
  ESP_LOGD(TAG, "'%s': Using the new method to calculate the power demand: %d %d", "modbusgerät", consumption,
           last_power_demand);

  // importing_now   consumption   buffer   last_power_demand   power_demand   return
  //     1000           1010         10          500               1500         900
  //      400            410         10          500                900         900
  //      300            310         10          500                800         800
  //       10             20         10          500                510         510
  //        0             10         10          500                500         500
  //     -200           -190         10          500                300         300
  //     -500           -490         10          500                  0           0
  //     -700           -690         10          500               -200           0
  int16_t importing_now = consumption - this->buffer_;
  int16_t power_demand = importing_now + last_power_demand;

  if (power_demand >= this->max_power_demand_) {
    return this->max_power_demand_;
  }

  if (power_demand < this->min_power_demand_) {
    return (this->zero_output_on_min_power_demand_) ? 0 : this->min_power_demand_;
  }

  return power_demand;
}

int16_t HuaweiR4850Component::calculate_power_demand_restart_on_crossing_zero_(int16_t consumption,
                                                                               uint16_t last_power_demand) {
  ESP_LOGD(TAG, "'%s': Using the restart on crossing zero method to calculate the power demand: %d %d", "modbusgerät",
           consumption, last_power_demand);
  if (this->buffer_ <= 0) {
    ESP_LOGE(TAG,
             "A non-positive buffer value (%d) doesn't make sense if you are using the restart on crossing zero method",
             this->buffer_);
  }

  // importing_now   consumption   buffer   last_power_demand   power_demand   return
  //     1000           1010         10          500               1500         900
  //      400            410         10          500                900         900
  //      300            310         10          500                800         800
  //       10             20         10          500                510         510
  //        5             15         10          500                505         505
  //        0             10         10          500                500         500
  //      -10              0         10          500                490           0
  //     -200           -190         10          500                300           0
  //     -500           -490         10          500                  0           0
  //     -700           -690         10          500               -200           0
  if (consumption <= 0) {
    return 0;
  }

  return this->calculate_power_demand_negative_measurements_(consumption, last_power_demand);
}

int16_t HuaweiR4850Component::calculate_power_demand_oem_(int16_t consumption) {
  ESP_LOGD(TAG, "'%s': Using the dumb OEM method to calculate the power demand: %d", "modbusgerät", consumption);

  // 5000 > 2000 + 10: 2000
  // 2011 > 2000 + 10: 2000
  // 2010 > 2000 + 10: continue
  // 500 > 2000 + 10: continue

  if (consumption > this->max_power_demand_ + this->buffer_)
    return this->max_power_demand_;

  // 5000 > 2000: abs(10 - 2000) = 1990 (already handled above!)
  // 2011 > 2000: abs(10 - 2000) = 1990 (already handled above!)
  // 2010 > 2000: abs(10 - 2000) = 1990
  // 2001 > 2000: abs(10 - 2000) = 1990
  // 2000 > 2000: continue
  //  500 > 2000: continue
  if (consumption > this->max_power_demand_)
    return std::abs(this->buffer_ - this->max_power_demand_);

  // 2001 >= 100: (abs(2001 - 10) + (2001 - 10)) / 2 = 1991 (already handled above!)
  // 2000 >= 100: (abs(2000 - 10) + (2000 - 10)) / 2 = 1990
  //  500 >= 100: (abs(500 - 10) + (500 - 10)) / 2 = 490
  //  100 >= 100: (abs(100 - 10) + (100 - 10)) / 2 = 90
  //   90 >= 100: continue
  if (consumption >= this->min_power_demand_)
    return (int16_t) ((std::abs(consumption - this->buffer_) + (consumption - this->buffer_)) / 2);

  // 90: 0
  return 0;
}

//================================================== hinzugefügt==========================================

void HuaweiR4850Component::set_output_voltage(float value, bool offline) {
  uint8_t functionCode = 0x0;
  if (offline)
    functionCode += 1;
  int32_t raw = 1024.0 * value;
  std::vector<uint8_t> data = {
      0x1, functionCode, 0x0, 0x0, (uint8_t) (raw >> 24), (uint8_t) (raw >> 16), (uint8_t) (raw >> 8), (uint8_t) raw};
  this->canbus->send_data(CAN_ID_SET, true, data);
}

// Hier ist die Stelle zum Strom setzen
void HuaweiR4850Component::set_max_output_current(float value, bool offline) {
  uint8_t functionCode = 0x3;
  if (offline)
    functionCode += 1;
  int32_t raw = 20.0 * value;
  std::vector<uint8_t> data = {
      0x1, functionCode, 0x0, 0x0, (uint8_t) (raw >> 24), (uint8_t) (raw >> 16), (uint8_t) (raw >> 8), (uint8_t) raw};
  this->canbus->send_data(CAN_ID_SET, true, data);
}

void HuaweiR4850Component::set_offline_values() {
  if (output_voltage_number_) {
    set_output_voltage(output_voltage_number_->state, true);
  };
  if (max_output_current_number_) {
    set_max_output_current(max_output_current_number_->state, true);
  }
}

void HuaweiR4850Component::on_frame(uint32_t can_id, bool rtr, std::vector<uint8_t> &data) {
  if (can_id == CAN_ID_DATA) {
    uint32_t value = (data[4] << 24) + (data[5] << 16) + (data[6] << 8) + data[7];
    float conv_value = 0;
    switch (data[1]) {
      case R48xx_DATA_INPUT_POWER:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->input_power_sensor_, conv_value);
        ESP_LOGV(TAG, "Input power: %f", conv_value);
        break;

      case R48xx_DATA_INPUT_FREQ:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->input_frequency_sensor_, conv_value);
        ESP_LOGV(TAG, "Input frequency: %f", conv_value);
        break;

      case R48xx_DATA_INPUT_CURRENT:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->input_current_sensor_, conv_value);
        ESP_LOGV(TAG, "Input current: %f", conv_value);
        break;

      case R48xx_DATA_OUTPUT_POWER:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->output_power_sensor_, conv_value);
        ESP_LOGV(TAG, "Output power: %f", conv_value);
        break;

      case R48xx_DATA_EFFICIENCY:
        conv_value = value / 1024.0 * 100;
        this->publish_sensor_state_(this->efficiency_sensor_, conv_value);
        ESP_LOGV(TAG, "Efficiency: %f", conv_value);
        break;

      case R48xx_DATA_OUTPUT_VOLTAGE:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->output_voltage_sensor_, conv_value);
        ESP_LOGV(TAG, "Output voltage: %f", conv_value);
        break;

      case R48xx_DATA_OUTPUT_CURRENT_MAX:
        conv_value = value / 20.0;
        this->publish_number_state_(this->max_output_current_number_, conv_value);
        ESP_LOGV(TAG, "Max Output current: %f", conv_value);
        break;

      case R48xx_DATA_INPUT_VOLTAGE:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->input_voltage_sensor_, conv_value);
        ESP_LOGV(TAG, "Input voltage: %f", conv_value);
        break;

      case R48xx_DATA_OUTPUT_TEMPERATURE:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->output_temp_sensor_, conv_value);
        ESP_LOGV(TAG, "Output temperature: %f", conv_value);
        break;

      case R48xx_DATA_INPUT_TEMPERATURE:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->input_temp_sensor_, conv_value);
        ESP_LOGV(TAG, "Input temperature: %f", conv_value);
        break;

      case R48xx_DATA_OUTPUT_CURRENT1:
        // printf("Output Current(1) %.02fA\r\n", value / 1024.0);
        // output_current = value / 1024.0;
        break;

      case R48xx_DATA_OUTPUT_CURRENT:
        conv_value = value / 1024.0;
        this->publish_sensor_state_(this->output_current_sensor_, conv_value);
        ESP_LOGV(TAG, "Output current: %f", conv_value);

        // this usually is the last message
        this->lastUpdate_ = millis();
        break;

      default:
        // printf("Unknown parameter 0x%02X, 0x%04X\r\n",frame[1], value);
        break;
    }
  }
}

void HuaweiR4850Component::publish_sensor_state_(sensor::Sensor *sensor, float value) {
  if (sensor) {
    sensor->publish_state(value);
  }
}

void HuaweiR4850Component::publish_number_state_(number::Number *number, float value) {
  if (number) {
    number->publish_state(value);
  }
}

}  // namespace huawei_r4850
}  // namespace esphome
