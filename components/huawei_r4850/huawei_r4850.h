#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/canbus/canbus.h"

namespace esphome {
namespace huawei_r4850 {

//================================================== hinzugefügt==========================================
enum PowerDemandCalculation {
  POWER_DEMAND_CALCULATION_DUMB_OEM_BEHAVIOR,
  POWER_DEMAND_CALCULATION_NEGATIVE_MEASUREMENTS_REQUIRED,
  POWER_DEMAND_CALCULATION_RESTART_ON_CROSSING_ZERO,
};

//================================================== hinzugefügt==========================================

class HuaweiR4850Component : public PollingComponent {
 public:
  HuaweiR4850Component(canbus::Canbus *canbus);
  void setup() override;
  void update() override;

  void set_output_voltage(float value, bool offline = false);
  void set_max_output_current(float value, bool offline = false);
  void set_offline_values();

//================================================== hinzugefügt==========================================
  void set_power_demand_calculation(PowerDemandCalculation power_demand_calculation) {
    this->power_demand_calculation_ = power_demand_calculation;
  }
//================================================== hinzugefügt==========================================

  void set_power_sensor(sensor::Sensor *power_sensor) { power_sensor_ = power_sensor; }

  void set_input_voltage_sensor(sensor::Sensor *input_voltage_sensor) { input_voltage_sensor_ = input_voltage_sensor; }
  void set_input_frequency_sensor(sensor::Sensor *input_frequency_sensor) {
    input_frequency_sensor_ = input_frequency_sensor;
  }
  void set_input_current_sensor(sensor::Sensor *input_current_sensor) { input_current_sensor_ = input_current_sensor; }
  void set_input_power_sensor(sensor::Sensor *input_power_sensor) { input_power_sensor_ = input_power_sensor; }
  void set_input_temp_sensor(sensor::Sensor *input_temp_sensor) { input_temp_sensor_ = input_temp_sensor; }
  void set_efficiency_sensor(sensor::Sensor *efficiency_sensor) { efficiency_sensor_ = efficiency_sensor; }
  void set_output_voltage_sensor(sensor::Sensor *output_voltage_sensor) {
    output_voltage_sensor_ = output_voltage_sensor;
  }
  void set_output_current_sensor(sensor::Sensor *output_current_sensor) {
    output_current_sensor_ = output_current_sensor;
  }
  // void set_max_output_current_sensor(sensor::Sensor *max_output_current_sensor) { max_output_current_sensor_ =
  // max_output_current_sensor; }
  void set_output_power_sensor(sensor::Sensor *output_power_sensor) { output_power_sensor_ = output_power_sensor; }
  void set_output_temp_sensor(sensor::Sensor *output_temp_sensor) { output_temp_sensor_ = output_temp_sensor; }

  void set_output_voltage_number(number::Number *output_voltage_number) {
    output_voltage_number_ = output_voltage_number;
  }
  void set_max_output_current_number(number::Number *max_output_current_number) {
    max_output_current_number_ = max_output_current_number;
  }

 protected:
  canbus::Canbus *canbus;
  uint32_t lastUpdate_;

  //================================================== hinzugefügt==========================================
  int32_t last_power_demand_received_{0};
  uint16_t last_power_demand_{0};
  int16_t power_demand_;
  int16_t buffer_;
  int16_t min_power_demand_;
  int16_t max_power_demand_;
  bool zero_output_on_min_power_demand_{true};

  //================================================== hinzugefügt==========================================

  sensor::Sensor *power_sensor_{nullptr};  // power demand hinzugefügt==========================================
  sensor::Sensor *input_voltage_sensor_{nullptr};
  sensor::Sensor *input_frequency_sensor_{nullptr};
  sensor::Sensor *input_current_sensor_{nullptr};
  sensor::Sensor *input_power_sensor_{nullptr};
  sensor::Sensor *input_temp_sensor_{nullptr};
  sensor::Sensor *efficiency_sensor_{nullptr};
  sensor::Sensor *output_voltage_sensor_{nullptr};
  sensor::Sensor *output_current_sensor_{nullptr};
  // sensor::Sensor *max_output_current_sensor_{nullptr};
  sensor::Sensor *output_power_sensor_{nullptr};
  sensor::Sensor *output_temp_sensor_{nullptr};

  number::Number *output_voltage_number_{nullptr};
  number::Number *max_output_current_number_{nullptr};

//================================================== hinzugefügt==========================================
  PowerDemandCalculation power_demand_calculation_{POWER_DEMAND_CALCULATION_DUMB_OEM_BEHAVIOR};

  int16_t calculate_power_demand_(int16_t consumption, uint16_t last_power_demand);
  int16_t calculate_power_demand_negative_measurements_(int16_t consumption, uint16_t last_power_demand);
  int16_t calculate_power_demand_restart_on_crossing_zero_(int16_t consumption, uint16_t last_power_demand);
  int16_t calculate_power_demand_oem_(int16_t consumption);

  //================================================== hinzugefügt==========================================

  void on_frame(uint32_t can_id, bool rtr, std::vector<uint8_t> &data);

  void publish_sensor_state_(sensor::Sensor *sensor, float value);
  void publish_number_state_(number::Number *number, float value);
};

}  // namespace huawei_r4850
}  // namespace esphome
