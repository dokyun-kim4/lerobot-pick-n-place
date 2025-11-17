#include <fmt/ranges.h>

#include <algorithm>
#include <feetech_driver/common.hpp>
#include <feetech_driver/communication_protocol.hpp>
#include <feetech_ros2_driver/feetech_ros2_driver.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace feetech_ros2_driver {


CallbackReturn FeetechHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  // Retrieve hardware parameters from URDF file, stored in `info_` (part of SystemInterface)
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Check USB connection
  const auto usb_port_it = info_.hardware_parameters.find("usb_port"); // this is an iterator
  if (usb_port_it == info_.hardware_parameters.end()) { // if not found
    spdlog::error(
        "FeetechHardware::on_init Hardware parameter [{}] not found!. "
        "Make sure to have <param name=\"usb_port\">/dev/XXXX</param>");
    return CallbackReturn::ERROR;
  }

  // Initialize serial port connection to USB port; Raise error if it fails
  //TODO: read feetech_driver::SerialPort documentation
  auto serial_port = std::make_unique<feetech_driver::SerialPort>(usb_port_it->second);
  if (const auto result = serial_port->configure(); !result) {
    spdlog::error("FeetechHardware::on_init -> {}", result.error());
    return CallbackReturn::ERROR;
  }

  //TODO: read feetech_driver::CommunicationProtocol documentation
  communication_protocol_ = std::make_unique<feetech_driver::CommunicationProtocol>(std::move(serial_port));

  // vectors to hold joint id and joint offset 
  joint_ids_.resize(info_.joints.size(), 0);
  joint_offsets_.resize(info_.joints.size(), 0);

  // Iterate through each joint specified in the URDF file
  for (uint i = 0; i < info_.joints.size(); i++) {
    // Retrieve joint parameters from URDF file
    const auto& joint_params = info_.joints[i].parameters;
    joint_ids_[i] = std::stoi(joint_params.at("id")); // get ID (1 - 6), should correspond to STS internal IDs
    joint_offsets_[i] = [&] { // get <offset>
      if (const auto offset_it = joint_params.find("offset"); offset_it != joint_params.end()) {
        return std::stoi(offset_it->second);
      }
      spdlog::info("Joint '{}' does not specify an offset parameter - Setting it to 0", info_.joints[i].name);
      return 0;
    }();

    for (const auto& [parameter_name, address] : {std::pair{"p_cofficient", SMS_STS_P_COEF},
                                                  {"d_cofficient", SMS_STS_D_COEF},
                                                  {"i_cofficient", SMS_STS_I_COEF}}) {
      if (const auto param_it = joint_params.find(parameter_name); param_it != joint_params.end()) {
        const auto result = communication_protocol_->write( // Write PID coeffs to servo
            joint_ids_[i], address, std::experimental::make_array(static_cast<uint8_t>(std::stoi(param_it->second))));
        if (!result) {
          spdlog::error("FeetechHardwareInterface::on_init -> {}", result.error());
          return CallbackReturn::ERROR;
        }
      }
    }
    // Disable holding torque for joints that do not have command interfaces.
    if (info_.joints[i].command_interfaces.empty()) {
      communication_protocol_->set_torque(joint_ids_[i], false);
    }
  }

  // Not that relevant for our use case; Assume it works
  const auto joint_model_series = joint_ids_ | ranges::views::transform([&](const auto id) {
                                    return communication_protocol_->read_model_number(id)
                                        .and_then(feetech_driver::get_model_name)
                                        .and_then(feetech_driver::get_model_series);
                                  });

  if (std::ranges::any_of(joint_model_series, [](const auto& series) { return !series.has_value(); })) {
    spdlog::error("FeetechHardware::on_init [One of the joints has an error]. Input: {}",
                  ranges::views::zip(joint_ids_, joint_model_series));
    return CallbackReturn::ERROR;
  }

  const auto js = joint_model_series | ranges::views::transform([](const auto& series) { return series.value(); });

  // Currently only supports STS series
  if (ranges::any_of(js, [](const auto& series) { return series != feetech_driver::ModelSeries::kSts; })) {
    spdlog::error("FeetechHardware::on_init [Only STS series is supported]. Input (id, series): {}",
                  ranges::views::zip(joint_ids_, js));
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

// Export current hardware state (position, velocity) to ROS2 control as a StateInterface
std::vector<hardware_interface::StateInterface> FeetechHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
  }

  return state_interfaces;
}

// Export each joint's command interfaces (position) to ROS2 control as a CommandInterface
std::vector<hardware_interface::CommandInterface> FeetechHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  for (uint i = 0; i < info_.joints.size(); i++) {
    if (!info_.joints[i].command_interfaces.empty()) {
      command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]);
    }
  }

  return command_interfaces;
}

// Read current joint states from servos and update state interfaces
hardware_interface::return_type FeetechHardwareInterface::read(const rclcpp::Time& /* time */,
                                                               const rclcpp::Duration& /* period */) {
  // 4 = 2 bytes for position + 2 bytes for speed
  std::vector<std::array<uint8_t, 4>> data;
  data.reserve(joint_ids_.size());
  if (auto result = communication_protocol_->sync_read(joint_ids_, SMS_STS_PRESENT_POSITION_L, &data); !result) {
    spdlog::error("FeetechHardwareInterface::read -> {}", result.error());
    return hardware_interface::return_type::ERROR;
  }
  ranges::for_each(data | ranges::views::enumerate, [&](const auto& values) {
    const auto& [index, readings] = values;
    position_states_[index] = feetech_driver::to_radians(
        feetech_driver::from_sts(feetech_driver::WordBytes{.low = readings[0], .high = readings[1]}) - joint_offsets_[index]
        ); // joint offset used here
    velocity_states_[index] = feetech_driver::to_radians(
        feetech_driver::from_sts(feetech_driver::WordBytes{.low = readings[2], .high = readings[3]})
        );
  });
  return hardware_interface::return_type::OK;
}

// Write command positions from ROS2 control to servos
hardware_interface::return_type FeetechHardwareInterface::write(const rclcpp::Time& /* time */,
                                                                const rclcpp::Duration& /* period */) {
  // Create vectors only for joints that have command interfaces
  std::vector<uint8_t> commanded_joint_ids;
  std::vector<int> commanded_positions;
  std::vector<int> commanded_speeds;
  std::vector<int> commanded_accelerations;

  for (uint i = 0; i < info_.joints.size(); i++) {
    // Only include joints with command interfaces
    if (!info_.joints[i].command_interfaces.empty()) {
      commanded_joint_ids.push_back(joint_ids_[i]);
      commanded_positions.push_back(feetech_driver::from_radians(position_commands_[i]) + joint_offsets_[i]);
      commanded_speeds.push_back(2400);       // Default speed
      commanded_accelerations.push_back(50);  // Default acceleration
    }
  }

  // Only send commands if there are joints to command
  if (!commanded_joint_ids.empty()) {
    const auto write_result = communication_protocol_->sync_write_position(
        commanded_joint_ids, commanded_positions, commanded_speeds, commanded_accelerations);
    if (!write_result) {
      spdlog::error("FeetechHardwareInterface::write -> {}", write_result.error());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

CallbackReturn FeetechHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  // Time/Duration are not used
  read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
  // Set the initial command to current joint positions
  position_commands_ = position_states_;
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  // all joints torque off
  const auto torque_disable_parameters =
      std::vector(joint_ids_.size(), std::experimental::make_array(static_cast<uint8_t>(0)));
  if (const auto result =
          communication_protocol_->sync_write(joint_ids_, SMS_STS_TORQUE_ENABLE, torque_disable_parameters);
      !result) {
    spdlog::error("FeetechHardwareInterface::on_deactivate -> {}", result.error());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace feetech_ros2_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(feetech_ros2_driver::FeetechHardwareInterface, hardware_interface::SystemInterface)
