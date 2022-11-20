// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
// (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "robot_hardware/hoverboard_driver.hpp"
#define START_FRAME 0xABCD

namespace robot_hardware {
hardware_interface::CallbackReturn HoverboardDriver::on_init(
    const hardware_interface::HardwareInfo& info) {
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }


    hw_positions_.resize(info_.joints.size(),
                         std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(),
                          std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(),
                        std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
      if (joint.command_interfaces.size() != 1) {
        RCLCPP_FATAL(rclcpp::get_logger("HoverboardDriver"),
                     "Joint '%s' has %zu command interfaces found. 2 expected.",
                     joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(
            rclcpp::get_logger("HoverboardDriver"),
            "Joint '%s' have %s command interfaces found. '%s' expected.",
            joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
            "drivewhl_l_joint");
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2) {
        RCLCPP_FATAL(rclcpp::get_logger("HoverboardDriver"),
                     "Joint '%s' has %zu state interface. 2 expected.",
                     joint.name.c_str(), joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name !=
          hardware_interface::HW_IF_POSITION) {
        RCLCPP_FATAL(
            rclcpp::get_logger("HoverboardDriver"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name !=
          hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(
            rclcpp::get_logger("HoverboardDriver"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
            hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
HoverboardDriver::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
HoverboardDriver::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn HoverboardDriver::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if ((port_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("HoverboardDriver"),
                 "Opening serial port %s failed: %d", port_name_.c_str(),
                 port_fd_);
    return CallbackReturn::ERROR;
  }
  struct termios options;
  tcgetattr(port_fd_, &options);
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(port_fd_, TCIFLUSH);
  if (tcsetattr(port_fd_, TCSANOW, &options) <= -1)
  {
    RCLCPP_FATAL(rclcpp::get_logger("HoverboardDriver"),
                 "Setting attributes on port %s failed: %d", port_name_.c_str(),
                 port_fd_);
    return CallbackReturn::ERROR;
  }
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }
    SerialCommand command;
    command.start = (uint16_t)START_FRAME;
    command.steer = (int16_t)0;
    command.speed = (int16_t)0;
    command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);

    int rc = ::write(port_fd_, (const void*)&command, sizeof(command));
    if (rc < 0) {
        RCLCPP_FATAL(rclcpp::get_logger("HoverboardDriver"),
              "Failed to activate! Cannot write to port %s!", port_name_.c_str());
              return CallbackReturn::ERROR;
    }
    int i=0;
    unsigned char c;
    int r=0;
  
    while ((r = ::read(port_fd_, &c, 1)) > 0 && i++ < 1024) protocol_recv_(c);



  RCLCPP_INFO(rclcpp::get_logger("HoverboardDriver"),
              "Successfully activated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HoverboardDriver::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (port_fd_ != -1) close(port_fd_);
  RCLCPP_INFO(rclcpp::get_logger("HoverboardDriver"),
              "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type HoverboardDriver::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (port_fd_ != -1) {
    unsigned char c;
    int i = 0, r = 0;

    while ((r = ::read(port_fd_, &c, 1)) > 0 && i++ < 1024) protocol_recv_(c);

    if (r < 0 && errno != EAGAIN) {
      RCLCPP_WARN(rclcpp::get_logger("HoverboardDriver"),
                  "Reading from serial %s failed: %d", port_name_.c_str(), r);
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HoverboardDriver::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (port_fd_ == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("HoverboardDriver"),
                 "Writing to serial %s failed", port_name_.c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void HoverboardDriver::protocol_recv_(char byte) {
  start_frame = ((uint16_t)(byte) << 8) | (uint8_t)prev_byte;

    // Read the start frame
    if (start_frame == START_FRAME) {
  p = (char*)&msg_;
      *p++ = prev_byte;
      *p++ = byte;
      msg_len = 2;
    } else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback)) {
      // Otherwise just read the message content until the end
      *p++ = byte;
      msg_len++;
    }

    if (msg_len == sizeof(SerialFeedback)) {
      uint16_t checksum =
          (uint16_t)(msg_.start ^ msg_.cmd1 ^ msg_.cmd2 ^ msg_.speedR_meas ^
                     msg_.speedL_meas ^ msg_.wheelR_cnt ^ msg_.wheelL_cnt ^
                     msg_.batVoltage ^ msg_.boardTemp ^ msg_.cmdLed);

      if (msg_.start == START_FRAME && msg_.checksum == checksum) {
            double f;
        f = (double)msg_.batVoltage / 100.0;
        
        RCLCPP_WARN(rclcpp::get_logger("HoverboardDriver"),
                 "bat voltage: %hu ", checksum);
    
        

        // Convert RPM to RAD/S
        // joints[0].vel.data =
        //     direction_correction * (abs(msg.speedL_meas) * 0.10472);
        // joints[1].vel.data =
        //     direction_correction * (abs(msg.speedR_meas) * 0.10472);
        // vel_pub[0].publish(joints[0].vel);
        // vel_pub[1].publish(joints[1].vel);

        // Process encoder values and update odometry
        // on_encoder_update_(msg.wheelR_cnt, msg.wheelL_cnt);
      // } else {
      //   RCLCPP_WARN(rclcpp::get_logger("HoverboardDriver"),
      //            "Writing to serial %s failed", port_name_.c_str());
      }
    }
    prev_byte = byte;
}

}  // namespace robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robot_hardware::HoverboardDriver,
                       hardware_interface::SystemInterface)
