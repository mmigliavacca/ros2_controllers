// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "swerve_steering_controller/swerve_steering_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace swerve_steering_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

SwerveSteeringController::SwerveSteeringController() : controller_interface::ControllerInterface() {}

const char * SwerveSteeringController::feedback_type() const
{
  return odom_params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

CallbackReturn SwerveSteeringController::on_init()
{
  try
  {
    // with the lifecycle node being initialized, we can declare parameters
    auto_declare<std::vector<std::string>>("wheels", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("holders", std::vector<std::string>());

    auto_declare<std::vector<double>>("radii", std::vector<double>());
    auto_declare<std::vector<bool>>("limitless", std::vector<bool>());
    auto_declare<std::vector<double>>("offsets", std::vector<double>());
    auto_declare<std::vector<double>>("positions", std::vector<double>());
    auto_declare<std::vector<double>>("limits", std::vector<double>());

    auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id);
    auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id);
    auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
    auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());

    auto_declare<bool>("open_loop", odom_params_.open_loop);
    auto_declare<bool>("position_feedback", odom_params_.position_feedback);
    auto_declare<bool>("enable_odom_tf", odom_params_.enable_odom_tf);

    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
    auto_declare<bool>("publish_limited_velocity", publish_limited_velocity_);
    auto_declare<int>("velocity_rolling_window_size", 10);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

    auto_declare<bool>("linear.x.has_velocity_limits", false);
    auto_declare<bool>("linear.x.has_acceleration_limits", false);
    auto_declare<bool>("linear.x.has_jerk_limits", false);
    auto_declare<double>("linear.x.max_velocity", NAN);
    auto_declare<double>("linear.x.min_velocity", NAN);
    auto_declare<double>("linear.x.max_acceleration", NAN);
    auto_declare<double>("linear.x.min_acceleration", NAN);
    auto_declare<double>("linear.x.max_jerk", NAN);
    auto_declare<double>("linear.x.min_jerk", NAN);

    auto_declare<bool>("angular.z.has_velocity_limits", false);
    auto_declare<bool>("angular.z.has_acceleration_limits", false);
    auto_declare<bool>("angular.z.has_jerk_limits", false);
    auto_declare<double>("angular.z.max_velocity", NAN);
    auto_declare<double>("angular.z.min_velocity", NAN);
    auto_declare<double>("angular.z.max_acceleration", NAN);
    auto_declare<double>("angular.z.min_acceleration", NAN);
    auto_declare<double>("angular.z.max_jerk", NAN);
    auto_declare<double>("angular.z.min_jerk", NAN);
    auto_declare<double>("publish_rate", publish_rate_);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration SwerveSteeringController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : wheel_names_)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : holder_names_)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration SwerveSteeringController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : wheel_names_)
  {
    conf_names.push_back(joint_name + "/" + feedback_type());
  }
  for (const auto & joint_name : holder_names_)
  {
    conf_names.push_back(joint_name + "/" + feedback_type());
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type SwerveSteeringController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto logger = node_->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  const auto current_time = time;

  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = current_time - last_command_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.linear.y = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  Twist current_command = *last_command_msg;
  double & linear_x_command = current_command.twist.linear.x;
  double & linear_y_command = current_command.twist.linear.x;
  double & angular_command = current_command.twist.angular.z;

  if (odom_params_.open_loop)
  {
    //odometry_.updateOpenLoop(linear_command, angular_command, current_time);
  }
  else
  {
    // TODO: ODOM
    /*
    double left_feedback_mean = 0.0;
    double right_feedback_mean = 0.0;
    for (size_t index = 0; index < wheels.wheels_per_side; ++index)
    {
      const double left_feedback = registered_left_wheel_handles_[index].feedback.get().get_value();
      const double right_feedback =
        registered_right_wheel_handles_[index].feedback.get().get_value();

      if (std::isnan(left_feedback) || std::isnan(right_feedback))
      {
        RCLCPP_ERROR(
          logger, "Either the left or right wheel %s is invalid for index [%zu]", feedback_type(),
          index);
        return controller_interface::return_type::ERROR;
      }

      left_feedback_mean += left_feedback;
      right_feedback_mean += right_feedback;
    }
    left_feedback_mean /= wheels.wheels_per_side;
    right_feedback_mean /= wheels.wheels_per_side;

    if (odom_params_.position_feedback)
    {
      odometry_.update(left_feedback_mean, right_feedback_mean, current_time);
    }
    else
    {
      odometry_.updateFromVelocity(left_feedback_mean, right_feedback_mean, current_time);
    }
    */
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  if (previous_publish_timestamp_ + publish_period_ < current_time)
  {
    previous_publish_timestamp_ += publish_period_;

    if (realtime_odometry_publisher_->trylock())
    {
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = current_time;
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x = odometry_.getLinearX();
      odometry_message.twist.twist.linear.x = odometry_.getLinearY();
      odometry_message.twist.twist.angular.z = odometry_.getAngular();
      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (odom_params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
      auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = current_time;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  const auto update_dt = current_time - previous_update_timestamp_;
  previous_update_timestamp_ = current_time;

  auto & last_command = previous_commands_.back().twist;
  auto & second_to_last_command = previous_commands_.front().twist;
  limiter_linear_x_.limit(
    linear_x_command, last_command.linear.x, second_to_last_command.linear.x, update_dt.seconds());
  limiter_linear_y_.limit(
    linear_y_command, last_command.linear.y, second_to_last_command.linear.y, update_dt.seconds());
  limiter_angular_.limit(
    angular_command, last_command.angular.z, second_to_last_command.angular.z, update_dt.seconds());

  previous_commands_.pop();
  previous_commands_.emplace(current_command);

  //    Publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = current_time;
    limited_velocity_command.twist = current_command.twist;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  // Compute wheels velocities and holders positions:
  std::vector<double> desired_velocities,desired_positions;

  for (size_t i = 0; i < wheel_joints_size_; ++i)
  {
    double current_angle = registered_holder_handles_[i].feedback.get().get_value();
    
    double wheel_vx = linear_x_command - angular_command * wheels_[i].position[1] - wheels_[i].offset * cos(current_angle);
    double wheel_vy = linear_y_command + angular_command * wheels_[i].position[0] + wheels_[i].offset * sin(current_angle);
    
    //get the required wheel omega and the required wheel steering angle 
    double w_w  = sqrt(pow(wheel_vx, 2) + pow(wheel_vy, 2)) / wheels_[i].radius;
    double w_th = atan2(wheel_vy, wheel_vx);

    //process the requested w,th in the wheel class
    wheels_[i].set_current_angle(current_angle); //to keep the wheel object updated

    wheels_[i].set_command_velocity(w_w);
    wheels_[i].set_command_angle(w_th); //this will do the closest angle calculation and set it in the object.
  }

  // Set joints:
  for (size_t i = 0; i < wheel_joints_size_; ++i)
  {
    registered_wheel_handles_[i].setpoint.get().set_value(wheels_[i].get_command_velocity());
    registered_holder_handles_[i].setpoint.get().set_value(wheels_[i].get_command_angle());
  }

  return controller_interface::return_type::OK;
}

CallbackReturn SwerveSteeringController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = node_->get_logger();

  // update parameters
  wheel_names_ = node_->get_parameter("wheels").as_string_array();
  holder_names_ = node_->get_parameter("holders").as_string_array();

  if (wheel_names_.size() != holder_names_.size())
  {
    RCLCPP_ERROR(
      logger, "The number of  wheels [%zu] and the number of holders [%zu] are different",
      wheel_names_.size(), holder_names_.size());
    return CallbackReturn::ERROR;
  }

  if (wheel_names_.empty())
  {
    RCLCPP_ERROR(logger, "Wheel names parameter is empty!");
    return CallbackReturn::ERROR;
  }

  wheel_joints_size_ = wheel_names_.size();

  auto wheel_radii = node_->get_parameter("radii").as_double_array();
  auto wheel_positions = node_->get_parameter("positions").as_double_array();
  auto wheel_offsets = node_->get_parameter("offsets").as_double_array();
  auto wheel_limitless = node_->get_parameter("limitless").as_bool_array();
  auto wheel_limits = node_->get_parameter("limits").as_double_array();

  std::vector<std::array<double,2>> positions;
  for (size_t i = 0; i < wheel_joints_size_; ++i)
  {
    auto w = wheel(wheel_radii[i], {wheel_positions[i*2], wheel_positions[i*2+1]}, wheel_offsets[i], wheel_limitless[i], {wheel_limits[i*2], wheel_limits[i*2+1]});
		wheels_.push_back(w);
    positions.push_back({wheel_positions[i*2], wheel_positions[i*2+1]});
  }

  odometry_.setWheelsParams(wheel_radii, positions);

  odometry_.setVelocityRollingWindowSize(
    node_->get_parameter("velocity_rolling_window_size").as_int());

  odom_params_.odom_frame_id = node_->get_parameter("odom_frame_id").as_string();
  odom_params_.base_frame_id = node_->get_parameter("base_frame_id").as_string();

  auto pose_diagonal = node_->get_parameter("pose_covariance_diagonal").as_double_array();
  std::copy(
    pose_diagonal.begin(), pose_diagonal.end(), odom_params_.pose_covariance_diagonal.begin());

  auto twist_diagonal = node_->get_parameter("twist_covariance_diagonal").as_double_array();
  std::copy(
    twist_diagonal.begin(), twist_diagonal.end(), odom_params_.twist_covariance_diagonal.begin());

  odom_params_.open_loop = node_->get_parameter("open_loop").as_bool();
  odom_params_.position_feedback = node_->get_parameter("position_feedback").as_bool();
  odom_params_.enable_odom_tf = node_->get_parameter("enable_odom_tf").as_bool();

  cmd_vel_timeout_ = std::chrono::milliseconds{
    static_cast<int>(node_->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
  publish_limited_velocity_ = node_->get_parameter("publish_limited_velocity").as_bool();
  use_stamped_vel_ = node_->get_parameter("use_stamped_vel").as_bool();

  try
  {
    limiter_linear_x_ = SpeedLimiter(
      node_->get_parameter("linear.x.has_velocity_limits").as_bool(),
      node_->get_parameter("linear.x.has_acceleration_limits").as_bool(),
      node_->get_parameter("linear.x.has_jerk_limits").as_bool(),
      node_->get_parameter("linear.x.min_velocity").as_double(),
      node_->get_parameter("linear.x.max_velocity").as_double(),
      node_->get_parameter("linear.x.min_acceleration").as_double(),
      node_->get_parameter("linear.x.max_acceleration").as_double(),
      node_->get_parameter("linear.x.min_jerk").as_double(),
      node_->get_parameter("linear.x.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error configuring linear X speed limiter: %s", e.what());
  }

  try
  {
    limiter_linear_y_ = SpeedLimiter(
      node_->get_parameter("linear.y.has_velocity_limits").as_bool(),
      node_->get_parameter("linear.y.has_acceleration_limits").as_bool(),
      node_->get_parameter("linear.y.has_jerk_limits").as_bool(),
      node_->get_parameter("linear.y.min_velocity").as_double(),
      node_->get_parameter("linear.y.max_velocity").as_double(),
      node_->get_parameter("linear.y.min_acceleration").as_double(),
      node_->get_parameter("linear.y.max_acceleration").as_double(),
      node_->get_parameter("linear.y.min_jerk").as_double(),
      node_->get_parameter("linear.y.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error configuring linear Y speed limiter: %s", e.what());
  }

  try
  {
    limiter_angular_ = SpeedLimiter(
      node_->get_parameter("angular.z.has_velocity_limits").as_bool(),
      node_->get_parameter("angular.z.has_acceleration_limits").as_bool(),
      node_->get_parameter("angular.z.has_jerk_limits").as_bool(),
      node_->get_parameter("angular.z.min_velocity").as_double(),
      node_->get_parameter("angular.z.max_velocity").as_double(),
      node_->get_parameter("angular.z.min_acceleration").as_double(),
      node_->get_parameter("angular.z.max_acceleration").as_double(),
      node_->get_parameter("angular.z.min_jerk").as_double(),
      node_->get_parameter("angular.z.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error configuring angular speed limiter: %s", e.what());
  }

  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ =
      node_->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = node_->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            node_->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = node_->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
      });
  }
  else
  {
    velocity_command_unstamped_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }

        // Write fake header in the stored stamped command
        std::shared_ptr<Twist> twist_stamped;
        received_velocity_msg_ptr_.get(twist_stamped);
        twist_stamped->twist = *msg;
        twist_stamped->header.stamp = node_->get_clock()->now();
      });
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_params_.odom_frame_id;
  odometry_message.child_frame_id = odom_params_.base_frame_id;

  // limit the publication on the topics /odom and /tf
  publish_rate_ = node_->get_parameter("publish_rate").as_double();
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  previous_publish_timestamp_ = node_->get_clock()->now();

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = odom_params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] =
      odom_params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = node_->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_params_.odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = odom_params_.base_frame_id;

  previous_update_timestamp_ = node_->get_clock()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveSteeringController::on_activate(const rclcpp_lifecycle::State &)
{
  const auto wheels_result = configure_joints(wheel_names_, registered_wheel_handles_);
  const auto holders_result = configure_joints(holder_names_, registered_holder_handles_);

  if (wheels_result == CallbackReturn::ERROR || holders_result == CallbackReturn::ERROR)
  {
    return CallbackReturn::ERROR;
  }

  if (registered_wheel_handles_.empty() || registered_holder_handles_.empty())
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Either wheel interfaces, holder interfaces are non existent");
    return CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveSteeringController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveSteeringController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveSteeringController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool SwerveSteeringController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  registered_wheel_handles_.clear();
  registered_holder_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

CallbackReturn SwerveSteeringController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void SwerveSteeringController::halt()
{
  const auto halt_joints = [](auto & joint_handles) {
    for (const auto & joint_handle : joint_handles)
    {
      joint_handle.setpoint.get().set_value(0.0);
    }
  };

  halt_joints(registered_wheel_handles_);
  halt_joints(registered_holder_handles_);
}

CallbackReturn SwerveSteeringController::configure_joints(
  const std::vector<std::string> & joint_names,
  std::vector<JointHandle> & registered_handles)
{
  auto logger = node_->get_logger();

  if (joint_names.empty())
  {
    RCLCPP_ERROR(logger, "No joint names specified");
    return CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(joint_names.size());
  for (const auto & joint_name : joint_names)
  {
    const auto interface_name = feedback_type();
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&joint_name, &interface_name](const auto & interface) {
        return interface.get_name() == joint_name &&
               interface.get_interface_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", joint_name.c_str());
      return CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&joint_name](const auto & interface) {
        return interface.get_name() == joint_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", joint_name.c_str());
      return CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      JointHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }

  return CallbackReturn::SUCCESS;
}
}  // namespace swerve_steering_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  swerve_steering_controller::SwerveSteeringController, controller_interface::ControllerInterface)
