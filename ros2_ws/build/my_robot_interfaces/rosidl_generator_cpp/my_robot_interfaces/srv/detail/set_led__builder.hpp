// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_interfaces:srv/SetLed.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES__SRV__DETAIL__SET_LED__BUILDER_HPP_
#define MY_ROBOT_INTERFACES__SRV__DETAIL__SET_LED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_interfaces/srv/detail/set_led__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLed_Request_soc
{
public:
  Init_SetLed_Request_soc()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_robot_interfaces::srv::SetLed_Request soc(::my_robot_interfaces::srv::SetLed_Request::_soc_type arg)
  {
    msg_.soc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::srv::SetLed_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::srv::SetLed_Request>()
{
  return my_robot_interfaces::srv::builder::Init_SetLed_Request_soc();
}

}  // namespace my_robot_interfaces


namespace my_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLed_Response_open
{
public:
  explicit Init_SetLed_Response_open(::my_robot_interfaces::srv::SetLed_Response & msg)
  : msg_(msg)
  {}
  ::my_robot_interfaces::srv::SetLed_Response open(::my_robot_interfaces::srv::SetLed_Response::_open_type arg)
  {
    msg_.open = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::srv::SetLed_Response msg_;
};

class Init_SetLed_Response_states
{
public:
  Init_SetLed_Response_states()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLed_Response_open states(::my_robot_interfaces::srv::SetLed_Response::_states_type arg)
  {
    msg_.states = std::move(arg);
    return Init_SetLed_Response_open(msg_);
  }

private:
  ::my_robot_interfaces::srv::SetLed_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::srv::SetLed_Response>()
{
  return my_robot_interfaces::srv::builder::Init_SetLed_Response_states();
}

}  // namespace my_robot_interfaces

#endif  // MY_ROBOT_INTERFACES__SRV__DETAIL__SET_LED__BUILDER_HPP_
