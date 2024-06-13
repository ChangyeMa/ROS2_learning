// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_robot_interfaces:srv/SetLed.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES__SRV__DETAIL__SET_LED__TRAITS_HPP_
#define MY_ROBOT_INTERFACES__SRV__DETAIL__SET_LED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_robot_interfaces/srv/detail/set_led__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetLed_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: soc
  {
    out << "soc: ";
    rosidl_generator_traits::value_to_yaml(msg.soc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetLed_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: soc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "soc: ";
    rosidl_generator_traits::value_to_yaml(msg.soc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetLed_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace my_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use my_robot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_robot_interfaces::srv::SetLed_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_interfaces::srv::SetLed_Request & msg)
{
  return my_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_interfaces::srv::SetLed_Request>()
{
  return "my_robot_interfaces::srv::SetLed_Request";
}

template<>
inline const char * name<my_robot_interfaces::srv::SetLed_Request>()
{
  return "my_robot_interfaces/srv/SetLed_Request";
}

template<>
struct has_fixed_size<my_robot_interfaces::srv::SetLed_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<my_robot_interfaces::srv::SetLed_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<my_robot_interfaces::srv::SetLed_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace my_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetLed_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: states
  {
    if (msg.states.size() == 0) {
      out << "states: []";
    } else {
      out << "states: [";
      size_t pending_items = msg.states.size();
      for (auto item : msg.states) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: open
  {
    out << "open: ";
    rosidl_generator_traits::value_to_yaml(msg.open, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetLed_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.states.size() == 0) {
      out << "states: []\n";
    } else {
      out << "states:\n";
      for (auto item : msg.states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: open
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "open: ";
    rosidl_generator_traits::value_to_yaml(msg.open, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetLed_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace my_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use my_robot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_robot_interfaces::srv::SetLed_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_interfaces::srv::SetLed_Response & msg)
{
  return my_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_interfaces::srv::SetLed_Response>()
{
  return "my_robot_interfaces::srv::SetLed_Response";
}

template<>
inline const char * name<my_robot_interfaces::srv::SetLed_Response>()
{
  return "my_robot_interfaces/srv/SetLed_Response";
}

template<>
struct has_fixed_size<my_robot_interfaces::srv::SetLed_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_robot_interfaces::srv::SetLed_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_robot_interfaces::srv::SetLed_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<my_robot_interfaces::srv::SetLed>()
{
  return "my_robot_interfaces::srv::SetLed";
}

template<>
inline const char * name<my_robot_interfaces::srv::SetLed>()
{
  return "my_robot_interfaces/srv/SetLed";
}

template<>
struct has_fixed_size<my_robot_interfaces::srv::SetLed>
  : std::integral_constant<
    bool,
    has_fixed_size<my_robot_interfaces::srv::SetLed_Request>::value &&
    has_fixed_size<my_robot_interfaces::srv::SetLed_Response>::value
  >
{
};

template<>
struct has_bounded_size<my_robot_interfaces::srv::SetLed>
  : std::integral_constant<
    bool,
    has_bounded_size<my_robot_interfaces::srv::SetLed_Request>::value &&
    has_bounded_size<my_robot_interfaces::srv::SetLed_Response>::value
  >
{
};

template<>
struct is_service<my_robot_interfaces::srv::SetLed>
  : std::true_type
{
};

template<>
struct is_service_request<my_robot_interfaces::srv::SetLed_Request>
  : std::true_type
{
};

template<>
struct is_service_response<my_robot_interfaces::srv::SetLed_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MY_ROBOT_INTERFACES__SRV__DETAIL__SET_LED__TRAITS_HPP_
