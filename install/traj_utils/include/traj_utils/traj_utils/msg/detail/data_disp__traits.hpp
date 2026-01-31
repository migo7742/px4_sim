// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from traj_utils:msg/DataDisp.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__DATA_DISP__TRAITS_HPP_
#define TRAJ_UTILS__MSG__DETAIL__DATA_DISP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "traj_utils/msg/detail/data_disp__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace traj_utils
{

namespace msg
{

inline void to_flow_style_yaml(
  const DataDisp & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << ", ";
  }

  // member: b
  {
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << ", ";
  }

  // member: c
  {
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
    out << ", ";
  }

  // member: d
  {
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
    out << ", ";
  }

  // member: e
  {
    out << "e: ";
    rosidl_generator_traits::value_to_yaml(msg.e, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DataDisp & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << "\n";
  }

  // member: c
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
    out << "\n";
  }

  // member: d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
    out << "\n";
  }

  // member: e
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "e: ";
    rosidl_generator_traits::value_to_yaml(msg.e, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DataDisp & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace traj_utils

namespace rosidl_generator_traits
{

[[deprecated("use traj_utils::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const traj_utils::msg::DataDisp & msg,
  std::ostream & out, size_t indentation = 0)
{
  traj_utils::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use traj_utils::msg::to_yaml() instead")]]
inline std::string to_yaml(const traj_utils::msg::DataDisp & msg)
{
  return traj_utils::msg::to_yaml(msg);
}

template<>
inline const char * data_type<traj_utils::msg::DataDisp>()
{
  return "traj_utils::msg::DataDisp";
}

template<>
inline const char * name<traj_utils::msg::DataDisp>()
{
  return "traj_utils/msg/DataDisp";
}

template<>
struct has_fixed_size<traj_utils::msg::DataDisp>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<traj_utils::msg::DataDisp>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<traj_utils::msg::DataDisp>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TRAJ_UTILS__MSG__DETAIL__DATA_DISP__TRAITS_HPP_
