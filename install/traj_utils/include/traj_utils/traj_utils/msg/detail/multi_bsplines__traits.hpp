// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from traj_utils:msg/MultiBsplines.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__TRAITS_HPP_
#define TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "traj_utils/msg/detail/multi_bsplines__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'traj'
#include "traj_utils/msg/detail/bspline__traits.hpp"

namespace traj_utils
{

namespace msg
{

inline void to_flow_style_yaml(
  const MultiBsplines & msg,
  std::ostream & out)
{
  out << "{";
  // member: drone_id_from
  {
    out << "drone_id_from: ";
    rosidl_generator_traits::value_to_yaml(msg.drone_id_from, out);
    out << ", ";
  }

  // member: traj
  {
    if (msg.traj.size() == 0) {
      out << "traj: []";
    } else {
      out << "traj: [";
      size_t pending_items = msg.traj.size();
      for (auto item : msg.traj) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MultiBsplines & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: drone_id_from
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drone_id_from: ";
    rosidl_generator_traits::value_to_yaml(msg.drone_id_from, out);
    out << "\n";
  }

  // member: traj
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.traj.size() == 0) {
      out << "traj: []\n";
    } else {
      out << "traj:\n";
      for (auto item : msg.traj) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MultiBsplines & msg, bool use_flow_style = false)
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
  const traj_utils::msg::MultiBsplines & msg,
  std::ostream & out, size_t indentation = 0)
{
  traj_utils::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use traj_utils::msg::to_yaml() instead")]]
inline std::string to_yaml(const traj_utils::msg::MultiBsplines & msg)
{
  return traj_utils::msg::to_yaml(msg);
}

template<>
inline const char * data_type<traj_utils::msg::MultiBsplines>()
{
  return "traj_utils::msg::MultiBsplines";
}

template<>
inline const char * name<traj_utils::msg::MultiBsplines>()
{
  return "traj_utils/msg/MultiBsplines";
}

template<>
struct has_fixed_size<traj_utils::msg::MultiBsplines>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<traj_utils::msg::MultiBsplines>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<traj_utils::msg::MultiBsplines>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__TRAITS_HPP_
