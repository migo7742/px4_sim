// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from traj_utils:msg/Bspline.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__BSPLINE__TRAITS_HPP_
#define TRAJ_UTILS__MSG__DETAIL__BSPLINE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "traj_utils/msg/detail/bspline__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'start_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'pos_pts'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace traj_utils
{

namespace msg
{

inline void to_flow_style_yaml(
  const Bspline & msg,
  std::ostream & out)
{
  out << "{";
  // member: drone_id
  {
    out << "drone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.drone_id, out);
    out << ", ";
  }

  // member: order
  {
    out << "order: ";
    rosidl_generator_traits::value_to_yaml(msg.order, out);
    out << ", ";
  }

  // member: traj_id
  {
    out << "traj_id: ";
    rosidl_generator_traits::value_to_yaml(msg.traj_id, out);
    out << ", ";
  }

  // member: start_time
  {
    out << "start_time: ";
    to_flow_style_yaml(msg.start_time, out);
    out << ", ";
  }

  // member: knots
  {
    if (msg.knots.size() == 0) {
      out << "knots: []";
    } else {
      out << "knots: [";
      size_t pending_items = msg.knots.size();
      for (auto item : msg.knots) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pos_pts
  {
    if (msg.pos_pts.size() == 0) {
      out << "pos_pts: []";
    } else {
      out << "pos_pts: [";
      size_t pending_items = msg.pos_pts.size();
      for (auto item : msg.pos_pts) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: yaw_pts
  {
    if (msg.yaw_pts.size() == 0) {
      out << "yaw_pts: []";
    } else {
      out << "yaw_pts: [";
      size_t pending_items = msg.yaw_pts.size();
      for (auto item : msg.yaw_pts) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: yaw_dt
  {
    out << "yaw_dt: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_dt, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Bspline & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: drone_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.drone_id, out);
    out << "\n";
  }

  // member: order
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "order: ";
    rosidl_generator_traits::value_to_yaml(msg.order, out);
    out << "\n";
  }

  // member: traj_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "traj_id: ";
    rosidl_generator_traits::value_to_yaml(msg.traj_id, out);
    out << "\n";
  }

  // member: start_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_time:\n";
    to_block_style_yaml(msg.start_time, out, indentation + 2);
  }

  // member: knots
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.knots.size() == 0) {
      out << "knots: []\n";
    } else {
      out << "knots:\n";
      for (auto item : msg.knots) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pos_pts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pos_pts.size() == 0) {
      out << "pos_pts: []\n";
    } else {
      out << "pos_pts:\n";
      for (auto item : msg.pos_pts) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: yaw_pts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.yaw_pts.size() == 0) {
      out << "yaw_pts: []\n";
    } else {
      out << "yaw_pts:\n";
      for (auto item : msg.yaw_pts) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: yaw_dt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_dt: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_dt, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Bspline & msg, bool use_flow_style = false)
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
  const traj_utils::msg::Bspline & msg,
  std::ostream & out, size_t indentation = 0)
{
  traj_utils::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use traj_utils::msg::to_yaml() instead")]]
inline std::string to_yaml(const traj_utils::msg::Bspline & msg)
{
  return traj_utils::msg::to_yaml(msg);
}

template<>
inline const char * data_type<traj_utils::msg::Bspline>()
{
  return "traj_utils::msg::Bspline";
}

template<>
inline const char * name<traj_utils::msg::Bspline>()
{
  return "traj_utils/msg/Bspline";
}

template<>
struct has_fixed_size<traj_utils::msg::Bspline>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<traj_utils::msg::Bspline>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<traj_utils::msg::Bspline>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TRAJ_UTILS__MSG__DETAIL__BSPLINE__TRAITS_HPP_
