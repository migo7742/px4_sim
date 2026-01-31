// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from traj_utils:msg/Bspline.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "traj_utils/msg/detail/bspline__rosidl_typesupport_introspection_c.h"
#include "traj_utils/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "traj_utils/msg/detail/bspline__functions.h"
#include "traj_utils/msg/detail/bspline__struct.h"


// Include directives for member types
// Member `start_time`
#include "builtin_interfaces/msg/time.h"
// Member `start_time`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `knots`
// Member `yaw_pts`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `pos_pts`
#include "geometry_msgs/msg/point.h"
// Member `pos_pts`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  traj_utils__msg__Bspline__init(message_memory);
}

void traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_fini_function(void * message_memory)
{
  traj_utils__msg__Bspline__fini(message_memory);
}

size_t traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__size_function__Bspline__knots(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_const_function__Bspline__knots(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_function__Bspline__knots(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__fetch_function__Bspline__knots(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_const_function__Bspline__knots(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__assign_function__Bspline__knots(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_function__Bspline__knots(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__resize_function__Bspline__knots(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__size_function__Bspline__pos_pts(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_const_function__Bspline__pos_pts(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_function__Bspline__pos_pts(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__fetch_function__Bspline__pos_pts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_const_function__Bspline__pos_pts(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__assign_function__Bspline__pos_pts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_function__Bspline__pos_pts(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__resize_function__Bspline__pos_pts(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

size_t traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__size_function__Bspline__yaw_pts(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_const_function__Bspline__yaw_pts(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_function__Bspline__yaw_pts(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__fetch_function__Bspline__yaw_pts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_const_function__Bspline__yaw_pts(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__assign_function__Bspline__yaw_pts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_function__Bspline__yaw_pts(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__resize_function__Bspline__yaw_pts(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_message_member_array[8] = {
  {
    "drone_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(traj_utils__msg__Bspline, drone_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "order",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(traj_utils__msg__Bspline, order),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "traj_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(traj_utils__msg__Bspline, traj_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "start_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(traj_utils__msg__Bspline, start_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "knots",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(traj_utils__msg__Bspline, knots),  // bytes offset in struct
    NULL,  // default value
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__size_function__Bspline__knots,  // size() function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_const_function__Bspline__knots,  // get_const(index) function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_function__Bspline__knots,  // get(index) function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__fetch_function__Bspline__knots,  // fetch(index, &value) function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__assign_function__Bspline__knots,  // assign(index, value) function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__resize_function__Bspline__knots  // resize(index) function pointer
  },
  {
    "pos_pts",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(traj_utils__msg__Bspline, pos_pts),  // bytes offset in struct
    NULL,  // default value
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__size_function__Bspline__pos_pts,  // size() function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_const_function__Bspline__pos_pts,  // get_const(index) function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_function__Bspline__pos_pts,  // get(index) function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__fetch_function__Bspline__pos_pts,  // fetch(index, &value) function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__assign_function__Bspline__pos_pts,  // assign(index, value) function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__resize_function__Bspline__pos_pts  // resize(index) function pointer
  },
  {
    "yaw_pts",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(traj_utils__msg__Bspline, yaw_pts),  // bytes offset in struct
    NULL,  // default value
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__size_function__Bspline__yaw_pts,  // size() function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_const_function__Bspline__yaw_pts,  // get_const(index) function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__get_function__Bspline__yaw_pts,  // get(index) function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__fetch_function__Bspline__yaw_pts,  // fetch(index, &value) function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__assign_function__Bspline__yaw_pts,  // assign(index, value) function pointer
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__resize_function__Bspline__yaw_pts  // resize(index) function pointer
  },
  {
    "yaw_dt",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(traj_utils__msg__Bspline, yaw_dt),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_message_members = {
  "traj_utils__msg",  // message namespace
  "Bspline",  // message name
  8,  // number of fields
  sizeof(traj_utils__msg__Bspline),
  traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_message_member_array,  // message members
  traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_init_function,  // function to initialize message memory (memory has to be allocated)
  traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_message_type_support_handle = {
  0,
  &traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_traj_utils
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, traj_utils, msg, Bspline)() {
  traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_message_type_support_handle.typesupport_identifier) {
    traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &traj_utils__msg__Bspline__rosidl_typesupport_introspection_c__Bspline_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
