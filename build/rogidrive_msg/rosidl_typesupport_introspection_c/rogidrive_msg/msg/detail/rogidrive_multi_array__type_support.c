// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rogidrive_msg:msg/RogidriveMultiArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rogidrive_msg/msg/detail/rogidrive_multi_array__rosidl_typesupport_introspection_c.h"
#include "rogidrive_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rogidrive_msg/msg/detail/rogidrive_multi_array__functions.h"
#include "rogidrive_msg/msg/detail/rogidrive_multi_array__struct.h"


// Include directives for member types
// Member `data`
#include "rogidrive_msg/msg/rogidrive_message.h"
// Member `data`
#include "rogidrive_msg/msg/detail/rogidrive_message__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__RogidriveMultiArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rogidrive_msg__msg__RogidriveMultiArray__init(message_memory);
}

void rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__RogidriveMultiArray_fini_function(void * message_memory)
{
  rogidrive_msg__msg__RogidriveMultiArray__fini(message_memory);
}

size_t rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__size_function__RogidriveMultiArray__data(
  const void * untyped_member)
{
  const rogidrive_msg__msg__RogidriveMessage__Sequence * member =
    (const rogidrive_msg__msg__RogidriveMessage__Sequence *)(untyped_member);
  return member->size;
}

const void * rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__get_const_function__RogidriveMultiArray__data(
  const void * untyped_member, size_t index)
{
  const rogidrive_msg__msg__RogidriveMessage__Sequence * member =
    (const rogidrive_msg__msg__RogidriveMessage__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__get_function__RogidriveMultiArray__data(
  void * untyped_member, size_t index)
{
  rogidrive_msg__msg__RogidriveMessage__Sequence * member =
    (rogidrive_msg__msg__RogidriveMessage__Sequence *)(untyped_member);
  return &member->data[index];
}

void rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__fetch_function__RogidriveMultiArray__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rogidrive_msg__msg__RogidriveMessage * item =
    ((const rogidrive_msg__msg__RogidriveMessage *)
    rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__get_const_function__RogidriveMultiArray__data(untyped_member, index));
  rogidrive_msg__msg__RogidriveMessage * value =
    (rogidrive_msg__msg__RogidriveMessage *)(untyped_value);
  *value = *item;
}

void rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__assign_function__RogidriveMultiArray__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rogidrive_msg__msg__RogidriveMessage * item =
    ((rogidrive_msg__msg__RogidriveMessage *)
    rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__get_function__RogidriveMultiArray__data(untyped_member, index));
  const rogidrive_msg__msg__RogidriveMessage * value =
    (const rogidrive_msg__msg__RogidriveMessage *)(untyped_value);
  *item = *value;
}

bool rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__resize_function__RogidriveMultiArray__data(
  void * untyped_member, size_t size)
{
  rogidrive_msg__msg__RogidriveMessage__Sequence * member =
    (rogidrive_msg__msg__RogidriveMessage__Sequence *)(untyped_member);
  rogidrive_msg__msg__RogidriveMessage__Sequence__fini(member);
  return rogidrive_msg__msg__RogidriveMessage__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__RogidriveMultiArray_message_member_array[1] = {
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rogidrive_msg__msg__RogidriveMultiArray, data),  // bytes offset in struct
    NULL,  // default value
    rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__size_function__RogidriveMultiArray__data,  // size() function pointer
    rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__get_const_function__RogidriveMultiArray__data,  // get_const(index) function pointer
    rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__get_function__RogidriveMultiArray__data,  // get(index) function pointer
    rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__fetch_function__RogidriveMultiArray__data,  // fetch(index, &value) function pointer
    rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__assign_function__RogidriveMultiArray__data,  // assign(index, value) function pointer
    rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__resize_function__RogidriveMultiArray__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__RogidriveMultiArray_message_members = {
  "rogidrive_msg__msg",  // message namespace
  "RogidriveMultiArray",  // message name
  1,  // number of fields
  sizeof(rogidrive_msg__msg__RogidriveMultiArray),
  rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__RogidriveMultiArray_message_member_array,  // message members
  rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__RogidriveMultiArray_init_function,  // function to initialize message memory (memory has to be allocated)
  rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__RogidriveMultiArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__RogidriveMultiArray_message_type_support_handle = {
  0,
  &rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__RogidriveMultiArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rogidrive_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rogidrive_msg, msg, RogidriveMultiArray)() {
  rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__RogidriveMultiArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rogidrive_msg, msg, RogidriveMessage)();
  if (!rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__RogidriveMultiArray_message_type_support_handle.typesupport_identifier) {
    rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__RogidriveMultiArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rogidrive_msg__msg__RogidriveMultiArray__rosidl_typesupport_introspection_c__RogidriveMultiArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
