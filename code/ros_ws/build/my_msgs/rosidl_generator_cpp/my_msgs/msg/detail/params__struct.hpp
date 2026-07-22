// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_msgs:msg/Params.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/params.hpp"


#ifndef MY_MSGS__MSG__DETAIL__PARAMS__STRUCT_HPP_
#define MY_MSGS__MSG__DETAIL__PARAMS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__my_msgs__msg__Params __attribute__((deprecated))
#else
# define DEPRECATED__my_msgs__msg__Params __declspec(deprecated)
#endif

namespace my_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Params_
{
  using Type = Params_<ContainerAllocator>;

  explicit Params_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->h_max = 0.0f;
      this->v_max = 0.0f;
      this->land_on_site = false;
    }
  }

  explicit Params_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->h_max = 0.0f;
      this->v_max = 0.0f;
      this->land_on_site = false;
    }
  }

  // field types and members
  using _h_max_type =
    float;
  _h_max_type h_max;
  using _v_max_type =
    float;
  _v_max_type v_max;
  using _land_on_site_type =
    bool;
  _land_on_site_type land_on_site;

  // setters for named parameter idiom
  Type & set__h_max(
    const float & _arg)
  {
    this->h_max = _arg;
    return *this;
  }
  Type & set__v_max(
    const float & _arg)
  {
    this->v_max = _arg;
    return *this;
  }
  Type & set__land_on_site(
    const bool & _arg)
  {
    this->land_on_site = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_msgs::msg::Params_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_msgs::msg::Params_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_msgs::msg::Params_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_msgs::msg::Params_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_msgs::msg::Params_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_msgs::msg::Params_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_msgs::msg::Params_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_msgs::msg::Params_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_msgs::msg::Params_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_msgs::msg::Params_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_msgs__msg__Params
    std::shared_ptr<my_msgs::msg::Params_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_msgs__msg__Params
    std::shared_ptr<my_msgs::msg::Params_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Params_ & other) const
  {
    if (this->h_max != other.h_max) {
      return false;
    }
    if (this->v_max != other.v_max) {
      return false;
    }
    if (this->land_on_site != other.land_on_site) {
      return false;
    }
    return true;
  }
  bool operator!=(const Params_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Params_

// alias to use template instance with default allocator
using Params =
  my_msgs::msg::Params_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_msgs

#endif  // MY_MSGS__MSG__DETAIL__PARAMS__STRUCT_HPP_
