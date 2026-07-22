// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_msgs:msg/Params.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/msg/params.hpp"


#ifndef MY_MSGS__MSG__DETAIL__PARAMS__BUILDER_HPP_
#define MY_MSGS__MSG__DETAIL__PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_msgs/msg/detail/params__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_msgs
{

namespace msg
{

namespace builder
{

class Init_Params_land_on_site
{
public:
  explicit Init_Params_land_on_site(::my_msgs::msg::Params & msg)
  : msg_(msg)
  {}
  ::my_msgs::msg::Params land_on_site(::my_msgs::msg::Params::_land_on_site_type arg)
  {
    msg_.land_on_site = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_msgs::msg::Params msg_;
};

class Init_Params_v_max
{
public:
  explicit Init_Params_v_max(::my_msgs::msg::Params & msg)
  : msg_(msg)
  {}
  Init_Params_land_on_site v_max(::my_msgs::msg::Params::_v_max_type arg)
  {
    msg_.v_max = std::move(arg);
    return Init_Params_land_on_site(msg_);
  }

private:
  ::my_msgs::msg::Params msg_;
};

class Init_Params_h_max
{
public:
  Init_Params_h_max()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Params_v_max h_max(::my_msgs::msg::Params::_h_max_type arg)
  {
    msg_.h_max = std::move(arg);
    return Init_Params_v_max(msg_);
  }

private:
  ::my_msgs::msg::Params msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_msgs::msg::Params>()
{
  return my_msgs::msg::builder::Init_Params_h_max();
}

}  // namespace my_msgs

#endif  // MY_MSGS__MSG__DETAIL__PARAMS__BUILDER_HPP_
