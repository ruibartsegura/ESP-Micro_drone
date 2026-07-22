// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_msgs:srv/Takeoff.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/srv/takeoff.hpp"


#ifndef MY_MSGS__SRV__DETAIL__TAKEOFF__BUILDER_HPP_
#define MY_MSGS__SRV__DETAIL__TAKEOFF__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_msgs/srv/detail/takeoff__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_msgs
{

namespace srv
{

namespace builder
{

class Init_Takeoff_Request_altitude
{
public:
  Init_Takeoff_Request_altitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_msgs::srv::Takeoff_Request altitude(::my_msgs::srv::Takeoff_Request::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_msgs::srv::Takeoff_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_msgs::srv::Takeoff_Request>()
{
  return my_msgs::srv::builder::Init_Takeoff_Request_altitude();
}

}  // namespace my_msgs


namespace my_msgs
{

namespace srv
{

namespace builder
{

class Init_Takeoff_Response_reason
{
public:
  explicit Init_Takeoff_Response_reason(::my_msgs::srv::Takeoff_Response & msg)
  : msg_(msg)
  {}
  ::my_msgs::srv::Takeoff_Response reason(::my_msgs::srv::Takeoff_Response::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_msgs::srv::Takeoff_Response msg_;
};

class Init_Takeoff_Response_accepted
{
public:
  Init_Takeoff_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Takeoff_Response_reason accepted(::my_msgs::srv::Takeoff_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Takeoff_Response_reason(msg_);
  }

private:
  ::my_msgs::srv::Takeoff_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_msgs::srv::Takeoff_Response>()
{
  return my_msgs::srv::builder::Init_Takeoff_Response_accepted();
}

}  // namespace my_msgs


namespace my_msgs
{

namespace srv
{

namespace builder
{

class Init_Takeoff_Event_response
{
public:
  explicit Init_Takeoff_Event_response(::my_msgs::srv::Takeoff_Event & msg)
  : msg_(msg)
  {}
  ::my_msgs::srv::Takeoff_Event response(::my_msgs::srv::Takeoff_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_msgs::srv::Takeoff_Event msg_;
};

class Init_Takeoff_Event_request
{
public:
  explicit Init_Takeoff_Event_request(::my_msgs::srv::Takeoff_Event & msg)
  : msg_(msg)
  {}
  Init_Takeoff_Event_response request(::my_msgs::srv::Takeoff_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Takeoff_Event_response(msg_);
  }

private:
  ::my_msgs::srv::Takeoff_Event msg_;
};

class Init_Takeoff_Event_info
{
public:
  Init_Takeoff_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Takeoff_Event_request info(::my_msgs::srv::Takeoff_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Takeoff_Event_request(msg_);
  }

private:
  ::my_msgs::srv::Takeoff_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_msgs::srv::Takeoff_Event>()
{
  return my_msgs::srv::builder::Init_Takeoff_Event_info();
}

}  // namespace my_msgs

#endif  // MY_MSGS__SRV__DETAIL__TAKEOFF__BUILDER_HPP_
