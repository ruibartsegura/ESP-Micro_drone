// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_msgs:srv/Takeoff.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_msgs/srv/takeoff.hpp"


#ifndef MY_MSGS__SRV__DETAIL__TAKEOFF__TRAITS_HPP_
#define MY_MSGS__SRV__DETAIL__TAKEOFF__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_msgs/srv/detail/takeoff__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Takeoff_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Takeoff_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Takeoff_Request & msg, bool use_flow_style = false)
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

}  // namespace my_msgs

namespace rosidl_generator_traits
{

[[deprecated("use my_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_msgs::srv::Takeoff_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const my_msgs::srv::Takeoff_Request & msg)
{
  return my_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<my_msgs::srv::Takeoff_Request>()
{
  return "my_msgs::srv::Takeoff_Request";
}

template<>
inline const char * name<my_msgs::srv::Takeoff_Request>()
{
  return "my_msgs/srv/Takeoff_Request";
}

template<>
struct has_fixed_size<my_msgs::srv::Takeoff_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<my_msgs::srv::Takeoff_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<my_msgs::srv::Takeoff_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace my_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Takeoff_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Takeoff_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Takeoff_Response & msg, bool use_flow_style = false)
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

}  // namespace my_msgs

namespace rosidl_generator_traits
{

[[deprecated("use my_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_msgs::srv::Takeoff_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const my_msgs::srv::Takeoff_Response & msg)
{
  return my_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<my_msgs::srv::Takeoff_Response>()
{
  return "my_msgs::srv::Takeoff_Response";
}

template<>
inline const char * name<my_msgs::srv::Takeoff_Response>()
{
  return "my_msgs/srv/Takeoff_Response";
}

template<>
struct has_fixed_size<my_msgs::srv::Takeoff_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_msgs::srv::Takeoff_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_msgs::srv::Takeoff_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace my_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Takeoff_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
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
  const Takeoff_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Takeoff_Event & msg, bool use_flow_style = false)
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

}  // namespace my_msgs

namespace rosidl_generator_traits
{

[[deprecated("use my_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_msgs::srv::Takeoff_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const my_msgs::srv::Takeoff_Event & msg)
{
  return my_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<my_msgs::srv::Takeoff_Event>()
{
  return "my_msgs::srv::Takeoff_Event";
}

template<>
inline const char * name<my_msgs::srv::Takeoff_Event>()
{
  return "my_msgs/srv/Takeoff_Event";
}

template<>
struct has_fixed_size<my_msgs::srv::Takeoff_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_msgs::srv::Takeoff_Event>
  : std::integral_constant<bool, has_bounded_size<my_msgs::srv::Takeoff_Request>::value && has_bounded_size<my_msgs::srv::Takeoff_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<my_msgs::srv::Takeoff_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<my_msgs::srv::Takeoff>()
{
  return "my_msgs::srv::Takeoff";
}

template<>
inline const char * name<my_msgs::srv::Takeoff>()
{
  return "my_msgs/srv/Takeoff";
}

template<>
struct has_fixed_size<my_msgs::srv::Takeoff>
  : std::integral_constant<
    bool,
    has_fixed_size<my_msgs::srv::Takeoff_Request>::value &&
    has_fixed_size<my_msgs::srv::Takeoff_Response>::value
  >
{
};

template<>
struct has_bounded_size<my_msgs::srv::Takeoff>
  : std::integral_constant<
    bool,
    has_bounded_size<my_msgs::srv::Takeoff_Request>::value &&
    has_bounded_size<my_msgs::srv::Takeoff_Response>::value
  >
{
};

template<>
struct is_service<my_msgs::srv::Takeoff>
  : std::true_type
{
};

template<>
struct is_service_request<my_msgs::srv::Takeoff_Request>
  : std::true_type
{
};

template<>
struct is_service_response<my_msgs::srv::Takeoff_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MY_MSGS__SRV__DETAIL__TAKEOFF__TRAITS_HPP_
