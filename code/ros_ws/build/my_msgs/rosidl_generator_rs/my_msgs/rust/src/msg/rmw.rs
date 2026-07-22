#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "my_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_msgs__msg__Params() -> *const std::ffi::c_void;
}

#[link(name = "my_msgs__rosidl_generator_c")]
extern "C" {
    fn my_msgs__msg__Params__init(msg: *mut Params) -> bool;
    fn my_msgs__msg__Params__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Params>, size: usize) -> bool;
    fn my_msgs__msg__Params__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Params>);
    fn my_msgs__msg__Params__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Params>, out_seq: *mut rosidl_runtime_rs::Sequence<Params>) -> bool;
}

// Corresponds to my_msgs__msg__Params
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Params {

    // This member is not documented.
    #[allow(missing_docs)]
    pub h_max: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub v_max: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub land_on_site: bool,

}



impl Default for Params {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_msgs__msg__Params__init(&mut msg as *mut _) {
        panic!("Call to my_msgs__msg__Params__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Params {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_msgs__msg__Params__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_msgs__msg__Params__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_msgs__msg__Params__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Params {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Params where Self: Sized {
  const TYPE_NAME: &'static str = "my_msgs/msg/Params";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_msgs__msg__Params() }
  }
}


