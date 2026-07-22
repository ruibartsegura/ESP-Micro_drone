#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to my_msgs__msg__Params

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Params::default())
  }
}

impl rosidl_runtime_rs::Message for Params {
  type RmwMsg = super::msg::rmw::Params;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        h_max: msg.h_max,
        v_max: msg.v_max,
        land_on_site: msg.land_on_site,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      h_max: msg.h_max,
      v_max: msg.v_max,
      land_on_site: msg.land_on_site,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      h_max: msg.h_max,
      v_max: msg.v_max,
      land_on_site: msg.land_on_site,
    }
  }
}


