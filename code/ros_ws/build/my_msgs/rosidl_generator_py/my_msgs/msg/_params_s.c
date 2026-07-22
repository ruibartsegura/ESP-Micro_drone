// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from my_msgs:msg/Params.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "my_msgs/msg/detail/params__struct.h"
#include "my_msgs/msg/detail/params__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool my_msgs__msg__params__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[27];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("my_msgs.msg._params.Params", full_classname_dest, 26) == 0);
  }
  my_msgs__msg__Params * ros_message = _ros_message;
  {  // h_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "h_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->h_max = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // v_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "v_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->v_max = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // land_on_site
    PyObject * field = PyObject_GetAttrString(_pymsg, "land_on_site");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->land_on_site = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * my_msgs__msg__params__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Params */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("my_msgs.msg._params");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Params");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  my_msgs__msg__Params * ros_message = (my_msgs__msg__Params *)raw_ros_message;
  {  // h_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->h_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "h_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // v_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->v_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "v_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // land_on_site
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->land_on_site ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "land_on_site", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
