// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_msgs:msg/Params.idl
// generated code does not contain a copyright notice
#include "my_msgs/msg/detail/params__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
my_msgs__msg__Params__init(my_msgs__msg__Params * msg)
{
  if (!msg) {
    return false;
  }
  // h_max
  // v_max
  // land_on_site
  return true;
}

void
my_msgs__msg__Params__fini(my_msgs__msg__Params * msg)
{
  if (!msg) {
    return;
  }
  // h_max
  // v_max
  // land_on_site
}

bool
my_msgs__msg__Params__are_equal(const my_msgs__msg__Params * lhs, const my_msgs__msg__Params * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // h_max
  if (lhs->h_max != rhs->h_max) {
    return false;
  }
  // v_max
  if (lhs->v_max != rhs->v_max) {
    return false;
  }
  // land_on_site
  if (lhs->land_on_site != rhs->land_on_site) {
    return false;
  }
  return true;
}

bool
my_msgs__msg__Params__copy(
  const my_msgs__msg__Params * input,
  my_msgs__msg__Params * output)
{
  if (!input || !output) {
    return false;
  }
  // h_max
  output->h_max = input->h_max;
  // v_max
  output->v_max = input->v_max;
  // land_on_site
  output->land_on_site = input->land_on_site;
  return true;
}

my_msgs__msg__Params *
my_msgs__msg__Params__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__Params * msg = (my_msgs__msg__Params *)allocator.allocate(sizeof(my_msgs__msg__Params), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_msgs__msg__Params));
  bool success = my_msgs__msg__Params__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_msgs__msg__Params__destroy(my_msgs__msg__Params * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_msgs__msg__Params__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_msgs__msg__Params__Sequence__init(my_msgs__msg__Params__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__Params * data = NULL;

  if (size) {
    data = (my_msgs__msg__Params *)allocator.zero_allocate(size, sizeof(my_msgs__msg__Params), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_msgs__msg__Params__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_msgs__msg__Params__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
my_msgs__msg__Params__Sequence__fini(my_msgs__msg__Params__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      my_msgs__msg__Params__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

my_msgs__msg__Params__Sequence *
my_msgs__msg__Params__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_msgs__msg__Params__Sequence * array = (my_msgs__msg__Params__Sequence *)allocator.allocate(sizeof(my_msgs__msg__Params__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_msgs__msg__Params__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_msgs__msg__Params__Sequence__destroy(my_msgs__msg__Params__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_msgs__msg__Params__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_msgs__msg__Params__Sequence__are_equal(const my_msgs__msg__Params__Sequence * lhs, const my_msgs__msg__Params__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_msgs__msg__Params__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_msgs__msg__Params__Sequence__copy(
  const my_msgs__msg__Params__Sequence * input,
  my_msgs__msg__Params__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_msgs__msg__Params);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_msgs__msg__Params * data =
      (my_msgs__msg__Params *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_msgs__msg__Params__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_msgs__msg__Params__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_msgs__msg__Params__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
