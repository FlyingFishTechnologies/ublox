//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_SERIALIZATION_ROS_H
#define UBLOX_SERIALIZATION_ROS_H

#include "serialization.h"
#include "checksum.h"

#include <algorithm>
#include <vector>

#include <ros/message_traits.h>
#include <ros/serialization.h>

namespace ublox {

namespace detail {

//! Serialized size of an empty std_msgs/Header (seq, stamp, empty frame_id).
static const uint32_t kEmptyHeaderLength = 16;

template <typename T>
void read(const uint8_t *data, uint32_t count,
          typename boost::call_traits<T>::reference message,
          const ros::message_traits::FalseType&) {
  ros::serialization::IStream stream(const_cast<uint8_t *>(data), count);
  ros::serialization::Serializer<T>::read(stream, message);
}

template <typename T>
void read(const uint8_t *data, uint32_t count,
          typename boost::call_traits<T>::reference message,
          const ros::message_traits::TrueType&) {
  // ROS messages put Header first; u-blox payloads do not include it.
  std::vector<uint8_t> buf(kEmptyHeaderLength + count, 0);
  std::copy(data, data + count, buf.begin() + kEmptyHeaderLength);
  ros::serialization::IStream stream(&buf[0], buf.size());
  ros::serialization::Serializer<T>::read(stream, message);
}

template <typename T>
uint32_t serializedLength(typename boost::call_traits<T>::param_type message,
                          const ros::message_traits::FalseType&) {
  return ros::serialization::Serializer<T>::serializedLength(message);
}

template <typename T>
uint32_t serializedLength(typename boost::call_traits<T>::param_type message,
                          const ros::message_traits::TrueType&) {
  return ros::serialization::Serializer<T>::serializedLength(message) -
         ros::serialization::serializationLength(message.header);
}

template <typename T>
void write(uint8_t *data, uint32_t size,
           typename boost::call_traits<T>::param_type message,
           const ros::message_traits::FalseType&) {
  ros::serialization::OStream stream(data, size);
  ros::serialization::Serializer<T>::write(stream, message);
}

template <typename T>
void write(uint8_t *data, uint32_t size,
           typename boost::call_traits<T>::param_type message,
           const ros::message_traits::TrueType&) {
  const uint32_t header_len =
      ros::serialization::serializationLength(message.header);
  const uint32_t full_len =
      ros::serialization::Serializer<T>::serializedLength(message);
  std::vector<uint8_t> buf(full_len);
  ros::serialization::OStream stream(&buf[0], full_len);
  ros::serialization::Serializer<T>::write(stream, message);
  std::copy(buf.begin() + header_len, buf.end(), data);
  (void)size;
}

}  // namespace detail

template <typename T>
void Serializer<T>::read(const uint8_t *data, uint32_t count, 
                         typename boost::call_traits<T>::reference message) {
  detail::read<T>(data, count, message,
                  typename ros::message_traits::HasHeader<T>::type());
}

template <typename T>
uint32_t Serializer<T>::serializedLength(
    typename boost::call_traits<T>::param_type message) {
  return detail::serializedLength<T>(
      message, typename ros::message_traits::HasHeader<T>::type());
}

template <typename T>
void Serializer<T>::write(uint8_t *data, uint32_t size, 
                          typename boost::call_traits<T>::param_type message) {
  detail::write<T>(data, size, message,
                   typename ros::message_traits::HasHeader<T>::type());
}

} // namespace ublox

#endif // UBLOX_SERIALIZATION_ROS_H
