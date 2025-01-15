/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to CXX Translator
  File name: Twist_.idl
  Source: Twist_.hpp
  Cyclone DDS: v0.10.2

*****************************************************************/
#ifndef DDSCXX_UNITREE_IDL_ROS2_TWIST__HPP
#define DDSCXX_UNITREE_IDL_ROS2_TWIST__HPP

#include "unitree/idl/ros2/Vector3_.hpp"


namespace geometry_msgs
{
namespace msg
{
namespace dds_
{
class Twist_
{
private:
 ::geometry_msgs::msg::dds_::Vector3_ linear_;
 ::geometry_msgs::msg::dds_::Vector3_ angular_;

public:
  Twist_() = default;

  explicit Twist_(
    const ::geometry_msgs::msg::dds_::Vector3_& linear,
    const ::geometry_msgs::msg::dds_::Vector3_& angular) :
    linear_(linear),
    angular_(angular) { }

  const ::geometry_msgs::msg::dds_::Vector3_& linear() const { return this->linear_; }
  ::geometry_msgs::msg::dds_::Vector3_& linear() { return this->linear_; }
  void linear(const ::geometry_msgs::msg::dds_::Vector3_& _val_) { this->linear_ = _val_; }
  void linear(::geometry_msgs::msg::dds_::Vector3_&& _val_) { this->linear_ = _val_; }
  const ::geometry_msgs::msg::dds_::Vector3_& angular() const { return this->angular_; }
  ::geometry_msgs::msg::dds_::Vector3_& angular() { return this->angular_; }
  void angular(const ::geometry_msgs::msg::dds_::Vector3_& _val_) { this->angular_ = _val_; }
  void angular(::geometry_msgs::msg::dds_::Vector3_&& _val_) { this->angular_ = _val_; }

  bool operator==(const Twist_& _other) const
  {
    (void) _other;
    return linear_ == _other.linear_ &&
      angular_ == _other.angular_;
  }

  bool operator!=(const Twist_& _other) const
  {
    return !(*this == _other);
  }

};

}

}

}

#include "dds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/datatopic.hpp"

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace topic {

template <> constexpr const char* TopicTraits<::geometry_msgs::msg::dds_::Twist_>::getTypeName()
{
  return "geometry_msgs::msg::dds_::Twist_";
}

template <> constexpr bool TopicTraits<::geometry_msgs::msg::dds_::Twist_>::isKeyless()
{
  return true;
}

#ifdef DDSCXX_HAS_TYPE_DISCOVERY
template<> constexpr unsigned int TopicTraits<::geometry_msgs::msg::dds_::Twist_>::type_map_blob_sz() { return 580; }
template<> constexpr unsigned int TopicTraits<::geometry_msgs::msg::dds_::Twist_>::type_info_blob_sz() { return 148; }
template<> inline const uint8_t * TopicTraits<::geometry_msgs::msg::dds_::Twist_>::type_map_blob() {
  static const uint8_t blob[] = {
 0xbf,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0xf1,  0x28,  0xcf,  0x21,  0x56,  0x4e,  0xbe,  0xe5, 
 0x84,  0x6f,  0x1b,  0x3c,  0xfb,  0x2b,  0x52,  0x00,  0x51,  0x00,  0x00,  0x00,  0xf1,  0x51,  0x01,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x41,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00, 
 0x19,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf1,  0x5e,  0x73,  0x97,  0xe7,  0xe8, 
 0x64,  0x40,  0xdf,  0x64,  0xaf,  0x76,  0xcd,  0x4c,  0xbc,  0x9a,  0x93,  0x2b,  0x3c,  0x00,  0x00,  0x00, 
 0x19,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf1,  0x5e,  0x73,  0x97,  0xe7,  0xe8, 
 0x64,  0x40,  0xdf,  0x64,  0xaf,  0x76,  0xcd,  0x4c,  0xbc,  0xd1,  0x8b,  0x86,  0x24,  0xf1,  0x5e,  0x73, 
 0x97,  0xe7,  0xe8,  0x64,  0x40,  0xdf,  0x64,  0xaf,  0x76,  0xcd,  0x4c,  0xbc,  0x43,  0x00,  0x00,  0x00, 
 0xf1,  0x51,  0x01,  0x00,  0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x33,  0x00,  0x00,  0x00, 
 0x03,  0x00,  0x00,  0x00,  0x0b,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0x9d, 
 0xd4,  0xe4,  0x61,  0x00,  0x0b,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0x41, 
 0x52,  0x90,  0x76,  0x00,  0x0b,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0xfb, 
 0xad,  0xe9,  0xe3,  0x00,  0x38,  0x01,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0xf2,  0x66,  0x9d,  0xb6, 
 0x2b,  0xd5,  0x53,  0x1f,  0x0b,  0xe5,  0x2c,  0x2a,  0x17,  0x6d,  0x6f,  0x00,  0x92,  0x00,  0x00,  0x00, 
 0xf2,  0x51,  0x01,  0x00,  0x29,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x21,  0x00,  0x00,  0x00, 
 0x67,  0x65,  0x6f,  0x6d,  0x65,  0x74,  0x72,  0x79,  0x5f,  0x6d,  0x73,  0x67,  0x73,  0x3a,  0x3a,  0x6d, 
 0x73,  0x67,  0x3a,  0x3a,  0x64,  0x64,  0x73,  0x5f,  0x3a,  0x3a,  0x54,  0x77,  0x69,  0x73,  0x74,  0x5f, 
 0x00,  0x00,  0x00,  0x00,  0x5a,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x25,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c, 
 0xda,  0xb3,  0x78,  0xfd,  0x32,  0x00,  0x00,  0x00,  0x07,  0x00,  0x00,  0x00,  0x6c,  0x69,  0x6e,  0x65, 
 0x61,  0x72,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x26,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c,  0xda,  0xb3,  0x78,  0xfd, 
 0x32,  0x00,  0x00,  0x00,  0x08,  0x00,  0x00,  0x00,  0x61,  0x6e,  0x67,  0x75,  0x6c,  0x61,  0x72,  0x00, 
 0x00,  0x00,  0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c,  0xda,  0xb3,  0x78,  0xfd, 
 0x32,  0x00,  0x00,  0x00,  0x78,  0x00,  0x00,  0x00,  0xf2,  0x51,  0x01,  0x00,  0x2b,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x23,  0x00,  0x00,  0x00,  0x67,  0x65,  0x6f,  0x6d,  0x65,  0x74,  0x72,  0x79, 
 0x5f,  0x6d,  0x73,  0x67,  0x73,  0x3a,  0x3a,  0x6d,  0x73,  0x67,  0x3a,  0x3a,  0x64,  0x64,  0x73,  0x5f, 
 0x3a,  0x3a,  0x56,  0x65,  0x63,  0x74,  0x6f,  0x72,  0x33,  0x5f,  0x00,  0x00,  0x40,  0x00,  0x00,  0x00, 
 0x03,  0x00,  0x00,  0x00,  0x10,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0x00, 
 0x02,  0x00,  0x00,  0x00,  0x78,  0x00,  0x00,  0x00,  0x10,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x0a,  0x00,  0x02,  0x00,  0x00,  0x00,  0x79,  0x00,  0x00,  0x00,  0x10,  0x00,  0x00,  0x00, 
 0x02,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0x00,  0x02,  0x00,  0x00,  0x00,  0x7a,  0x00,  0x00,  0x00, 
 0x40,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0xf2,  0x66,  0x9d,  0xb6,  0x2b,  0xd5,  0x53,  0x1f, 
 0x0b,  0xe5,  0x2c,  0x2a,  0x17,  0x6d,  0x6f,  0xf1,  0x28,  0xcf,  0x21,  0x56,  0x4e,  0xbe,  0xe5,  0x84, 
 0x6f,  0x1b,  0x3c,  0xfb,  0x2b,  0x52,  0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c, 
 0xda,  0xb3,  0x78,  0xfd,  0x32,  0xf1,  0x5e,  0x73,  0x97,  0xe7,  0xe8,  0x64,  0x40,  0xdf,  0x64,  0xaf, 
 0x76,  0xcd,  0x4c,  0xbc, };
  return blob;
}
template<> inline const uint8_t * TopicTraits<::geometry_msgs::msg::dds_::Twist_>::type_info_blob() {
  static const uint8_t blob[] = {
 0x90,  0x00,  0x00,  0x00,  0x01,  0x10,  0x00,  0x40,  0x40,  0x00,  0x00,  0x00,  0x3c,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf1,  0x28,  0xcf,  0x21,  0x56,  0x4e,  0xbe,  0xe5,  0x84,  0x6f,  0x1b,  0x3c, 
 0xfb,  0x2b,  0x52,  0x00,  0x55,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x1c,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00,  0xf1,  0x5e,  0x73,  0x97,  0xe7,  0xe8,  0x64,  0x40, 
 0xdf,  0x64,  0xaf,  0x76,  0xcd,  0x4c,  0xbc,  0x00,  0x47,  0x00,  0x00,  0x00,  0x02,  0x10,  0x00,  0x40, 
 0x40,  0x00,  0x00,  0x00,  0x3c,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00,  0xf2,  0x66,  0x9d,  0xb6, 
 0x2b,  0xd5,  0x53,  0x1f,  0x0b,  0xe5,  0x2c,  0x2a,  0x17,  0x6d,  0x6f,  0x00,  0x96,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x1c,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00, 
 0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c,  0xda,  0xb3,  0x78,  0xfd,  0x32,  0x00, 
 0x7c,  0x00,  0x00,  0x00, };
  return blob;
}
#endif //DDSCXX_HAS_TYPE_DISCOVERY

} //namespace topic
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

namespace dds {
namespace topic {

template <>
struct topic_type_name<::geometry_msgs::msg::dds_::Twist_>
{
    static std::string value()
    {
      return org::eclipse::cyclonedds::topic::TopicTraits<::geometry_msgs::msg::dds_::Twist_>::getTypeName();
    }
};

}
}

REGISTER_TOPIC_TYPE(::geometry_msgs::msg::dds_::Twist_)

namespace org{
namespace eclipse{
namespace cyclonedds{
namespace core{
namespace cdr{

template<>
propvec &get_type_props<::geometry_msgs::msg::dds_::Twist_>();

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool write(T& streamer, const ::geometry_msgs::msg::dds_::Twist_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.linear(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.angular(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool write(S& str, const ::geometry_msgs::msg::dds_::Twist_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::Twist_>();
  str.set_mode(cdr_stream::stream_mode::write, as_key);
  return write(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool read(T& streamer, ::geometry_msgs::msg::dds_::Twist_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.linear(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.angular(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool read(S& str, ::geometry_msgs::msg::dds_::Twist_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::Twist_>();
  str.set_mode(cdr_stream::stream_mode::read, as_key);
  return read(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool move(T& streamer, const ::geometry_msgs::msg::dds_::Twist_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.linear(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.angular(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool move(S& str, const ::geometry_msgs::msg::dds_::Twist_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::Twist_>();
  str.set_mode(cdr_stream::stream_mode::move, as_key);
  return move(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool max(T& streamer, const ::geometry_msgs::msg::dds_::Twist_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.linear(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.angular(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool max(S& str, const ::geometry_msgs::msg::dds_::Twist_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::Twist_>();
  str.set_mode(cdr_stream::stream_mode::max, as_key);
  return max(str, instance, props.data()); 
}

} //namespace cdr
} //namespace core
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

#endif // DDSCXX_UNITREE_IDL_ROS2_TWIST__HPP
