#include <iostream>

#include "nanocdr/nanocdr.hpp"

// Custom type to test TypeDefinition

struct Position {
  float x;
  float y;
  float z;
};

struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

struct Pose {
  Position position;
  Quaternion orientation;
};

struct ComplexType {
  int32_t int32;
  double double_value;
  float float_value;
  std::string str;
  Pose pose;
  std::vector<int32_t> vec;
};

namespace nanocdr {
template <>
struct TypeDefinition<Position> {
  template <class Operator>
  void operator()(Position& obj, Operator& op) {
    op(obj.x);
    op(obj.y);
    op(obj.z);
  }
};

template <>
struct TypeDefinition<Quaternion> {
  template <class Operator>
  void operator()(Quaternion& obj, Operator& op) {
    op(obj.w);
    op(obj.x);
    op(obj.y);
    op(obj.z);
  }
};

template <>
struct TypeDefinition<Pose> {
  template <class Operator>
  void operator()(Pose& obj, Operator& op) {
    op(obj.position);
    op(obj.orientation);
  }
};

template <>
struct TypeDefinition<ComplexType> {
  template <class Operator>
  void operator()(ComplexType& obj, Operator& op) {
    op(obj.int32);
    op(obj.double_value);
    op(obj.float_value);
    op(obj.str);
    op(obj.pose);
    op(obj.vec);
  }
};
}  // namespace nanocdr

//----------------------------------------------------------------------------------------

int main() {
  using namespace nanocdr;
  std::cout << " Endianness: " << static_cast<int>(getCurrentEndianness()) << std::endl;

  Encoder encoder(CdrHeader{});

  ComplexType complex_in = {
      42, 3.14, 1.111, "Hello, World!", {{1.1f, 2.2f, 3.3f}, {0.1f, 0.2f, 0.3f, 0.4f}}, {1, 2, 3, 4, 5}};

  encoder.encode(complex_in);

  auto buffer = encoder.encodedBuffer();
  std::cout << "Encoded size in bytes: " << buffer.size() << std::endl;

  // ----- decode -----

  Decoder decoder(buffer);
  ComplexType complex_out;
  decoder.decode(complex_out);

  std::cout << "int32: " << complex_out.int32 << std::endl;
  std::cout << "double_value: " << complex_out.double_value << std::endl;
  std::cout << "float_value: " << complex_out.float_value << std::endl;
  std::cout << "str: " << complex_out.str << std::endl;
  std::cout << "pose/position: " << complex_out.pose.position.x << ", " << complex_out.pose.position.y << ", "
            << complex_out.pose.position.z << std::endl;
  std::cout << "pose/orientation: " << complex_out.pose.orientation.w << ", " << complex_out.pose.orientation.x << ", "
            << complex_out.pose.orientation.y << ", " << complex_out.pose.orientation.z << std::endl;
  std::cout << "vec: ";
  for (const auto& v : complex_out.vec) {
    std::cout << v << " ";
  }
  std::cout << std::endl;

  return 0;
}
