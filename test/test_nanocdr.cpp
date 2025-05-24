#include <iostream>

#include "nanocdr/nanocdr.hpp"

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

namespace nanocdr {
template <>
void Encode(const CdrHeader& header, std::vector<uint8_t>& buffer, const Position& in) {
  Encode(header, buffer, in.x);
  Encode(header, buffer, in.y);
  Encode(header, buffer, in.z);
}
template <>
void Decode(const CdrHeader& header, ConstBuffer& buffer, Position& out) {
  Decode(header, buffer, out.x);
  Decode(header, buffer, out.y);
  Decode(header, buffer, out.z);
}

template <>
void Encode(const CdrHeader& header, std::vector<uint8_t>& buffer, const Quaternion& in) {
  Encode(header, buffer, in.w);
  Encode(header, buffer, in.x);
  Encode(header, buffer, in.y);
  Encode(header, buffer, in.z);
}
template <>
void Decode(const CdrHeader& header, ConstBuffer& buffer, Quaternion& out) {
  Decode(header, buffer, out.w);
  Decode(header, buffer, out.x);
  Decode(header, buffer, out.y);
  Decode(header, buffer, out.z);
}

template <>
void Encode(const CdrHeader& header, std::vector<uint8_t>& buffer, const Pose& in) {
  Encode(header, buffer, in.position);
  Encode(header, buffer, in.orientation);
}
template <>
void Decode(const CdrHeader& header, ConstBuffer& buffer, Pose& out) {
  Decode(header, buffer, out.position);
  Decode(header, buffer, out.orientation);
}
}  // namespace nanocdr

//----------------------------------------------------------------------------------------

int main() {
  using namespace nanocdr;
  std::cout << " Endianness: " << static_cast<int>(getCurrentEndianness()) << std::endl;

  Encoder encoder(CdrHeader{});

  encoder.encode((int32_t(42)));
  encoder.encode((double(3.14)));
  encoder.encode((float(1.111)));

  std::vector<int64_t> vec_in = {1, 2, 3, 4, 5};
  encoder.encode(vec_in);

  Pose pose_in = {{1.1f, 2.2f, 3.3f}, {0.1f, 0.2f, 0.3f, 0.4f}};
  encoder.encode(pose_in);

  std::string str = "Hello, World!";
  encoder.encode(str);

  auto buffer = encoder.encodedBuffer();

  // ----- decode -----

  Decoder decoder(buffer);

  int32_t int32;
  decoder.decode(int32);
  std::cout << "int32: " << int32 << std::endl;

  double double_value;
  decoder.decode(double_value);
  std::cout << "double_value: " << double_value << std::endl;

  float float_value;
  decoder.decode(float_value);
  std::cout << "float_value: " << float_value << std::endl;

  std::vector<int64_t> vec_out;
  decoder.decode(vec_out);
  std::cout << "vec_out: ";
  for (const auto& v : vec_out) {
    std::cout << v << " ";
  }
  std::cout << std::endl;

  Pose pose_out;
  decoder.decode(pose_out);
  std::cout << "position: " << pose_out.position.x << ", " << pose_out.position.y << ", " << pose_out.position.z
            << std::endl;
  std::cout << "orientation: " << pose_out.orientation.w << ", " << pose_out.orientation.x << ", "
            << pose_out.orientation.y << ", " << pose_out.orientation.z << std::endl;

  return 0;
}
