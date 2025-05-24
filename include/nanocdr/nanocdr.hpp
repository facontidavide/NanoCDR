#pragma once

#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace nanocdr {

template <typename T>
class Span {
 public:
  Span(T* data, size_t size) : data_(data), size_(size) {}

  // if std::is_const_v<T> is true, add the constructor from std::vector<std::remove_const_t<T>>
  template <typename U, typename Allocator, typename = std::enable_if_t<std::is_const_v<U>>>
  Span(const std::vector<U, Allocator>& vec) : data_(vec.data()), size_(vec.size()) {}

  // if std::is_const_v<T> is false, add the constructor from std::vector<T>
  template <typename U, typename Allocator, typename = std::enable_if_t<!std::is_const_v<U>>>
  Span(std::vector<U, Allocator>& vec) : data_(vec.data()), size_(vec.size()) {}

  T* data() const { return data_; }
  size_t size() const { return size_; }

  T& operator[](size_t index) const { return data_[index]; }

  T* begin() const { return data_; }
  T* end() const { return data_ + size_; }

  bool empty() const { return size_ == 0; }

  void trim_front(size_t n) {
    data_ += n;
    size_ -= n;
  }

  void trim_back(size_t n) { size_ -= n; }

 private:
  T* data_ = nullptr;
  size_t size_ = 0;
};

using Buffer = Span<uint8_t>;
using ConstBuffer = Span<const uint8_t>;

//----------------------------------------------------------------------------------------

// CDR Header related types and constants
enum class CdrVersion : uint8_t { CORBA_CDR = 0, DDS_CDR = 1, XCDRv1 = 2, XCDRv2 = 3 };

enum class EncodingAlgorithmFlag : uint8_t {
  PLAIN_CDR = 0x0,
  PL_CDR = 0x2,
  PLAIN_CDR2 = 0x6,
  DELIMIT_CDR2 = 0x8,
  PL_CDR2 = 0xa
};

enum class Endianness : uint8_t { CDR_LITTLE_ENDIAN = 0x00, CDR_BIG_ENDIAN = 0x01 };

constexpr Endianness getCurrentEndianness() {
  union {
    uint8_t u8;
    uint16_t u16 = 0x0100;
  } endian_test = {};
  endian_test.u16 = 0x0100;
  return endian_test.u8 == 0x01 ? Endianness::CDR_BIG_ENDIAN : Endianness::CDR_LITTLE_ENDIAN;
}

template <typename T>
inline void swapEndianness(T& val) {
  static_assert(std::is_arithmetic_v<T>, "swapEndianness: T must be an arithmetic type");
  if constexpr (sizeof(T) == 2) {
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&val);
    std::swap(ptr[0], ptr[1]);
  } else if constexpr (sizeof(T) == 4) {
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&val);
    std::swap(ptr[0], ptr[3]);
    std::swap(ptr[1], ptr[2]);
  } else if constexpr (sizeof(T) == 8) {
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&val);
    std::swap(ptr[0], ptr[7]);
    std::swap(ptr[1], ptr[6]);
    std::swap(ptr[2], ptr[5]);
    std::swap(ptr[3], ptr[4]);
  }
}

struct CdrHeader {
  Endianness endianness = Endianness::CDR_LITTLE_ENDIAN;
  EncodingAlgorithmFlag encoding = EncodingAlgorithmFlag::PLAIN_CDR2;
  CdrVersion version = CdrVersion::XCDRv2;
  uint8_t options = 0;  // Reserved for future use
};

inline void EncodeCdrHeader(Buffer& buffer, const CdrHeader& header) {
  if (buffer.size() < sizeof(CdrHeader)) {
    throw std::runtime_error("EncodeCdrHeader: not enough space in buffer");
  }

  uint8_t* ptr = buffer.data();
  ptr[0] = static_cast<uint8_t>(header.endianness);
  ptr[1] = static_cast<uint8_t>(header.encoding);
  ptr[2] = static_cast<uint8_t>(header.version);
  ptr[3] = header.options;

  buffer.trim_front(sizeof(CdrHeader));
}

inline void DecodeCdrHeader(ConstBuffer& buffer, CdrHeader& header) {
  if (buffer.size() < sizeof(CdrHeader)) {
    throw std::runtime_error("DecodeCdrHeader: not enough data in buffer");
  }

  const uint8_t* ptr = buffer.data();
  header.endianness = static_cast<Endianness>(ptr[0]);
  header.encoding = static_cast<EncodingAlgorithmFlag>(ptr[1]);
  header.version = static_cast<CdrVersion>(ptr[2]);
  header.options = ptr[3];

  buffer.trim_front(sizeof(CdrHeader));
}

//----------------------------------------------------------------------------------------

template <typename T>
inline void Decode(const CdrHeader& header, ConstBuffer& buffer, T& out) {
  static_assert(std::is_arithmetic_v<T>, "Default implementation for numeric values");
  if (buffer.size() < sizeof(T)) {
    throw std::runtime_error("Decode: not enough data to decode");
  }
  memcpy(&out, buffer.data(), sizeof(T));

  if constexpr (sizeof(T) >= 2) {
    if (header.endianness != getCurrentEndianness()) {
      swapEndianness(out);
    }
  }
  buffer.trim_front(sizeof(T));
}

template <typename T, typename Allocator>
inline void Decode(const CdrHeader& header, ConstBuffer& buffer, std::vector<T, Allocator>& out) {
  if (std::is_trivially_copyable_v<T> && buffer.size() < sizeof(T)) {
    throw std::runtime_error("Decode: not enough data to decode");
  }
  uint32_t len = 0;
  Decode(header, buffer, len);
  out.resize(len);
  for (uint32_t i = 0; i < len; i++) {
    Decode(header, buffer, out[i]);
  }
}

inline void Decode(const CdrHeader& header, ConstBuffer& buffer, std::string& out) {
  uint32_t len = 0;
  Decode(header, buffer, len);
  if (buffer.size() < len) {
    throw std::runtime_error("Decode: not enough data to decode (string)");
  }
  out.resize(len);
  memcpy(out.data(), buffer.data(), len);
  buffer.trim_front(len);
}

//----------------------------------------------------------------------------------------

class Decoder {
 public:
  Decoder(ConstBuffer& buffer) : buffer_(buffer) { DecodeCdrHeader(buffer_, header_); }

  const CdrHeader& header() const { return header_; }

  template <typename T, typename Allocator>
  void decode(std::vector<T, Allocator>& out) {
    Decode(header_, buffer_, out);
  }

  void decode(std::string& out) { Decode(header_, buffer_, out); }

  template <typename T>
  void decode(T& out) {
    Decode(header_, buffer_, out);
  }

  ConstBuffer currentBuffer() const { return buffer_; }

 private:
  ConstBuffer buffer_;
  CdrHeader header_;
};

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

template <typename T>
inline void Encode(const CdrHeader& header, std::vector<uint8_t>& buffer, const T& in) {
  static_assert(std::is_arithmetic_v<T>, "Default implementation for numeric values");
  const auto prev_size = buffer.size();
  buffer.resize(prev_size + sizeof(T));
  if constexpr (sizeof(T) >= 2) {
    if (header.endianness != getCurrentEndianness()) {
      T tmp = in;
      swapEndianness(tmp);
      memcpy(buffer.data() + prev_size, &tmp, sizeof(T));
      return;
    }
  }
  memcpy(buffer.data() + prev_size, &in, sizeof(T));
}

template <typename T, typename Allocator>
inline void Encode(const CdrHeader& header, std::vector<uint8_t>& buffer, const std::vector<T, Allocator>& vect) {
  const uint32_t len = vect.size();
  Encode(header, buffer, len);
  for (const auto& item : vect) {
    Encode(header, buffer, item);
  }
}

inline void Encode(const CdrHeader& header, std::vector<uint8_t>& buffer, const std::string& in) {
  const auto prev_size = buffer.size();
  const uint32_t str_len = in.size();
  Encode(header, buffer, str_len);
  buffer.resize(prev_size + str_len);
  memcpy(buffer.data() + prev_size, in.data(), str_len);
}

//----------------------------------------------------------------------------------------

template <typename T>
inline size_t Size(const T& in) {
  static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
  return sizeof(T);
}

template <typename T, typename Allocator>
inline size_t Size(const std::vector<T, Allocator>& in) {
  return sizeof(uint32_t) + in.size() * Size<T>();
}

inline size_t Size(const std::string& in) { return sizeof(uint32_t) + in.size(); }

//----------------------------------------------------------------------------------------

class Encoder {
 public:
  Encoder(CdrHeader header) : Encoder(header, default_storage_) {}

  Encoder(CdrHeader header, std::vector<uint8_t>& storage) : header_(header), storage_(&storage) {
    storage_->clear();
    storage_->reserve(1024);
    storage_->resize(4);
    Buffer buffer(*storage_);
    EncodeCdrHeader(buffer, header_);
  }

  const CdrHeader& header() const { return header_; }

  template <typename T>
  void encode(const T& in) {
    Encode(header_, *storage_, in);
  }
  template <typename T, typename Allocator>
  void encode(const std::vector<T, Allocator>& in) {
    Encode(header_, *storage_, in);
  }

  void encode(const std::string& in) { Encode(header_, *storage_, in); }

  ConstBuffer encodedBuffer() const { return ConstBuffer(storage_->data(), storage_->size()); }

 private:
  CdrHeader header_;
  std::vector<uint8_t> default_storage_;
  std::vector<uint8_t>* storage_ = nullptr;
};

}  // namespace nanocdr
