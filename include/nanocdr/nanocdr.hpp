#pragma once

#include <array>
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

constexpr Endianness getCurrentEndianness();

template <typename T>
inline void swapEndianness(T& val);

struct CdrHeader {
  Endianness endianness = Endianness::CDR_LITTLE_ENDIAN;
  EncodingAlgorithmFlag encoding = EncodingAlgorithmFlag::PLAIN_CDR2;
  CdrVersion version = CdrVersion::XCDRv2;
  uint8_t options = 0;  // Reserved for future use
};

void EncodeCdrHeader(Buffer& buffer, const CdrHeader& header);
void DecodeCdrHeader(ConstBuffer& buffer, CdrHeader& header);

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

/**
 * @brief TypeDefinition is a template class that defines the encoding and decoding of a type.
 *
 * Example usage. Given a struct:
 * @code
 * struct MyType {
 *   int a;
 *   float b;
 * };
 * @endcode
 *
 * You should define a specialization of TypeDefinition for your type:
 * @code
 * namespace nanocdr {
 * template <> struct TypeDefinition<MyType> {
 *   template <class Operator>
 *   void operator()(MyType& obj, Operator& op) {
 *     op(obj.a);
 *     op(obj.b);
 *   }
 * };
 * }
 * @endcode
 */
template <class Type>
struct TypeDefinition {
  TypeDefinition() = delete;

  template <class Operator>
  void operator()(Type& obj, Operator& op);
};

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
/**
 * @brief Decoder is a class that decodes data from a buffer.
 */
class Decoder {
 public:
  Decoder(ConstBuffer& buffer);

  const CdrHeader& header() const { return header_; }

  /**
   * @brief Decode a single value from the buffer.
   *
   * @tparam T The type of the value to decode.
   * @param out The value to decode into.
   */
  template <typename T>
  void decode(T& out);

  // specializations for std::vector
  template <typename T, typename Allocator>
  void decode(std::vector<T, Allocator>& out);

  // specializations for std::array
  template <typename T, size_t N>
  void decode(std::array<T, N>& out);

  // specializations for std::string
  void decode(std::string& out);

  // Move forwardthe pointer of the buffer
  void jump(size_t offset) { buffer_.trim_front(offset); }

  /// Get a view to the current buffer (bytes left to decode)
  ConstBuffer currentBuffer() const { return buffer_; }

 private:
  ConstBuffer buffer_;
  CdrHeader header_;
};

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

/**
 * @brief Encoder is a class that encodes data into a buffer.
 */
class Encoder {
 public:
  // Use this constructor if you want the encoder to use its own internal buffer.
  // You can copy it later using encodedBuffer()
  Encoder(CdrHeader header) : Encoder(header, default_storage_) {}

  // Use this constructor if you alredy have a buffer to encode into
  Encoder(CdrHeader header, std::vector<uint8_t>& storage);

  const CdrHeader& header() const { return header_; }

  /**
   * @brief Encode a single value into the buffer.
   *
   * @tparam T The type of the value to encode.
   * @param in The value to encode.
   */
  template <typename T>
  void encode(const T& in);

  // specializations for std::string
  void encode(const std::string& in);

  // specializations for std::vector
  template <typename T, typename Allocator>
  void encode(const std::vector<T, Allocator>& in);

  // specialization for std::array
  template <typename T, size_t N>
  void encode(const std::array<T, N>& in);

  // Get a view to the current buffer (bytes already encoded)
  ConstBuffer encodedBuffer() const { return ConstBuffer(storage_->data(), storage_->size()); }

 private:
  CdrHeader header_;
  std::vector<uint8_t> default_storage_;
  std::vector<uint8_t>* storage_ = nullptr;
};

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

template <typename T>
constexpr bool is_arithmetic() {
  return std::is_arithmetic_v<T> || std::is_same_v<T, std::byte> || std::is_same_v<T, char>;
}

template <typename T, class = void>
struct is_type_defined : std::false_type {};

template <typename T>
struct is_type_defined<T, decltype(TypeDefinition<T>(), void())> : std::true_type {};

template <typename T>
constexpr bool is_type_defined_v() {
  return is_type_defined<T>::value;
}

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
  static_assert(is_arithmetic<T>(), "swapEndianness: T must be an arithmetic type");
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

Decoder::Decoder(ConstBuffer& buffer) : buffer_(buffer) { DecodeCdrHeader(buffer_, header_); }

template <typename T, typename Allocator>
void Decoder::decode(std::vector<T, Allocator>& out) {
  uint32_t len = 0;
  decode(len);
  std::cout << "len: " << len << std::endl;
  out.resize(len);
  for (uint32_t i = 0; i < len; i++) {
    decode(out[i]);
  }
}

template <typename T, size_t N>
void Decoder::decode(std::array<T, N>& out) {
  for (uint32_t i = 0; i < out.size(); i++) {
    decode(out[i]);
  }
}

void Decoder::decode(std::string& out) {
  uint32_t len = 0;
  decode(len);
  if (buffer_.size() < len) {
    throw std::runtime_error("Decode: not enough data to decode (string)");
  }
  out.resize(len);
  memcpy(out.data(), buffer_.data(), len);
  buffer_.trim_front(len);
}

template <typename T>
void Decoder::decode(T& out) {
  if constexpr (is_arithmetic<T>()) {
    if (buffer_.size() < sizeof(T)) {
      throw std::runtime_error("Decode: not enough data to decode");
    }
    memcpy(&out, buffer_.data(), sizeof(T));

    if constexpr (sizeof(T) >= 2) {
      if (header_.endianness != getCurrentEndianness()) {
        swapEndianness(out);
      }
    }
    buffer_.trim_front(sizeof(T));
    return;
  }
  if constexpr (is_type_defined_v<T>()) {
    auto op = [this](auto& obj) { decode(obj); };
    TypeDefinition<T>().operator()(out, op);
  }
}

Encoder::Encoder(CdrHeader header, std::vector<uint8_t>& storage) : header_(header), storage_(&storage) {
  storage_->clear();
  storage_->reserve(1024);
  storage_->resize(4);
  Buffer buffer(*storage_);
  EncodeCdrHeader(buffer, header_);
}

template <typename T>
void Encoder::encode(const T& in) {
  if constexpr (is_arithmetic<T>()) {
    const auto prev_size = storage_->size();
    storage_->resize(prev_size + sizeof(T));
    if constexpr (sizeof(T) >= 2) {
      if (header_.endianness != getCurrentEndianness()) {
        T tmp = in;
        swapEndianness(tmp);
        memcpy(storage_->data() + prev_size, &tmp, sizeof(T));
        return;
      }
    }
    memcpy(storage_->data() + prev_size, &in, sizeof(T));
    return;
  }
  if constexpr (is_type_defined_v<T>()) {
    auto op = [this](auto& obj) { encode(obj); };
    TypeDefinition<T>().operator()(const_cast<T&>(in), op);
  }
}

void Encoder::encode(const std::string& in) {
  const auto prev_size = storage_->size();
  const uint32_t str_len = in.size();
  encode(str_len);
  storage_->resize(prev_size + str_len);
  memcpy(storage_->data() + prev_size, in.data(), str_len);
}

template <typename T, typename Allocator>
void Encoder::encode(const std::vector<T, Allocator>& in) {
  const uint32_t len = in.size();
  encode(len);
  for (const auto& item : in) {
    encode(item);
  }
}

template <typename T, size_t N>
void Encoder::encode(const std::array<T, N>& in) {
  for (const auto& item : in) {
    encode(item);
  }
}

}  // namespace nanocdr
