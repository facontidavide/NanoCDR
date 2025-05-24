#include "nanocdr/nanocdr.hpp"
#include <iostream>

int main() {

    using namespace nanocdr;

    Encoder encoder(CdrHeader{});

    encoder.encode((int32_t(42)));
    encoder.encode((double(3.14)));
    encoder.encode((float(1.111)));

    std::vector<int64_t> vec_in = {1, 2, 3, 4, 5};
    encoder.encode(vec_in);

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

    return 0;   
}