#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

// The simulator's I/O, the way a PLC sees it: a process image of input and
// output bytes, addressed like soft_plc and Siemens do.
//
//   %IX0.3   input bit 3 of byte 0 (a photocell)       %QX1.0   output bit 0 of byte 1 (a valve)
//   %IW4     input word at bytes 4-5                   %ID8     input double word at bytes 8-11 (an encoder count)
//
// Words and double words are big-endian, as on an S7: in %IW4, byte 4 is the
// high byte. The layout is exactly soft_plc's, so the PLC link (milestone 6)
// copies these bytes into soft_plc's process image and back, unchanged.
namespace fsim {

struct IoAddress {
    enum class Area { Input, Output };
    enum class Size { Bit, Word, DoubleWord };
    Area area = Area::Input;
    Size size = Size::Bit;
    std::size_t byte = 0;
    unsigned bit = 0; // Bit only

    bool operator==(const IoAddress&) const = default;
};

// Reads "%IX0.3", "%I0.3", "%QX1.0", "%IW4", "%ID8" (any case). Nothing if it isn't one.
std::optional<IoAddress> parseAddress(std::string_view text);
// Writes it the way parseAddress reads it: "%IX0.3", "%QX1.0", "%IW4", "%ID8".
std::string formatAddress(const IoAddress& address);

class IoImage {
public:
    static constexpr std::size_t kBytes = 16; // per area, like soft_plc's default

    bool bit(const IoAddress& address) const;
    void setBit(const IoAddress& address, bool value);
    std::uint32_t value(const IoAddress& address) const; // a bit reads as 0 or 1
    void setValue(const IoAddress& address, std::uint32_t value);

    // Whether the address fits in the image.
    static bool fits(const IoAddress& address);

    std::array<std::uint8_t, kBytes>& inputs() { return inputs_; }
    std::array<std::uint8_t, kBytes>& outputs() { return outputs_; }
    const std::array<std::uint8_t, kBytes>& inputs() const { return inputs_; }
    const std::array<std::uint8_t, kBytes>& outputs() const { return outputs_; }

private:
    std::array<std::uint8_t, kBytes>& area(IoAddress::Area a) {
        return a == IoAddress::Area::Input ? inputs_ : outputs_;
    }
    const std::array<std::uint8_t, kBytes>& area(IoAddress::Area a) const {
        return a == IoAddress::Area::Input ? inputs_ : outputs_;
    }

    std::array<std::uint8_t, kBytes> inputs_{};
    std::array<std::uint8_t, kBytes> outputs_{};
};

} // namespace fsim
