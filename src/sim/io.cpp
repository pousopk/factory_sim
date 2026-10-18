#include "sim/io.hpp"
#include <cctype>
#include <charconv>
#include <fmt/format.h>

namespace fsim {

namespace {

std::size_t widthOf(IoAddress::Size size) {
    switch (size) {
    case IoAddress::Size::Bit:
        return 1;
    case IoAddress::Size::Word:
        return 2;
    case IoAddress::Size::DoubleWord:
        return 4;
    }
    return 1;
}

std::optional<std::size_t> number(std::string_view& text) {
    std::size_t value = 0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc() || end == text.data()) {
        return std::nullopt;
    }
    text.remove_prefix(static_cast<std::size_t>(end - text.data()));
    return value;
}

} // namespace

std::optional<IoAddress> parseAddress(std::string_view text) {
    if (text.size() < 3 || text[0] != '%') {
        return std::nullopt;
    }
    IoAddress address;
    const char area = static_cast<char>(std::toupper(static_cast<unsigned char>(text[1])));
    if (area != 'I' && area != 'Q') {
        return std::nullopt;
    }
    address.area = area == 'I' ? IoAddress::Area::Input : IoAddress::Area::Output;
    text.remove_prefix(2);
    const char size = static_cast<char>(std::toupper(static_cast<unsigned char>(text[0])));
    if (size == 'X') {
        text.remove_prefix(1);
    } else if (size == 'W' || size == 'D') {
        address.size = size == 'W' ? IoAddress::Size::Word : IoAddress::Size::DoubleWord;
        text.remove_prefix(1);
    } // else a bit without the X: %I0.3

    const auto byte = number(text);
    if (!byte) {
        return std::nullopt;
    }
    address.byte = *byte;
    if (address.size == IoAddress::Size::Bit) {
        if (text.empty() || text[0] != '.') {
            return std::nullopt;
        }
        text.remove_prefix(1);
        const auto bit = number(text);
        if (!bit || *bit > 7) {
            return std::nullopt;
        }
        address.bit = static_cast<unsigned>(*bit);
    }
    if (!text.empty()) {
        return std::nullopt; // something left over: "%IX0.3x"
    }
    return address;
}

std::string formatAddress(const IoAddress& address) {
    const char area = address.area == IoAddress::Area::Input ? 'I' : 'Q';
    switch (address.size) {
    case IoAddress::Size::Bit:
        return fmt::format("%{}X{}.{}", area, address.byte, address.bit);
    case IoAddress::Size::Word:
        return fmt::format("%{}W{}", area, address.byte);
    case IoAddress::Size::DoubleWord:
        return fmt::format("%{}D{}", area, address.byte);
    }
    return {};
}

bool IoImage::fits(const IoAddress& address) {
    return address.byte + widthOf(address.size) <= kBytes;
}

bool IoImage::bit(const IoAddress& address) const {
    if (!fits(address)) {
        return false;
    }
    return ((static_cast<unsigned>(area(address.area)[address.byte]) >> address.bit) & 1U) != 0;
}

void IoImage::setBit(const IoAddress& address, bool value) {
    if (!fits(address)) {
        return;
    }
    auto& byte = area(address.area)[address.byte];
    const auto mask = static_cast<std::uint8_t>(1U << address.bit);
    byte = static_cast<std::uint8_t>(value ? byte | mask : byte & ~mask);
}

std::uint32_t IoImage::value(const IoAddress& address) const {
    if (!fits(address)) {
        return 0;
    }
    if (address.size == IoAddress::Size::Bit) {
        return bit(address) ? 1U : 0U;
    }
    // Big-endian: the first byte is the most significant.
    std::uint32_t result = 0;
    const auto& bytes = area(address.area);
    for (std::size_t i = 0; i < widthOf(address.size); ++i) {
        result = (result << 8U) | bytes[address.byte + i];
    }
    return result;
}

void IoImage::setValue(const IoAddress& address, std::uint32_t value) {
    if (!fits(address)) {
        return;
    }
    if (address.size == IoAddress::Size::Bit) {
        setBit(address, value != 0);
        return;
    }
    auto& bytes = area(address.area);
    const auto width = widthOf(address.size);
    for (std::size_t i = 0; i < width; ++i) {
        bytes[address.byte + width - 1 - i] = static_cast<std::uint8_t>(value & 0xFFU);
        value >>= 8U;
    }
}

} // namespace fsim
