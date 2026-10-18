#pragma once
#include <cstddef>

// The look of the UI: its font (Roboto, built into the program by CMake from
// Dear ImGui's misc/fonts), colours, rounding and spacing.
namespace fsim {

extern const unsigned char kUiFont[];
extern const std::size_t kUiFontSize;

// Sets the font and style, everything scaled by `scale` (the screen's content
// scale: 2 on a HiDPI screen set to 200 %). Call once, after ImGui::CreateContext.
void applyTheme(float scale);

} // namespace fsim
