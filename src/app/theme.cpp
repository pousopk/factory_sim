#include "app/theme.hpp"
#include <imgui.h>

namespace fsim {

void applyTheme(float scale) {
    auto& io = ImGui::GetIO();
    ImFontConfig font;
    font.FontDataOwnedByAtlas = false; // the bytes are the program's own, not the atlas's to free
    io.Fonts->AddFontFromMemoryTTF(const_cast<unsigned char*>(kUiFont), static_cast<int>(kUiFontSize), 0.0F, &font);

    ImGui::StyleColorsDark();
    auto& style = ImGui::GetStyle();
    style.FontSizeBase = 15.0F; // points; ImGui 1.92 rasterises fonts at whatever size they're used
    style.FontScaleDpi = scale;
    // Calm and readable: rounded corners, a little more room, thin borders.
    style.WindowRounding = 5.0F;
    style.ChildRounding = 4.0F;
    style.FrameRounding = 4.0F;
    style.PopupRounding = 4.0F;
    style.GrabRounding = 4.0F;
    style.TabRounding = 4.0F;
    style.ScrollbarRounding = 8.0F;
    style.WindowPadding = ImVec2(10.0F, 8.0F);
    style.FramePadding = ImVec2(7.0F, 4.0F);
    style.ItemSpacing = ImVec2(8.0F, 6.0F);
    style.ItemInnerSpacing = ImVec2(6.0F, 4.0F);
    style.CellPadding = ImVec2(6.0F, 3.0F);
    style.WindowBorderSize = 1.0F;
    style.FrameBorderSize = 0.0F;
    style.TabBorderSize = 0.0F;
    style.WindowTitleAlign = ImVec2(0.0F, 0.5F);
    style.SeparatorTextBorderSize = 1.0F;
    style.ScaleAllSizes(scale);

    // Dark blue-grey panels, one blue accent for what's active or selected.
    const ImVec4 panel{0.115F, 0.125F, 0.145F, 1.0F};
    const ImVec4 raised{0.165F, 0.180F, 0.210F, 1.0F};
    const ImVec4 hover{0.220F, 0.250F, 0.300F, 1.0F};
    const ImVec4 accent{0.260F, 0.490F, 0.860F, 1.0F};
    const ImVec4 accentSoft{0.260F, 0.490F, 0.860F, 0.55F};
    auto* c = style.Colors;
    c[ImGuiCol_WindowBg] = panel;
    c[ImGuiCol_ChildBg] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_PopupBg] = ImVec4(0.10F, 0.11F, 0.13F, 0.98F);
    c[ImGuiCol_Border] = ImVec4(0.25F, 0.27F, 0.31F, 0.60F);
    c[ImGuiCol_MenuBarBg] = ImVec4(0.095F, 0.105F, 0.125F, 1.0F);
    c[ImGuiCol_TitleBg] = ImVec4(0.095F, 0.105F, 0.125F, 1.0F);
    c[ImGuiCol_TitleBgActive] = ImVec4(0.13F, 0.16F, 0.22F, 1.0F);
    c[ImGuiCol_TitleBgCollapsed] = c[ImGuiCol_TitleBg];
    c[ImGuiCol_FrameBg] = raised;
    c[ImGuiCol_FrameBgHovered] = hover;
    c[ImGuiCol_FrameBgActive] = ImVec4(0.26F, 0.31F, 0.40F, 1.0F);
    c[ImGuiCol_Button] = ImVec4(0.20F, 0.24F, 0.31F, 1.0F);
    c[ImGuiCol_ButtonHovered] = ImVec4(0.27F, 0.36F, 0.52F, 1.0F);
    c[ImGuiCol_ButtonActive] = accent;
    c[ImGuiCol_Header] = ImVec4(0.20F, 0.26F, 0.36F, 1.0F);
    c[ImGuiCol_HeaderHovered] = ImVec4(0.25F, 0.35F, 0.52F, 1.0F);
    c[ImGuiCol_HeaderActive] = accent;
    c[ImGuiCol_CheckMark] = ImVec4(0.45F, 0.68F, 1.0F, 1.0F);
    c[ImGuiCol_SliderGrab] = accent;
    c[ImGuiCol_SliderGrabActive] = ImVec4(0.45F, 0.68F, 1.0F, 1.0F);
    c[ImGuiCol_Tab] = ImVec4(0.14F, 0.16F, 0.19F, 1.0F);
    c[ImGuiCol_TabHovered] = ImVec4(0.27F, 0.36F, 0.52F, 1.0F);
    c[ImGuiCol_TabSelected] = ImVec4(0.20F, 0.27F, 0.39F, 1.0F);
    c[ImGuiCol_TabSelectedOverline] = accent;
    c[ImGuiCol_TabDimmed] = ImVec4(0.12F, 0.13F, 0.15F, 1.0F);
    c[ImGuiCol_TabDimmedSelected] = ImVec4(0.17F, 0.20F, 0.26F, 1.0F);
    c[ImGuiCol_DockingPreview] = accentSoft;
    c[ImGuiCol_DockingEmptyBg] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_SeparatorHovered] = accentSoft;
    c[ImGuiCol_SeparatorActive] = accent;
    c[ImGuiCol_ResizeGrip] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_ResizeGripHovered] = accentSoft;
    c[ImGuiCol_ResizeGripActive] = accent;
    c[ImGuiCol_TableHeaderBg] = ImVec4(0.15F, 0.17F, 0.20F, 1.0F);
    c[ImGuiCol_TableRowBgAlt] = ImVec4(1.0F, 1.0F, 1.0F, 0.025F);
    c[ImGuiCol_TextSelectedBg] = accentSoft;
    c[ImGuiCol_NavCursor] = accent;
}

} // namespace fsim
