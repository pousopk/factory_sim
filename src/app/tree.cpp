// The line as a tree (each conveyor with what's on it, live), what's under
// the mouse in the 3D view (a tooltip with its name, place, wiring and live
// values), and an item's inspector.
#include "app/app.hpp"
#include "app/widgets.hpp"
#include <algorithm>
#include <cctype>
#include <fmt/format.h>
#include <imgui.h>
#include <imgui_stdlib.h>
#include <optional>

namespace fsim {

namespace {

const ImVec4 kOn{1.0F, 0.85F, 0.1F, 1.0F};

// An item of this shape, in words.
const char* oneOf(Shape shape) {
    switch (shape) {
    case Shape::Box:
        return "a box";
    case Shape::Tin:
        return "a tin";
    case Shape::Bottle:
        return "a bottle";
    case Shape::Bag:
        return "a bag of crisps";
    case Shape::Ball:
        return "a ball";
    case Shape::Bracket:
        return "an angle bracket";
    case Shape::Fish:
        return "a frozen fish";
    case Shape::Stl:
        return "an STL part";
    }
    return "an item";
}

bool contains(const std::string& text, const std::string& filter) {
    if (filter.empty()) {
        return true;
    }
    const auto lower = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
        return s;
    };
    return lower(text).find(lower(filter)) != std::string::npos;
}

// A small LED before a row: lit, dark, or (nothing to show) a gap.
void dot(std::optional<bool> on) {
    const float size = ImGui::GetFontSize() * 0.55F;
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const ImVec2 middle(at.x + size / 2.0F, at.y + ImGui::GetTextLineHeight() / 2.0F);
    if (on) {
        ImGui::GetWindowDrawList()->AddCircleFilled(middle, size / 2.0F,
                                                    *on ? ImGui::GetColorU32(kOn) : IM_COL32(70, 64, 40, 255));
    }
    ImGui::Dummy(ImVec2(size, ImGui::GetTextLineHeight()));
    ImGui::SameLine();
}

// One row under a conveyor: what it is, where, and its state.
struct Row {
    Pick pick;
    std::string name;
    std::string what; // kind and place
    std::optional<bool> on;
    std::string detail;
};

} // namespace

std::string App::describe(Pick pick) const {
    const auto& s = history_.current();
    const auto& w = *world_;
    // An address and its value now, as the PLC sees it.
    const auto io = [&](const std::string& text, const char* role) -> std::string {
        const auto address = parseAddress(text);
        if (!address || !IoImage::fits(*address)) {
            return text.empty() ? std::string() : fmt::format("\n{} {} (not an address)", role, text);
        }
        if (address->size == IoAddress::Size::Bit) {
            return fmt::format("\n{} {} = {}", role, text, w.io().bit(*address) ? 1 : 0);
        }
        const auto value = w.io().value(*address);
        return address->size == IoAddress::Size::Word
                   ? fmt::format("\n{} {} = 16#{:04X} ({})", role, text, value, static_cast<std::int16_t>(value))
                   : fmt::format("\n{} {} = {}", role, text, value);
    };
    const auto on = [&](std::size_t conveyor) {
        return conveyor < s.conveyors.size() ? s.conveyors[conveyor].spec.name : std::string("?");
    };
    switch (pick.kind) {
    case Pick::Kind::Conveyor: {
        if (pick.index >= s.conveyors.size() || pick.index >= w.conveyors().size()) {
            break;
        }
        const auto& spec = s.conveyors[pick.index].spec;
        const auto& running = w.conveyors()[pick.index];
        auto text = fmt::format("{}: conveyor, {:.2f} m, {:.2f} m/s (set {:.2f})", spec.name, spec.length,
                                running.speed, spec.targetSpeed);
        if (spec.zones > 1) {
            text += fmt::format(", {} zones", spec.zones);
        }
        if (spec.carriage != Carriage::None) {
            text += spec.carriage == Carriage::Turntable ? ", on a turntable" : ", on a lift";
        }
        text += io(spec.runOutput, "run") + io(spec.encoderInput, "encoder");
        if (spec.drive.fitted) {
            text += fmt::format("\ndrive: {}", stateName(running.drive.state())) +
                    io(spec.drive.controlOutput, "STW1") + io(spec.drive.statusInput, "ZSW1");
        }
        return text;
    }
    case Pick::Kind::Photocell:
        if (pick.index < s.photocells.size()) {
            const auto& p = s.photocells[pick.index];
            return fmt::format("{}: {} on {} at {:.2f} m", p.name, p.onlyNok ? "inspection sensor" : "photocell",
                               on(p.conveyor), p.along) +
                   io(p.input, "input");
        }
        break;
    case Pick::Kind::Proximity:
        if (pick.index < s.proximities.size() && pick.index < w.proximities().size()) {
            const auto& p = s.proximities[pick.index];
            const float distance = w.proximities()[pick.index].distance;
            return fmt::format("{}: {} switch on {} at {:.2f} m, sn {:.0f} mm{}", p.name,
                               p.inductive ? "inductive" : "capacitive", on(p.conveyor), p.along, p.range * 1000.0F,
                               distance >= 0.0F ? fmt::format(", an item {:.1f} mm away", distance * 1000.0F) : "") +
                   io(p.input, "input");
        }
        break;
    case Pick::Kind::Laser:
        if (pick.index < s.lasers.size() && pick.index < w.lasers().size()) {
            const auto& l = s.lasers[pick.index];
            const float d = w.lasers()[pick.index].distance;
            return fmt::format("{}: laser over {} at {:.2f} m, {}", l.name, on(l.conveyor), l.along,
                               d >= 0.0F ? fmt::format("{:.1f} mm to the surface", d * 1000.0F)
                                         : std::string("out of range")) +
                   io(l.output, "word");
        }
        break;
    case Pick::Kind::LightCurtain:
        if (pick.index < s.curtains.size() && pick.index < w.curtains().size()) {
            const auto& c = s.curtains[pick.index];
            return fmt::format("{}: light curtain on {} at {:.2f} m, {} of {} beams broken", c.name, on(c.conveyor),
                               c.along, w.curtains()[pick.index].blocked, c.beams) +
                   io(c.clearInput, "clear") + io(c.countInput, "count");
        }
        break;
    case Pick::Kind::Camera:
        if (pick.index < s.cameras.size() && pick.index < w.cameras().size()) {
            const auto& c = s.cameras[pick.index];
            const auto& running = w.cameras()[pick.index];
            if (c.kind == CameraKind::Line) {
                return fmt::format("{}: line-scan camera over {} at {:.2f} m, {} lines", c.name, on(c.conveyor),
                                   c.along, running.image.taken);
            }
            return fmt::format("{}: camera over {} at {:.2f} m, {}", c.name, on(c.conveyor), c.along,
                               running.timing.busyAt(w.time()) ? "busy" : "ready") +
                   io(c.triggerOutput, "trigger") + io(c.readyInput, "ready");
        }
        break;
    case Pick::Kind::Spawner:
        if (pick.index < s.spawners.size()) {
            const auto& sp = s.spawners[pick.index];
            return fmt::format("spawner on {} at {:.2f} m: {} every {:.2f} s{}", on(sp.conveyor), sp.offset,
                               sp.shape == Shape::Stl ? sp.stl.file.c_str() : oneOf(sp.shape), sp.interval,
                               sp.nokFraction > 0.0F ? fmt::format(", {:.0f} % bad", sp.nokFraction * 100.0F) : "");
        }
        break;
    case Pick::Kind::Pusher:
    case Pick::Kind::Diverter:
    case Pick::Kind::Stopper: {
        const auto describeActuator = [&](const auto& specs, const auto& running, const char* kind) -> std::string {
            if (pick.index >= specs.size() || pick.index >= running.size()) {
                return {};
            }
            const auto& a = specs[pick.index];
            const auto& stroke = running[pick.index].stroke;
            return fmt::format("{}: {} on {} at {:.2f} m, {}", a.name, kind, on(a.conveyor), a.along,
                               stroke.extended().value()    ? "out"
                               : stroke.retracted().value() ? "in"
                                                            : "moving") +
                   io(a.valveOutput, "valve") + io(a.extendedInput, "extended") + io(a.retractedInput, "retracted");
        };
        if (pick.kind == Pick::Kind::Pusher) {
            return describeActuator(s.pushers, w.pushers(), "pusher");
        }
        if (pick.kind == Pick::Kind::Diverter) {
            return describeActuator(s.diverters, w.diverters(), "diverter");
        }
        return describeActuator(s.stoppers, w.stoppers(), "stopper");
    }
    case Pick::Kind::Bin:
        if (pick.index < s.bins.size() && pick.index < w.bins().size()) {
            const auto& bin = w.bins()[pick.index];
            return fmt::format("{}: bin beside {}, {} items ({} bad)", s.bins[pick.index].name,
                               on(s.bins[pick.index].conveyor), bin.count, bin.nokCount);
        }
        break;
    case Pick::Kind::Item:
        for (const auto& item : w.items()) {
            if (item.id == pick.index) {
                return fmt::format("item {}: {} {:.0f} x {:.0f} x {:.0f} mm{}, {:.2f} m/s", item.id, oneOf(item.shape),
                                   item.size.x * 1000.0F, item.size.y * 1000.0F, item.size.z * 1000.0F,
                                   item.nok ? ", bad (NOK)" : "", glm::length(item.velocity));
            }
        }
        break;
    case Pick::Kind::None:
        break;
    }
    return {};
}

// Each conveyor with what's on it, filtered by name.
void App::drawLineTree() {
    const auto& s = scene();
    const auto& w = *world_;
    ImGui::SetNextItemWidth(-1.0F);
    ImGui::InputTextWithHint("##filter", "filter by name", &lineFilter_);
    for (std::size_t i = 0; i < s.conveyors.size(); ++i) {
        const auto& c = s.conveyors[i];
        std::vector<Row> rows;
        const auto add = [&](Pick::Kind kind, auto& specs, auto describeRow) {
            for (std::size_t k = 0; k < specs.size(); ++k) {
                if (specs[k].conveyor == i) {
                    Row row{Pick{kind, k}, {}, {}, std::nullopt, {}};
                    describeRow(specs[k], k, row);
                    rows.push_back(std::move(row));
                }
            }
        };
        add(Pick::Kind::Spawner, s.spawners, [&](const SpawnerSpec& sp, std::size_t, Row& r) {
            r.name = "spawner";
            r.what = fmt::format("{:.2f} m", sp.offset);
            r.detail = fmt::format("{}, every {:.2f} s", sp.shape == Shape::Stl ? sp.stl.file : shapeName(sp.shape),
                                   sp.interval);
        });
        add(Pick::Kind::Photocell, s.photocells, [&](const PhotocellSpec& p, std::size_t k, Row& r) {
            r.name = p.name;
            r.what = fmt::format("{} {:.2f} m", p.onlyNok ? "inspection" : "photocell", p.along);
            if (k < w.photocells().size()) {
                r.on = w.photocells()[k].output.value();
            }
            r.detail = p.input;
        });
        add(Pick::Kind::Proximity, s.proximities, [&](const ProximitySpec& p, std::size_t k, Row& r) {
            r.name = p.name;
            r.what = fmt::format("{} {:.2f} m", p.inductive ? "inductive" : "capacitive", p.along);
            if (k < w.proximities().size()) {
                r.on = w.proximities()[k].detector.output.value();
            }
            r.detail = p.input;
        });
        add(Pick::Kind::Laser, s.lasers, [&](const LaserSpec& l, std::size_t k, Row& r) {
            r.name = l.name;
            r.what = fmt::format("laser {:.2f} m", l.along);
            if (k < w.lasers().size()) {
                const float d = w.lasers()[k].distance;
                r.detail = d >= 0.0F ? fmt::format("{:.1f} mm", d * 1000.0F) : "out of range";
            }
        });
        add(Pick::Kind::LightCurtain, s.curtains, [&](const LightCurtainSpec& lc, std::size_t k, Row& r) {
            r.name = lc.name;
            r.what = fmt::format("light curtain {:.2f} m", lc.along);
            if (k < w.curtains().size()) {
                r.on = !w.curtains()[k].clear.value();
                r.detail = fmt::format("{} beams broken", w.curtains()[k].blocked);
            }
        });
        add(Pick::Kind::Camera, s.cameras, [&](const CameraSpec& cam, std::size_t k, Row& r) {
            r.name = cam.name;
            r.what = fmt::format("{} {:.2f} m", cam.kind == CameraKind::Line ? "line camera" : "camera", cam.along);
            if (k < w.cameras().size() && cam.kind == CameraKind::Area) {
                r.on = w.cameras()[k].timing.busyAt(w.time());
            }
        });
        const auto actuator = [&](Pick::Kind kind, const auto& specs, const auto& running, const char* what) {
            add(kind, specs, [&](const auto& a, std::size_t k, Row& r) {
                r.name = a.name;
                r.what = fmt::format("{} {:.2f} m", what, a.along);
                if (k < running.size()) {
                    r.on = running[k].stroke.extended().value();
                }
                r.detail = a.valveOutput;
            });
        };
        actuator(Pick::Kind::Pusher, s.pushers, w.pushers(), "pusher");
        actuator(Pick::Kind::Diverter, s.diverters, w.diverters(), "diverter");
        actuator(Pick::Kind::Stopper, s.stoppers, w.stoppers(), "stopper");
        add(Pick::Kind::Bin, s.bins, [&](const BinSpec& b, std::size_t k, Row& r) {
            r.name = b.name;
            r.what = fmt::format("bin {:.2f} m", b.along);
            if (k < w.bins().size()) {
                r.detail = fmt::format("{} items", w.bins()[k].count);
            }
        });
        // Filtered: the conveyor shows if it, or anything on it, matches.
        const bool conveyorMatches = contains(c.spec.name, lineFilter_);
        if (!lineFilter_.empty()) {
            std::erase_if(rows, [&](const Row& r) { return !contains(r.name, lineFilter_) && !conveyorMatches; });
            if (rows.empty() && !conveyorMatches) {
                continue;
            }
        }
        // The conveyor: name, length, what's special about it.
        std::string extra;
        if (c.follows && *c.follows < s.conveyors.size()) {
            extra += ", after " + s.conveyors[*c.follows].spec.name;
        } else if (c.joins && *c.joins < s.conveyors.size()) {
            extra += ", joins " + s.conveyors[*c.joins].spec.name;
        }
        if (c.spec.zones > 1) {
            extra += fmt::format(", {} zones", c.spec.zones);
        }
        if (c.spec.carriage != Carriage::None) {
            extra += c.spec.carriage == Carriage::Turntable ? ", turntable" : ", lift";
        }
        const Pick conveyorPick{Pick::Kind::Conveyor, i};
        ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick |
                                   ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_DefaultOpen;
        if (selected_ == conveyorPick) {
            flags |= ImGuiTreeNodeFlags_Selected;
        }
        if (rows.empty()) {
            flags |= ImGuiTreeNodeFlags_Leaf;
        }
        const bool running = i < w.conveyors().size() && w.conveyors()[i].speed > 0.001F;
        dot(running);
        const bool open = ImGui::TreeNodeEx(reinterpret_cast<void*>(static_cast<std::intptr_t>(i)), flags,
                                            "%s  %.2f m%s", c.spec.name.c_str(), c.spec.length, extra.c_str());
        if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
            selected_ = conveyorPick;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("%s", describe(conveyorPick).c_str());
        }
        if (!open) {
            continue;
        }
        // The kinds line up in a column just past the widest name.
        float widestName = 0.0F;
        for (const auto& row : rows) {
            widestName = std::max(widestName, ImGui::CalcTextSize(row.name.c_str()).x);
        }
        for (const auto& row : rows) {
            ImGui::PushID(static_cast<int>(row.pick.kind) * 10000 + static_cast<int>(row.pick.index));
            dot(row.on);
            const float column = ImGui::GetCursorPosX() + widestName + ImGui::GetFontSize();
            if (ImGui::Selectable(row.name.c_str(), selected_ == row.pick, ImGuiSelectableFlags_AllowOverlap)) {
                selected_ = row.pick;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("%s", describe(row.pick).c_str());
            }
            // Its kind and place, and a detail, after the name.
            ImGui::SameLine(column);
            ImGui::TextColored(kDim, "%s", row.what.c_str());
            if (!row.detail.empty()) {
                ImGui::SameLine();
                ImGui::TextColored(kDim, "  %s", row.detail.c_str());
            }
            ImGui::PopID();
        }
        ImGui::TreePop();
    }
}

void App::drawItemInspector(ItemId id) {
    const auto items = world_->items();
    const auto item = std::find_if(items.begin(), items.end(), [&](const ItemView& v) { return v.id == id; });
    if (item == items.end()) {
        ImGui::TextColored(kDim, "Gone.");
        return;
    }
    ImGui::Text("Item %llu: %s", static_cast<unsigned long long>(id), oneOf(item->shape));
    if (item->nok) {
        ImGui::SameLine();
        ImGui::TextColored(kProblem, " bad (NOK)");
    }
    ImGui::TextColored(kDim, "%.0f x %.0f x %.0f mm", item->size.x * 1000.0F, item->size.y * 1000.0F,
                       item->size.z * 1000.0F);
    if (item->geometry != nullptr) {
        const auto& g = *item->geometry;
        ImGui::TextColored(kDim, "%.3f kg of %s, %zu convex pieces", g.mass(), propertiesOf(g.material()).name,
                           std::max<std::size_t>(g.pieceCount(), 1));
    }
    ImGui::TextColored(kDim, "at (%.3f, %.3f, %.3f) m, moving %.3f m/s", item->position.x, item->position.y,
                       item->position.z, glm::length(item->velocity));
    ImGui::TextColored(kDim, "Items aren't part of the scene: the spawners make them, the run moves them.");
    if (ImGui::Button("Remove it")) {
        world_->removeItem(id);
        selected_.reset();
    }
    ImGui::SetItemTooltip("Del");
}

} // namespace fsim
