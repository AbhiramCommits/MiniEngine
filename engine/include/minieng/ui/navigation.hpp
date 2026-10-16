#pragma once

// Headless focus navigation for the immediate-mode UI.
//
// The context owns a list of focusable items (id, binding, rect) and a focused
// id. Directional movement picks the spatially nearest item in that direction
// (arrows / d-pad); Next/Prev walk the list in order (Tab). Both wrap, so every
// item is reachable from every other item. No ImGui involvement, so this is
// fully unit-testable with synthetic input.

#include <string>
#include <vector>

#include "minieng/ui/bindings.hpp"
#include "minieng/ui/layout.hpp"

namespace minieng {
namespace ui {

enum class FocusDir { Left, Right, Up, Down, Next, Prev };

struct FocusItem {
    std::string id;
    std::string binding;
    Rect rect;
};

class UiContext {
public:
    UiContext() = default;

    void set_items(const std::vector<FocusItem>& items);
    // Convenience: builds focus items from a laid-out widget tree.
    void set_items_from_layout(const Layout& layout);

    const std::vector<FocusItem>& items() const { return items_; }
    std::size_t size() const { return items_.size(); }

    const std::string& focused_id() const { return focused_; }
    bool has_focus() const { return !focused_.empty(); }
    const FocusItem* focused_item() const;

    bool focus(const std::string& id);
    bool focus_index(std::size_t index);
    void clear_focus();

    // Moves focus in `dir`, wrapping when there is nothing further that way.
    // Returns true when the focused id changed.
    bool move_focus(FocusDir dir);

    void set_bindings(BindingTable* table) { bindings_ = table; }

    // Fires the focused item's bound action (or toggles a bool binding).
    bool activate();

    // Spatial scoring knob used by the tests: primary distance weight.
    void set_cross_axis_weight(float weight) { cross_axis_weight_ = weight; }

private:
    bool move_spatial(FocusDir dir);
    bool move_linear(int step);

    std::vector<FocusItem> items_;
    std::string focused_;
    BindingTable* bindings_ = nullptr;
    float cross_axis_weight_ = 2.0f;
};

}  // namespace ui
}  // namespace minieng
