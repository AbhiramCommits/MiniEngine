#include "minieng/ui/navigation.hpp"

#include <cmath>
#include <limits>

namespace minieng {
namespace ui {

void UiContext::set_items(const std::vector<FocusItem>& items) {
    items_ = items;
    // Drop focus if the focused id no longer exists.
    if (!focused_.empty()) {
        bool found = false;
        for (const FocusItem& item : items_) {
            if (item.id == focused_) {
                found = true;
                break;
            }
        }
        if (!found) {
            focused_.clear();
        }
    }
}

void UiContext::set_items_from_layout(const Layout& layout) {
    std::vector<FocusItem> items;
    for (const Widget* widget : layout.focusable_widgets()) {
        FocusItem item;
        item.id = widget->id;
        item.binding = widget->binding;
        item.rect = widget->rect;
        items.push_back(item);
    }
    set_items(items);
}

const FocusItem* UiContext::focused_item() const {
    for (const FocusItem& item : items_) {
        if (item.id == focused_) {
            return &item;
        }
    }
    return nullptr;
}

bool UiContext::focus(const std::string& id) {
    for (const FocusItem& item : items_) {
        if (item.id == id) {
            focused_ = id;
            return true;
        }
    }
    return false;
}

bool UiContext::focus_index(std::size_t index) {
    if (index >= items_.size()) {
        return false;
    }
    focused_ = items_[index].id;
    return true;
}

void UiContext::clear_focus() { focused_.clear(); }

bool UiContext::move_focus(FocusDir dir) {
    if (items_.empty()) {
        return false;
    }

    const std::string previous = focused_;

    // With nothing focused, any move lands on the first item.
    if (focused_.empty()) {
        focused_ = items_.front().id;
        return true;
    }

    if (dir == FocusDir::Next) {
        move_linear(1);
    } else if (dir == FocusDir::Prev) {
        move_linear(-1);
    } else {
        move_spatial(dir);
    }
    return focused_ != previous;
}

bool UiContext::move_linear(int step) {
    std::size_t index = 0;
    for (std::size_t i = 0; i < items_.size(); ++i) {
        if (items_[i].id == focused_) {
            index = i;
            break;
        }
    }
    const int count = static_cast<int>(items_.size());
    int next = (static_cast<int>(index) + step) % count;
    if (next < 0) {
        next += count;
    }
    focused_ = items_[static_cast<std::size_t>(next)].id;
    return true;
}

bool UiContext::move_spatial(FocusDir dir) {
    const FocusItem* current = focused_item();
    if (current == nullptr) {
        return false;
    }

    float axis_sign = 1.0f;
    bool horizontal = true;
    switch (dir) {
        case FocusDir::Left: axis_sign = -1.0f; horizontal = true; break;
        case FocusDir::Right: axis_sign = 1.0f; horizontal = true; break;
        case FocusDir::Up: axis_sign = -1.0f; horizontal = false; break;
        case FocusDir::Down: axis_sign = 1.0f; horizontal = false; break;
        default: return false;
    }

    const float cx = current->rect.center_x();
    const float cy = current->rect.center_y();

    // Pass 0 looks strictly in the requested direction and prefers the nearest
    // well-aligned item. Pass 1 wraps: it searches the opposite side and prefers
    // the farthest item (i.e. the one at the other end of the layout).
    for (int pass = 0; pass < 2; ++pass) {
        float best_score = std::numeric_limits<float>::max();
        const FocusItem* best = nullptr;

        for (const FocusItem& item : items_) {
            if (item.id == current->id) {
                continue;
            }
            const float primary = horizontal ? (item.rect.center_x() - cx)
                                             : (item.rect.center_y() - cy);
            const float cross = horizontal ? (item.rect.center_y() - cy)
                                           : (item.rect.center_x() - cx);

            float score = 0.0f;
            if (pass == 0) {
                if (primary * axis_sign <= 0.0f) {
                    continue;  // not in the requested direction
                }
                score = std::fabs(primary) + cross_axis_weight_ * std::fabs(cross);
            } else {
                if (primary * axis_sign >= 0.0f) {
                    continue;  // must be on the opposite side to wrap
                }
                // Farthest opposite item wins; alignment only breaks ties.
                score = -std::fabs(primary) + cross_axis_weight_ * std::fabs(cross) * 1.0e-3f;
            }

            if (score < best_score) {
                best_score = score;
                best = &item;
            }
        }

        if (best != nullptr) {
            focused_ = best->id;
            return true;
        }
    }
    return false;
}

bool UiContext::activate() {
    const FocusItem* item = focused_item();
    if (item == nullptr || bindings_ == nullptr || item->binding.empty()) {
        return false;
    }
    return bindings_->activate(item->binding);
}

}  // namespace ui
}  // namespace minieng
