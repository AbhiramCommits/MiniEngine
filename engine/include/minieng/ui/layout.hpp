#pragma once

// Data-driven UI layout.
//
// A layout JSON file is parsed into a widget tree and then laid out for a given
// viewport size. Panels stack their children in a row or column with padding and
// spacing; non-stacked widgets position themselves with anchors. Nothing here
// touches ImGui, so the whole thing is testable headlessly.

#include <cstddef>
#include <string>
#include <vector>

namespace minieng {
namespace ui {

struct Rect {
    float x = 0.0f;
    float y = 0.0f;
    float w = 0.0f;
    float h = 0.0f;

    float right() const { return x + w; }
    float bottom() const { return y + h; }
    float center_x() const { return x + w * 0.5f; }
    float center_y() const { return y + h * 0.5f; }
    bool valid() const { return w > 0.0f && h > 0.0f; }
    bool contains(float px, float py) const {
        return px >= x && px <= x + w && py >= y && py <= y + h;
    }
};

enum class Anchor { TopLeft, TopRight, BottomLeft, BottomRight, Center, Stretch };
enum class Direction { None, Row, Column };

struct Widget {
    std::string id;
    std::string type;      // panel, label, button, slider, checkbox, list
    std::string text;
    std::string binding;   // BindingTable name (empty == unbound)
    float min = 0.0f;
    float max = 1.0f;

    // Authored position/size. -1 width/height means "derive it".
    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float width = -1.0f;
    float height = -1.0f;

    Anchor anchor = Anchor::TopLeft;
    Direction direction = Direction::None;
    float padding = 0.0f;
    float spacing = 0.0f;

    Rect rect;             // computed by Layout::compute_layout
    bool focusable = false;
    std::vector<Widget> children;
};

class Layout {
public:
    static Layout parse(const std::string& text, std::string* error = nullptr);
    static Layout load(const std::string& path, std::string* error = nullptr);

    const std::vector<Widget>& widgets() const { return widgets_; }
    const Widget* find(const std::string& id) const;

    float viewport_width() const { return viewport_width_; }
    float viewport_height() const { return viewport_height_; }

    // Computes every widget rect for the given viewport.
    void compute_layout(float viewport_width, float viewport_height);

    std::size_t widget_count() const;
    // Depth-first order, which is also the default Tab order.
    std::vector<const Widget*> focusable_widgets() const;

private:
    std::vector<Widget> widgets_;
    float viewport_width_ = 1280.0f;
    float viewport_height_ = 720.0f;
    float padding_ = 0.0f;
};

// Exposed for tests / the app renderer.
float default_widget_width(const Widget& widget);
float default_widget_height(const Widget& widget);

}  // namespace ui
}  // namespace minieng
