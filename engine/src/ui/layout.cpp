#include "minieng/ui/layout.hpp"

#include <fstream>
#include <sstream>

#include "minieng/json.hpp"

namespace minieng {
namespace ui {

namespace {

Anchor parse_anchor(const std::string& value) {
    if (value == "top-right")
        return Anchor::TopRight;
    if (value == "bottom-left")
        return Anchor::BottomLeft;
    if (value == "bottom-right")
        return Anchor::BottomRight;
    if (value == "center")
        return Anchor::Center;
    if (value == "stretch")
        return Anchor::Stretch;
    return Anchor::TopLeft;
}

Direction parse_direction(const std::string& value) {
    if (value == "row")
        return Direction::Row;
    if (value == "column")
        return Direction::Column;
    return Direction::None;
}

bool type_is_focusable_by_default(const std::string& type) {
    return type == "button" || type == "slider" || type == "checkbox" || type == "list";
}

Widget parse_widget(const Json& node, std::string& error_out) {
    Widget widget;
    widget.id = node["id"].as_string();
    widget.type = node["type"].as_string();
    widget.text = node["text"].as_string();
    widget.binding = node["binding"].as_string();
    if (node.has("min"))
        widget.min = node["min"].as_float();
    if (node.has("max"))
        widget.max = node["max"].as_float();
    if (node.has("x"))
        widget.offset_x = node["x"].as_float();
    if (node.has("y"))
        widget.offset_y = node["y"].as_float();
    if (node.has("width"))
        widget.width = node["width"].as_float();
    if (node.has("height"))
        widget.height = node["height"].as_float();
    if (node.has("anchor"))
        widget.anchor = parse_anchor(node["anchor"].as_string());
    if (node.has("direction"))
        widget.direction = parse_direction(node["direction"].as_string());
    if (node.has("padding"))
        widget.padding = node["padding"].as_float();
    if (node.has("spacing"))
        widget.spacing = node["spacing"].as_float();
    widget.focusable = node.has("focusable") ? node["focusable"].as_bool()
                                             : type_is_focusable_by_default(widget.type);
    if (widget.id.empty()) {
        error_out = "widget is missing an id";
    }

    const Json& children = node["children"];
    if (children.is_array()) {
        for (const Json& child : children.items()) {
            widget.children.push_back(parse_widget(child, error_out));
        }
    }
    return widget;
}

inline Rect inset(const Rect& r, float padding) {
    return Rect{r.x + padding, r.y + padding, r.w - 2.0f * padding, r.h - 2.0f * padding};
}

} // namespace

float default_widget_width(const Widget& widget) {
    if (widget.type == "panel")
        return 220.0f;
    if (widget.type == "button")
        return 120.0f;
    if (widget.type == "slider")
        return 180.0f;
    if (widget.type == "checkbox")
        return 160.0f;
    if (widget.type == "list")
        return 200.0f;
    return 200.0f; // label and unknown types
}

float default_widget_height(const Widget& widget) {
    if (widget.type == "panel")
        return 200.0f;
    if (widget.type == "button")
        return 26.0f;
    if (widget.type == "slider")
        return 22.0f;
    if (widget.type == "checkbox")
        return 22.0f;
    if (widget.type == "list")
        return 120.0f;
    return 22.0f; // label and unknown types
}

Layout Layout::parse(const std::string& text, std::string* error) {
    Layout layout;
    std::string local_error;
    const Json root = Json::parse(text, &local_error);
    if (!local_error.empty()) {
        if (error)
            *error = local_error;
        return layout;
    }
    if (!root.is_object()) {
        if (error)
            *error = "layout root must be a JSON object";
        return layout;
    }

    if (root["viewport"].is_object()) {
        const Json& vp = root["viewport"];
        if (vp.has("width"))
            layout.viewport_width_ = vp["width"].as_float();
        if (vp.has("height"))
            layout.viewport_height_ = vp["height"].as_float();
    }
    if (root.has("padding")) {
        layout.padding_ = root["padding"].as_float();
    }

    const Json& widgets = root["widgets"];
    if (!widgets.is_array()) {
        if (error)
            *error = "layout must contain a 'widgets' array";
        return layout;
    }
    for (const Json& node : widgets.items()) {
        layout.widgets_.push_back(parse_widget(node, local_error));
    }
    if (!local_error.empty() && error) {
        *error = local_error;
    }
    return layout;
}

Layout Layout::load(const std::string& path, std::string* error) {
    std::ifstream file(path);
    if (!file) {
        if (error) {
            *error = "could not open file: " + path;
        }
        return Layout();
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return parse(buffer.str(), error);
}

const Widget* Layout::find(const std::string& id) const {
    // Iterative DFS over the widget forest.
    std::vector<const Widget*> stack;
    for (auto it = widgets_.rbegin(); it != widgets_.rend(); ++it) {
        stack.push_back(&*it);
    }
    while (!stack.empty()) {
        const Widget* w = stack.back();
        stack.pop_back();
        if (w->id == id) {
            return w;
        }
        for (auto it = w->children.rbegin(); it != w->children.rend(); ++it) {
            stack.push_back(&*it);
        }
    }
    return nullptr;
}

std::size_t Layout::widget_count() const {
    std::size_t count = 0;
    std::vector<const Widget*> stack;
    for (const Widget& w : widgets_) {
        stack.push_back(&w);
    }
    while (!stack.empty()) {
        const Widget* w = stack.back();
        stack.pop_back();
        ++count;
        for (const Widget& child : w->children) {
            stack.push_back(&child);
        }
    }
    return count;
}

std::vector<const Widget*> Layout::focusable_widgets() const {
    std::vector<const Widget*> out;
    std::vector<const Widget*> stack;
    for (auto it = widgets_.rbegin(); it != widgets_.rend(); ++it) {
        stack.push_back(&*it);
    }
    while (!stack.empty()) {
        const Widget* w = stack.back();
        stack.pop_back();
        if (w->focusable) {
            out.push_back(w);
        }
        for (auto it = w->children.rbegin(); it != w->children.rend(); ++it) {
            stack.push_back(&*it);
        }
    }
    return out;
}

namespace {

void layout_widget(Widget& widget, const Rect& frame, bool stacked) {
    float width = widget.width;
    float height = widget.height;

    if (stacked) {
        if (width < 0.0f) {
            width = frame.w;
        }
        if (height < 0.0f) {
            height = default_widget_height(widget);
        }
    } else {
        if (widget.anchor == Anchor::Stretch) {
            if (width < 0.0f)
                width = frame.w;
            if (height < 0.0f)
                height = frame.h;
        } else {
            if (width < 0.0f)
                width = default_widget_width(widget);
            if (height < 0.0f)
                height = default_widget_height(widget);
        }
    }

    if (stacked) {
        // The parent already decided where this widget starts.
        widget.rect = Rect{frame.x + widget.offset_x, frame.y, width, height};
    } else {
        switch (widget.anchor) {
            case Anchor::TopLeft:
                widget.rect =
                    Rect{frame.x + widget.offset_x, frame.y + widget.offset_y, width, height};
                break;
            case Anchor::TopRight:
                widget.rect = Rect{frame.x + frame.w - width - widget.offset_x,
                                   frame.y + widget.offset_y, width, height};
                break;
            case Anchor::BottomLeft:
                widget.rect = Rect{frame.x + widget.offset_x,
                                   frame.y + frame.h - height - widget.offset_y, width, height};
                break;
            case Anchor::BottomRight:
                widget.rect = Rect{frame.x + frame.w - width - widget.offset_x,
                                   frame.y + frame.h - height - widget.offset_y, width, height};
                break;
            case Anchor::Center:
                widget.rect =
                    Rect{frame.x + (frame.w - width) * 0.5f + widget.offset_x,
                         frame.y + (frame.h - height) * 0.5f + widget.offset_y, width, height};
                break;
            case Anchor::Stretch:
                widget.rect =
                    Rect{frame.x + widget.offset_x, frame.y + widget.offset_y, width, height};
                break;
        }
    }

    if (widget.children.empty()) {
        return;
    }

    const Rect content = inset(widget.rect, widget.padding);
    float cursor_x = content.x;
    float cursor_y = content.y;

    for (Widget& child : widget.children) {
        float child_width = child.width >= 0.0f ? child.width : content.w;
        float child_height = child.height >= 0.0f ? child.height : default_widget_height(child);

        Rect child_frame;
        if (widget.direction == Direction::Row) {
            child_frame = Rect{cursor_x, content.y, child_width, child_height};
            cursor_x += child_width + widget.spacing;
        } else {
            // Column is the default stacking direction for panels.
            child_frame = Rect{content.x, cursor_y, child_width, child_height};
            cursor_y += child_height + widget.spacing;
        }
        layout_widget(child, child_frame, /*stacked=*/true);
    }
}

} // namespace

void Layout::compute_layout(float viewport_width, float viewport_height) {
    viewport_width_ = viewport_width;
    viewport_height_ = viewport_height;

    const Rect root{padding_, padding_, viewport_width - 2.0f * padding_,
                    viewport_height - 2.0f * padding_};
    for (Widget& widget : widgets_) {
        layout_widget(widget, root, /*stacked=*/false);
    }
}

} // namespace ui
} // namespace minieng
