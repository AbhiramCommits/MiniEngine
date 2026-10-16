#pragma once

// Minimal, dependency-free JSON reader. Supports the subset MiniEngine needs for
// UI layout files and scene descriptions: objects, arrays, strings, numbers,
// booleans and null. Object member order is preserved.

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace minieng {

class Json {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    Json() = default;

    static Json parse(const std::string& text, std::string* error = nullptr);
    static Json parse_file(const std::string& path, std::string* error = nullptr);

    Type type() const { return type_; }
    bool is_null() const { return type_ == Type::Null; }
    bool is_bool() const { return type_ == Type::Bool; }
    bool is_number() const { return type_ == Type::Number; }
    bool is_string() const { return type_ == Type::String; }
    bool is_array() const { return type_ == Type::Array; }
    bool is_object() const { return type_ == Type::Object; }

    bool as_bool(bool fallback = false) const;
    double as_number(double fallback = 0.0) const;
    float as_float(float fallback = 0.0f) const;
    int as_int(int fallback = 0) const;
    const std::string& as_string() const;

    bool has(const std::string& key) const;
    // Missing keys and out-of-range indices yield a shared null value.
    const Json& operator[](const std::string& key) const;
    const Json& at(std::size_t index) const;

    std::size_t size() const;
    const std::vector<Json>& items() const { return array_; }
    const std::vector<std::pair<std::string, Json>>& members() const { return object_; }

private:
    Type type_ = Type::Null;
    bool bool_value_ = false;
    double number_value_ = 0.0;
    std::string string_value_;
    std::vector<Json> array_;
    std::vector<std::pair<std::string, Json>> object_;

    friend class JsonParser;
};

}  // namespace minieng
