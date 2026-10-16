#include "minieng/json.hpp"

#include <cstdlib>
#include <fstream>
#include <sstream>

namespace minieng {

namespace {

const Json kNullValue;

void append_utf8(std::string& out, unsigned int codepoint) {
    if (codepoint <= 0x7F) {
        out.push_back(static_cast<char>(codepoint));
    } else if (codepoint <= 0x7FF) {
        out.push_back(static_cast<char>(0xC0 | (codepoint >> 6)));
        out.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    } else {
        out.push_back(static_cast<char>(0xE0 | (codepoint >> 12)));
        out.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
        out.push_back(static_cast<char>(0x80 | (codepoint & 0x3F)));
    }
}

} // namespace

class JsonParser {
public:
    JsonParser(const std::string& text, std::string* error) : text_(text), error_(error) {}

    bool parse(Json& out) {
        skip_ws();
        if (!parse_value(out)) {
            return false;
        }
        skip_ws();
        if (pos_ != text_.size()) {
            return fail("trailing characters after JSON value");
        }
        return true;
    }

private:
    bool fail(const std::string& message) {
        if (error_ != nullptr && error_->empty()) {
            std::ostringstream oss;
            oss << "JSON parse error at offset " << pos_ << ": " << message;
            *error_ = oss.str();
        }
        return false;
    }

    void skip_ws() {
        while (pos_ < text_.size()) {
            const char c = text_[pos_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                ++pos_;
            } else {
                break;
            }
        }
    }

    bool consume(char c) {
        if (pos_ < text_.size() && text_[pos_] == c) {
            ++pos_;
            return true;
        }
        return false;
    }

    bool expect(char c) {
        if (!consume(c)) {
            return fail(std::string("expected '") + c + "'");
        }
        return true;
    }

    bool parse_value(Json& out) {
        if (pos_ >= text_.size()) {
            return fail("unexpected end of input");
        }
        const char c = text_[pos_];
        switch (c) {
            case '{': return parse_object(out);
            case '[': return parse_array(out);
            case '"': {
                out.type_ = Json::Type::String;
                return parse_string(out.string_value_);
            }
            case 't':
                if (text_.compare(pos_, 4, "true") == 0) {
                    pos_ += 4;
                    out.type_ = Json::Type::Bool;
                    out.bool_value_ = true;
                    return true;
                }
                return fail("invalid literal");
            case 'f':
                if (text_.compare(pos_, 5, "false") == 0) {
                    pos_ += 5;
                    out.type_ = Json::Type::Bool;
                    out.bool_value_ = false;
                    return true;
                }
                return fail("invalid literal");
            case 'n':
                if (text_.compare(pos_, 4, "null") == 0) {
                    pos_ += 4;
                    out.type_ = Json::Type::Null;
                    return true;
                }
                return fail("invalid literal");
            default: return parse_number(out);
        }
    }

    bool parse_object(Json& out) {
        out.type_ = Json::Type::Object;
        if (!expect('{')) {
            return false;
        }
        skip_ws();
        if (consume('}')) {
            return true;
        }
        while (true) {
            skip_ws();
            std::string key;
            if (!parse_string(key)) {
                return false;
            }
            skip_ws();
            if (!expect(':')) {
                return false;
            }
            skip_ws();
            Json value;
            if (!parse_value(value)) {
                return false;
            }
            out.object_.emplace_back(std::move(key), std::move(value));
            skip_ws();
            if (consume(',')) {
                continue;
            }
            if (consume('}')) {
                return true;
            }
            return fail("expected ',' or '}' in object");
        }
    }

    bool parse_array(Json& out) {
        out.type_ = Json::Type::Array;
        if (!expect('[')) {
            return false;
        }
        skip_ws();
        if (consume(']')) {
            return true;
        }
        while (true) {
            skip_ws();
            Json value;
            if (!parse_value(value)) {
                return false;
            }
            out.array_.push_back(std::move(value));
            skip_ws();
            if (consume(',')) {
                continue;
            }
            if (consume(']')) {
                return true;
            }
            return fail("expected ',' or ']' in array");
        }
    }

    bool parse_string(std::string& out) {
        if (!expect('"')) {
            return false;
        }
        out.clear();
        while (pos_ < text_.size()) {
            const char c = text_[pos_++];
            if (c == '"') {
                return true;
            }
            if (c == '\\') {
                if (pos_ >= text_.size()) {
                    return fail("unterminated escape");
                }
                const char e = text_[pos_++];
                switch (e) {
                    case '"': out.push_back('"'); break;
                    case '\\': out.push_back('\\'); break;
                    case '/': out.push_back('/'); break;
                    case 'b': out.push_back('\b'); break;
                    case 'f': out.push_back('\f'); break;
                    case 'n': out.push_back('\n'); break;
                    case 'r': out.push_back('\r'); break;
                    case 't': out.push_back('\t'); break;
                    case 'u': {
                        if (pos_ + 4 > text_.size()) {
                            return fail("truncated \\u escape");
                        }
                        unsigned int code = 0;
                        for (int i = 0; i < 4; ++i) {
                            const char h = text_[pos_++];
                            code <<= 4;
                            if (h >= '0' && h <= '9') {
                                code |= static_cast<unsigned int>(h - '0');
                            } else if (h >= 'a' && h <= 'f') {
                                code |= static_cast<unsigned int>(h - 'a' + 10);
                            } else if (h >= 'A' && h <= 'F') {
                                code |= static_cast<unsigned int>(h - 'A' + 10);
                            } else {
                                return fail("invalid \\u hex digit");
                            }
                        }
                        append_utf8(out, code);
                        break;
                    }
                    default: return fail("unknown escape");
                }
            } else {
                out.push_back(c);
            }
        }
        return fail("unterminated string");
    }

    bool parse_number(Json& out) {
        const std::size_t start = pos_;
        if (pos_ < text_.size() && (text_[pos_] == '-' || text_[pos_] == '+')) {
            ++pos_;
        }
        bool any = false;
        while (pos_ < text_.size()) {
            const char c = text_[pos_];
            if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '+' ||
                c == '-') {
                ++pos_;
                any = true;
            } else {
                break;
            }
        }
        if (!any) {
            return fail("invalid number");
        }
        const std::string token = text_.substr(start, pos_ - start);
        char* end = nullptr;
        const double value = std::strtod(token.c_str(), &end);
        if (end == token.c_str() || *end != '\0') {
            return fail("invalid number token");
        }
        out.type_ = Json::Type::Number;
        out.number_value_ = value;
        return true;
    }

    const std::string& text_;
    std::string* error_;
    std::size_t pos_ = 0;
};

Json Json::parse(const std::string& text, std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    Json root;
    JsonParser parser(text, error);
    if (!parser.parse(root)) {
        return Json();
    }
    return root;
}

Json Json::parse_file(const std::string& path, std::string* error) {
    std::ifstream file(path);
    if (!file) {
        if (error != nullptr) {
            *error = "could not open file: " + path;
        }
        return Json();
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return parse(buffer.str(), error);
}

bool Json::as_bool(bool fallback) const {
    return type_ == Type::Bool ? bool_value_ : fallback;
}

double Json::as_number(double fallback) const {
    if (type_ == Type::Number) {
        return number_value_;
    }
    if (type_ == Type::Bool) {
        return bool_value_ ? 1.0 : 0.0;
    }
    return fallback;
}

float Json::as_float(float fallback) const {
    return static_cast<float>(as_number(fallback));
}

int Json::as_int(int fallback) const {
    return static_cast<int>(as_number(fallback));
}

const std::string& Json::as_string() const {
    if (type_ == Type::String) {
        return string_value_;
    }
    return kNullValue.string_value_;
}

bool Json::has(const std::string& key) const {
    if (type_ != Type::Object) {
        return false;
    }
    for (const auto& member : object_) {
        if (member.first == key) {
            return true;
        }
    }
    return false;
}

const Json& Json::operator[](const std::string& key) const {
    if (type_ == Type::Object) {
        for (const auto& member : object_) {
            if (member.first == key) {
                return member.second;
            }
        }
    }
    return kNullValue;
}

const Json& Json::at(std::size_t index) const {
    if (type_ == Type::Array && index < array_.size()) {
        return array_[index];
    }
    return kNullValue;
}

std::size_t Json::size() const {
    if (type_ == Type::Array) {
        return array_.size();
    }
    if (type_ == Type::Object) {
        return object_.size();
    }
    return 0;
}

} // namespace minieng
