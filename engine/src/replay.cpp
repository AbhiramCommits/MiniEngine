#include "minieng/replay.hpp"

#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace minieng {

namespace {
constexpr const char* kHeader = "miniengine-replay 1";
}

bool Replay::write_to_file(const std::string& path) const {
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << kHeader << "\n";
    out << "seed " << seed << "\n";
    out << "frames " << frames.size() << "\n";
    // max_digits10 guarantees the float round-trips exactly, so replaying the
    // file reproduces the recorded run bit for bit.
    out << std::setprecision(std::numeric_limits<float>::max_digits10);
    for (const ReplayFrame& frame : frames) {
        out << frame.input_x << ' ' << frame.input_y << ' ' << frame.hash << "\n";
    }
    return static_cast<bool>(out);
}

Replay Replay::read_from_file(const std::string& path) {
    Replay replay;
    std::ifstream in(path);
    if (!in) {
        return replay;
    }

    std::string header;
    std::getline(in, header);
    if (header != kHeader) {
        return replay;
    }

    std::string label;
    std::uint64_t seed = 0;
    std::size_t frame_count = 0;
    in >> label >> seed;
    if (label != "seed") {
        return Replay{};
    }
    in >> label >> frame_count;
    if (label != "frames") {
        return Replay{};
    }

    replay.seed = seed;
    replay.frames.reserve(frame_count);
    for (std::size_t i = 0; i < frame_count; ++i) {
        ReplayFrame frame;
        if (!(in >> frame.input_x >> frame.input_y >> frame.hash)) {
            return Replay{};
        }
        replay.frames.push_back(frame);
    }
    return replay;
}

bool Replay::same_as(const Replay& other, bool compare_hashes) const {
    if (seed != other.seed || frames.size() != other.frames.size()) {
        return false;
    }
    for (std::size_t i = 0; i < frames.size(); ++i) {
        if (frames[i].input_x != other.frames[i].input_x ||
            frames[i].input_y != other.frames[i].input_y) {
            return false;
        }
        if (compare_hashes && frames[i].hash != other.frames[i].hash) {
            return false;
        }
    }
    return true;
}

} // namespace minieng
