#pragma once

// Deterministic replay records.
//
// A replay stores the seed plus, for every tick, the input applied that tick and
// the resulting world hash. Re-running the same seed with the same inputs must
// reproduce the same per-tick hashes; that is the deterministic fixed-seed
// replay feature. The format is a small, diff-friendly text file.

#include <cstdint>
#include <string>
#include <vector>

namespace minieng {

struct ReplayFrame {
    float input_x = 0.0f;
    float input_y = 0.0f;
    std::uint64_t hash = 0;
};

struct Replay {
    std::uint64_t seed = 0;
    std::vector<ReplayFrame> frames;

    // Returns false when the file could not be opened for writing.
    bool write_to_file(const std::string& path) const;

    // Returns an empty replay (frames.size() == 0) when the file is missing or
    // malformed.
    static Replay read_from_file(const std::string& path);

    bool same_as(const Replay& other, bool compare_hashes = true) const;
};

} // namespace minieng
