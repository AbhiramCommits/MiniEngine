#pragma once

// Frame/system profiler.
//
// Two ways to record:
//   * MINIENG_PROFILE("name") — RAII scope timer around a block.
//   * add_sample("name", ms)  — a value you measured yourself.
//
// Scoped samples are grouped per frame: begin_frame() / end_frame() flush the
// accumulated per-system totals into each series. Everything is headless (no
// window dependency), so the benchmark and CI can use it too.

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace minieng {

struct FrameStats {
    double mean_ms = 0.0;
    double p50_ms = 0.0;
    double p95_ms = 0.0;
    double max_ms = 0.0;
    std::uint64_t samples = 0;
};

// Percentile helper shared by the profiler and the benchmark.
FrameStats compute_stats(const std::vector<double>& samples_ms);

class Profiler {
public:
    static Profiler& global();

    void begin_frame();
    void end_frame();

    void begin_sample(const char* name);
    void end_sample(const char* name);
    void add_sample(const char* name, double ms);

    // Cached stats for a named series (empty stats when unknown).
    FrameStats stats(const std::string& name) const;

    // The whole-frame series recorded via add_sample("frame", ms).
    FrameStats frame_stats() const;

    std::vector<std::string> names() const;
    std::vector<std::pair<std::string, FrameStats>> all_stats() const;

    void set_max_samples(std::size_t max_samples);
    void reset();

    // Human-readable table of every series.
    std::string report() const;

private:
    struct Series {
        std::vector<double> samples;
        double frame_total = 0.0;
    };

    Series& series(const std::string& name);

    std::vector<std::pair<std::string, Series>> series_;
    std::vector<std::pair<std::string, double>> frame_start_;
    std::size_t max_samples_ = 8192;
};

// RAII scope timer used by the MINIENG_PROFILE macro.
class ProfileScope {
public:
    explicit ProfileScope(const char* name);
    ~ProfileScope();

    ProfileScope(const ProfileScope&) = delete;
    ProfileScope& operator=(const ProfileScope&) = delete;

private:
    const char* name_;
};

}  // namespace minieng

#define MINIENG_PROFILE_CONCAT_INNER(a, b) a##b
#define MINIENG_PROFILE_CONCAT(a, b) MINIENG_PROFILE_CONCAT_INNER(a, b)
#define MINIENG_PROFILE(name)                                                      \
    ::minieng::ProfileScope MINIENG_PROFILE_CONCAT(minieng_profile_scope_, __LINE__)((name))
