#include "minieng/profiler.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace minieng {

namespace {

double now_ms() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double, std::milli>(clock::now().time_since_epoch()).count();
}

} // namespace

FrameStats compute_stats(const std::vector<double>& samples_ms) {
    FrameStats stats;
    if (samples_ms.empty()) {
        return stats;
    }

    std::vector<double> sorted = samples_ms;
    std::sort(sorted.begin(), sorted.end());

    double sum = 0.0;
    for (const double v : sorted) {
        sum += v;
    }

    const std::size_t n = sorted.size();
    const auto percentile = [&sorted, n](double p) {
        if (n == 1) {
            return sorted[0];
        }
        const double index = p * static_cast<double>(n - 1);
        const std::size_t lo = static_cast<std::size_t>(index);
        const std::size_t hi = std::min(lo + 1, n - 1);
        const double frac = index - static_cast<double>(lo);
        return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
    };

    stats.mean_ms = sum / static_cast<double>(n);
    stats.p50_ms = percentile(0.50);
    stats.p95_ms = percentile(0.95);
    stats.max_ms = sorted.back();
    stats.samples = static_cast<std::uint64_t>(n);
    return stats;
}

Profiler& Profiler::global() {
    static Profiler instance;
    return instance;
}

Profiler::Series& Profiler::series(const std::string& name) {
    for (auto& entry : series_) {
        if (entry.first == name) {
            return entry.second;
        }
    }
    series_.emplace_back(name, Series{});
    return series_.back().second;
}

void Profiler::set_max_samples(std::size_t max_samples) {
    max_samples_ = max_samples == 0 ? 1 : max_samples;
}

void Profiler::begin_frame() {
    for (auto& entry : series_) {
        entry.second.frame_total = 0.0;
    }
    frame_start_.clear();
}

void Profiler::end_frame() {
    for (auto& entry : series_) {
        Series& s = entry.second;
        if (s.frame_total > 0.0) {
            s.samples.push_back(s.frame_total);
            if (s.samples.size() > max_samples_) {
                s.samples.erase(s.samples.begin(),
                                s.samples.begin() +
                                    static_cast<std::ptrdiff_t>(s.samples.size() - max_samples_));
            }
        }
    }
}

void Profiler::begin_sample(const char* name) {
    if (name == nullptr) {
        return;
    }
    frame_start_.emplace_back(name, now_ms());
}

void Profiler::end_sample(const char* name) {
    if (name == nullptr) {
        return;
    }
    const double end = now_ms();
    // Pop the most recent matching start so nested scopes of different names work.
    for (auto it = frame_start_.end(); it != frame_start_.begin();) {
        --it;
        if (it->first == name) {
            const double elapsed = end - it->second;
            series(name).frame_total += elapsed;
            frame_start_.erase(it);
            return;
        }
    }
}

void Profiler::add_sample(const char* name, double ms) {
    if (name == nullptr) {
        return;
    }
    Series& s = series(name);
    s.samples.push_back(ms);
    if (s.samples.size() > max_samples_) {
        s.samples.erase(
            s.samples.begin(),
            s.samples.begin() + static_cast<std::ptrdiff_t>(s.samples.size() - max_samples_));
    }
}

FrameStats Profiler::stats(const std::string& name) const {
    for (const auto& entry : series_) {
        if (entry.first == name) {
            return compute_stats(entry.second.samples);
        }
    }
    return FrameStats{};
}

FrameStats Profiler::frame_stats() const {
    return stats("frame");
}

std::vector<std::string> Profiler::names() const {
    std::vector<std::string> out;
    out.reserve(series_.size());
    for (const auto& entry : series_) {
        out.push_back(entry.first);
    }
    return out;
}

std::vector<std::pair<std::string, FrameStats>> Profiler::all_stats() const {
    std::vector<std::pair<std::string, FrameStats>> out;
    out.reserve(series_.size());
    for (const auto& entry : series_) {
        out.emplace_back(entry.first, compute_stats(entry.second.samples));
    }
    return out;
}

void Profiler::reset() {
    for (auto& entry : series_) {
        entry.second.samples.clear();
        entry.second.frame_total = 0.0;
    }
    frame_start_.clear();
}

std::string Profiler::report() const {
    std::ostringstream oss;
    oss << "system            mean_ms    p50_ms    p95_ms    max_ms   samples\n";
    for (const auto& entry : series_) {
        const FrameStats s = compute_stats(entry.second.samples);
        char line[160];
        std::snprintf(line, sizeof(line), "%-16s %8.3f  %8.3f  %8.3f  %8.3f  %8llu\n",
                      entry.first.c_str(), s.mean_ms, s.p50_ms, s.p95_ms, s.max_ms,
                      static_cast<unsigned long long>(s.samples));
        oss << line;
    }
    return oss.str();
}

ProfileScope::ProfileScope(const char* name) : name_(name) {
    Profiler::global().begin_sample(name_);
}

ProfileScope::~ProfileScope() {
    Profiler::global().end_sample(name_);
}

} // namespace minieng
