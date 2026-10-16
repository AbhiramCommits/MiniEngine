#pragma once

// Deterministic xorshift64* pseudo-random generator.
//
// The core never calls std::rand or reads the clock: every source of randomness
// is an explicitly seeded Rng, which is what makes replay and cross-instance
// determinism possible.

#include <cstdint>

namespace minieng {

class Rng {
public:
    // Zero is not a valid xorshift state, so it is remapped internally.
    explicit Rng(std::uint64_t seed = 0x9E3779B97F4A7C15ull);

    void reseed(std::uint64_t seed);

    std::uint64_t next_u64();

    // Uniform in [0, 2^32).
    std::uint32_t next_u32();

    // Uniform in [0, 1).
    float next_float();

    // Uniform in [lo, hi).
    float range(float lo, float hi);

    // Uniform integer in [lo, hi].
    int range_int(int lo, int hi);

    std::uint64_t state() const { return state_; }

private:
    std::uint64_t state_;
};

} // namespace minieng
