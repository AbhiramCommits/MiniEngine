#include "minieng/rng.hpp"

namespace minieng {

namespace {
constexpr std::uint64_t kSeedScramble = 0x2545F4914F6CDD1Dull;
} // namespace

Rng::Rng(std::uint64_t seed) {
    reseed(seed);
}

void Rng::reseed(std::uint64_t seed) {
    state_ = seed ? seed : kSeedScramble;
    // Mix once so that nearby seeds (0, 1, 2, ...) diverge immediately.
    state_ ^= state_ >> 30;
    state_ *= 0xBF58476D1CE4E5B9ull;
    state_ ^= state_ >> 27;
    state_ *= 0x94D049BB133111EBull;
    state_ ^= state_ >> 31;
    if (state_ == 0) {
        state_ = kSeedScramble;
    }
}

std::uint64_t Rng::next_u64() {
    std::uint64_t x = state_;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    state_ = x;
    return x;
}

std::uint32_t Rng::next_u32() {
    return static_cast<std::uint32_t>(next_u64() >> 32);
}

float Rng::next_float() {
    // 24 bits of mantissa is plenty; keeps results identical across platforms.
    return static_cast<float>(next_u32() >> 8) * (1.0f / 16777216.0f);
}

float Rng::range(float lo, float hi) {
    return lo + (hi - lo) * next_float();
}

int Rng::range_int(int lo, int hi) {
    if (hi <= lo) {
        return lo;
    }
    const std::uint32_t span = static_cast<std::uint32_t>(hi - lo + 1);
    return lo + static_cast<int>(next_u32() % span);
}

} // namespace minieng
