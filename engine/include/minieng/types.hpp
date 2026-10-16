#pragma once

#include <cstdint>

namespace minieng {

// Entity ids are dense and start at 1 so that 0 can mean "none". Ids are never
// recycled within a World, which keeps iteration order equal to creation order
// (and therefore equal to ascending id order).
using EntityId = std::uint32_t;

constexpr EntityId kInvalidEntity = 0u;

} // namespace minieng
