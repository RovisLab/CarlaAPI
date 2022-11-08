#pragma once

#include <cstdint>
#include <vector>
#include <cstdio>

namespace carla {
namespace sensor {

namespace s11n {
  class UltrasonicSerializer;
}

namespace data {

  struct UltrasonicRange {
    float range;     // m
    float angle;     //rad
  };

  class UltrasonicData {
  public:
    explicit UltrasonicData() = default;

    constexpr static auto detection_size = sizeof(UltrasonicRange);

    UltrasonicData &operator=(UltrasonicData &&) = default;

    /// Returns the number of current ranges.
    size_t GetRangesCount() const {
      return _ranges.size();
    }

    /// Deletes the current range.
    void Reset() {
      _ranges.clear();
    }

    /// Adds a new range.
    void WriteRange(UltrasonicRange range) {
      _ranges.push_back(range);
    }
    
    void ClearRange(){
        _ranges.clear();
    }

  private:
    std::vector<UltrasonicRange> _ranges;

  friend class s11n::UltrasonicSerializer;
  };

} // namespace data
} // namespace sensor
} // namespace carla
