#pragma once

#include "carla/Debug.h"
#include "carla/sensor/data/Array.h"
#include "carla/sensor/s11n/UltrasonicSerializer.h"
#include "carla/sensor/data/UltrasonicData.h"

namespace carla {
namespace sensor {
namespace data {

  /// Measurement produced by a Radar. Consists of an array of RadarDetection.
  /// A RadarDetection contains 4 floats: velocity, azimuth, altitude and depth
  class UltrasonicMeasurement : public Array<data::UltrasonicRange> {
    using Super = Array<data::UltrasonicRange>;
  protected:

    using Serializer = s11n::UltrasonicSerializer;

    friend Serializer;

    explicit UltrasonicMeasurement(RawData &&data)
      : Super(0u, std::move(data)) {}

  public:

    Super::size_type GetRangeAmount() const {
      return Super::size();
    }
  };

} // namespace data
} // namespace sensor
} // namespace carla