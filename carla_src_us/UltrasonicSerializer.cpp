#include "carla/sensor/s11n/UltrasonicSerializer.h"

#include "carla/sensor/data/UltrasonicMeasurement.h"

namespace carla {
namespace sensor {
namespace s11n {

  SharedPtr<SensorData> UltrasonicSerializer::Deserialize(RawData &&data) {
    return SharedPtr<data::UltrasonicMeasurement>(
        new data::UltrasonicMeasurement{std::move(data)});
  }

} // namespace s11n
} // namespace sensor
} // namespace carla