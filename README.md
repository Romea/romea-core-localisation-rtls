# romea_core_localisation_rtls

## Overview

`romea_core_localisation_rtls` is a framework-independent C++ library that converts RTLS ranging data into typed localisation observations.

It sits between `romea_core_rtls`, which provides ranging data structures, ranging validation and trilateration algorithms, and `romea_core_localisation`, which defines the observations fused by localisation filters. The package validates ranges, converts 3D transceiver ranges into usable 2D ranges, builds range observations, and can estimate pose or position observations from buffered ranges.

Middleware-specific nodes and message conversions are intentionally kept outside this library.

---

## Concept

The RTLS localisation plugins receive ranging results between initiator and responder transceivers. Each valid range can be converted to an `ObservationRange`; accumulated 2D ranges can also be used to estimate higher-level pose or position observations depending on the localisation problem.

| Localisation mode | Class | Produced observations |
| ----------------- | ----- | --------------------- |
| Common range processing | `RTLSPluginBase` | `ObservationRange` |
| Robot-to-world | `R2WRTLSPlugin` | `ObservationRange`, `ObservationPose` |
| Robot-to-robot | `R2RRTLSPlugin` | `ObservationRange`, leader `ObservationPose` |
| Robot-to-human | `R2HRTLSPlugin` | `ObservationRange`, leader/human `ObservationPosition` |

---

## Range processing

`process_ranging_result()` handles the common part of RTLS localisation:

1. Evaluate the ranging result with `RTLSRangingStatusEvaluator`.
2. Reject unavailable ranges, ranges outside the configured interval and ranges whose received power is below the configured threshold.
3. Create an `ObservationRange` with the raw measured range, configured range standard deviation, initiator body position and responder body position.
4. Convert the raw 3D range into a 2D range by compensating the vertical offset between the initiator and the responder.
5. Store the 2D range in the plugin-specific `TrilaterationRangeBuffer`.

When a range becomes invalid, the corresponding stored 2D range is reset.

---

## Pose and position estimation

The specialised plugins estimate higher-level observations from the stored 2D ranges:

| Class | Estimator | Output |
| ----- | --------- | ------ |
| `R2WRTLSPlugin` | `RTLSPose2DEstimator` | Robot pose in the world frame. |
| `R2RRTLSPlugin` | `RTLSPose2DEstimator` | Leader pose in the follower frame. |
| `R2HRTLSPlugin` | `RTLSPosition2DEstimator` | Human or leader position in the robot frame. |

For robot-to-world localisation, `select_responders_ranges()` can be used to keep only the selected infrastructure responders before pose estimation.

---

## Minimal usage

```cpp
#include <romea_core_common/containers/Eigen/VectorOfEigenVector.hpp>
#include <romea_core_localisation_rtls/robot_to_world_rtls_plugin.hpp>

romea::core::VectorOfEigenVector3d initiator_positions = /* robot transceiver poses */;
romea::core::VectorOfEigenVector3d responder_positions = /* anchor poses */;

romea::core::localisation::R2WRTLSPlugin plugin(
  range_std,
  minimal_range,
  maximal_range,
  rx_power_rejection_threshold,
  initiator_positions,
  responder_positions);

romea::core::localisation::ObservationRange range_observation;
if (plugin.process_ranging_result(
    initiator_index,
    responder_index,
    ranging_result,
    range_observation)) {
  // Send range_observation to a localisation updater.
}

romea::core::localisation::ObservationPose pose_observation;
if (plugin.compute_pose(pose_observation)) {
  // Send pose_observation to a localisation updater.
}
```

---

## Related packages

| Package | Role |
| ------- | ---- |
| `romea_core_rtls` | RTLS ranging status, ranging result structures, scheduling and trilateration algorithms. |
| `romea_core_localisation` | Localisation observations and filter assembly components. |

---

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

This library was developed by **Jean Laneurit** with scientific contributions from **Christophe Debain**, **Roland Chapuis** and **Romuald Aufrere**, in the context of the Baudet Rob 2 and Adap2E ANR projects.
