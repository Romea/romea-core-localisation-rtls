# romea_core_localisation_rtls

## Overview

`romea_core_localisation_rtls` is a C++ library that converts RTLS ranging data into localisation observations usable by `romea_core_localisation` filters.

The package is framework-independent C++ code. Middleware-specific nodes and message conversions are intentionally kept outside this library.

---

## Concept

The RTLS localisation plugin receives ranging results between initiator and responder transceivers. It validates each range, stores usable 2D ranges and can derive range, pose or position observations depending on the localisation problem.

| Localisation mode | Class | Produced observations |
| ----------------- | ----- | --------------------- |
| Common range processing | `LocalisationRTLSPlugin` | `ObservationRange` |
| Robot-to-world | `R2WLocalisationRTLSPlugin` | `ObservationRange`, `ObservationPose` |
| Robot-to-robot | `R2RLocalisationRTLSPlugin` | `ObservationRange`, leader `ObservationPose` |
| Robot-to-human | `R2HLocalisationRTLSPlugin` | `ObservationRange`, leader/human `ObservationPosition` |

---

## Range processing

`LocalisationRTLSPlugin` handles the common part of RTLS localisation:

* validation of ranging status;
* rejection of ranges outside the configured interval;
* rejection based on received power;
* conversion from raw 3D transceiver geometry to a usable 2D range;
* creation of `ObservationRange` with the configured range standard deviation.

The plugin stores ranges in a `TrilaterationDataBuffer`, which is then used by the specialised robot-to-world, robot-to-robot or robot-to-human plugins.

---

## Pose and position estimation

The specialised plugins estimate higher-level observations from the stored ranges:

| Class | Estimator | Output |
| ----- | --------- | ------ |
| `R2WLocalisationRTLSPlugin` | `RTLSPose2DEstimator` | Robot pose in the world frame. |
| `R2RLocalisationRTLSPlugin` | `RTLSPose2DEstimator` | Leader pose in the follower frame. |
| `R2HLocalisationRTLSPlugin` | `RTLSPosition2DEstimator` | Human or leader position in the robot frame. |

For robot-to-world localisation, responders can be selected with `selectRespondersRanges()` to control which infrastructure anchors are used for pose estimation.

---

## Related packages

| Package | Role |
| ------- | ---- |
| `romea_core_rtls` | Trilateration and RTLS coordination algorithms. |
| `romea_core_rtls_transceiver` | RTLS ranging result and status data structures. |
| `romea_core_localisation` | Core localisation observations and filters. |

---

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

This library was developed by **Jean Laneurit** with scientific contributions from **Christophe Debain**, **Roland Chapuis** and **Romuald Aufrere**, in the context of the Baudet Rob 2 and Adap2E ANR projects.
