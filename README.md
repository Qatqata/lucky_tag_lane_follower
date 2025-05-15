# AprilTag Detection with Duckiebot — ROS Template

This repository is based on the [Duckietown ROS Template](https://github.com/duckietown/template-ros) and is used to detect AprilTags on a Duckiebot. The robot can interpret tag positions and publish relevant messages or transformations. Originally intended for SLAM, this project shifted focus solely to AprilTag detection and reaction due to time constraints.

---

## 🚀 Quick Start

Ensure you're in a development container and source the environment.

### Build the container

```bash
dts devel build -H lucky -f
```

### Run the publisher node

```bash
dts devel run -H lucky -L tag_publisher_launch -n publisher
```

### Run the subscriber node

```bash
dts devel run -H lucky -L tag_subscriber_launch -n subscriber
```

---

## 🧠 Functionality

The system uses the robot's camera to:

- Detect AprilTags using the `apriltag` Python library.
- Estimate each tag's 3D pose using `cv2.solvePnP`.
- Publish:
  - Tag detections on `/apriltag_detections`
  - Estimated poses on `/apriltag_poses`
  - Semantic string messages on `/apriltag_messages`
- Broadcast TF transforms between frames.

### Example tag-based messages

| Tag ID | Message |
|--------|---------|
| 2      | "lucky says to yield" |
| 6      | "lucky says to go right" |
| 7      | "lucky says to go left" |
| 20     | "lucky says to stop" |
| 74     | "lucky says to wait for the traffic light" |
| 96     | "lucky says to slow down" |

---

## 📦 Project Structure

```
.
├── launchers/
│   ├── default.sh
│   ├── tag_publisher_launch.sh
│   └── tag_subscriber_launch.sh
├── packages/
│   └── [your ROS packages here]
├── dependencies-apt.txt
├── dependencies-py3.txt
└── README.md
```

---

## 📋 Dependencies

### System packages (via APT)

List in `dependencies-apt.txt`:
- `ros-${ROS_DISTRO}-cv-bridge`
- `ros-${ROS_DISTRO}-image-transport`
- `ros-${ROS_DISTRO}-tf`

### Python packages (via pip)

List in `dependencies-py3.txt`:
- `opencv-python`
- `numpy`
- `apriltag`

---

## 🛠 Configuration Notes

- Default tag size: `0.065 m` (6.5 cm). Update this value in code if your tags are a different size.
- The camera calibration matrix and distortion coefficients are currently hardcoded — update them for better accuracy.

---

## 🌱 Future Work

- [x] AprilTag detection and pose publishing
- [ ] SLAM implementation (original goal, not yet implemented)
- [ ] Full behavior control based on detected tags

---

## 📸 Demo

*Coming soon — add a screenshot or link to a video here.*

---

## 📜 License

Licensed under the [MIT License](LICENSE).

---
