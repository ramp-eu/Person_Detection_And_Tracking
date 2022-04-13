# Person Detection And Tracking

The `Person Detection and Tracking` is based on 2D color images from cameras installed in environment and it is intended to monitor the shared spaces between humans and robots.

## Contents

- [Person Detection And Tracking](#title)
  - [Contents](#contents)
  - [Background](#background)
  - [Install](#install)
  - [Usage](#usage)
  - [Testing](#testing)
  - [License](#license)

## Background

The `Person Detection And Tracking` allows to detect people and track them using stereo pairs, obtaining the pose of each person detected in the navigation map. It requires a metric map to update dynamically the occupancy grid, information that could be used to implement local path planning to avoid dynamically occupied places, or it could be used to know the busy areas in the shopfloor for a better deployment of mobile platforms and optimization of available resources.

Image processing is based on `YOLO` (more info [here](https://arxiv.org/abs/1506.02640)), used to identify persons in every image acquired from different sources. The Regions of Interest obtained with `YOLO` from every image are paired using a combination on geometric computation and tracking using Minimum Output Sum of Squared Error (MOSSE) for disambiguation when required. The pose obtained is translated to the occupancy grid.

### Objetives

- Detect people in 2D images from environment cameras
- Extract the pose of every person using stereo pairs
- Track persons and translate it to occupancy grid

![Objetive](./docs/images/objetive.png)

## Install

Information about how to install the `Person Detection And Tracking` can be found at the corresponding section of the
[Installation & Administration Guide](docs/installationguide.md).

## Usage

Information about how to use the `Person Detection And Tracking` can be found in the [User & Programmers Manual](docs/usermanual.md).

## Testing

If there are two RGB cameras with their correspondent optics available, you can follow the following steps:

1. Calibrate the cameras with a chessboard pattern and the `OpenCV` calibration tools.
2. Launch the `Central Sensing and Perception`.
3. Launch the `Person Detection and Tracking`.

## License

[MIT](LICENSE) © <TTE>
