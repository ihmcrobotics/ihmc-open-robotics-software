# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.12.0] - 2019-03-05
## What's New

### ROS 1 & 2 Support

We are now by default wire compatible with ROS 2!

To use with ROS 1, download our prebuilt ROS 1 ↔ ROS 2 bridge process here: https://bintray.com/ihmcrobotics/distributions/ihmc-ros1-bridge/0.12.0#files

### Features

- New standalone application to run Atlas and Valkyrie with XBox controller.
- Controller computation time reduction.
- Controller tuneup for Valkyries.

### Dependency Upgrades

- Euclid 0.7.5 -> 0.10.0, Euclid-core 0.4.13 -> 0.7.0
- Kryo -> ROS2 / DDS
- `ihmc-graphics-description`, `ihmc-javafx-toolkit`, `ihmc-jmonkey-engine-tollkit`, `ihmc-swing-plotting`, `ihmc-geometry`, `ihmc-robot-description`, `ihmc-messager`, `simulation-construction-set` have all been extracted to standalone libraries.
- `mecano`: new standalone library for rigid-body dynamics
- `ihmc-commons`: exception handling, mutation testing, allocation free data structures, allocation testing
- `ihmc-build`: bug fixes, usability improvements, publishing flexibility, decoupled from CI plugin

## Test Results

**3,025 tests in total**
**22 test failed**
**26 tests were skipped**
**259 minutes taken in total.**

### Atlas Hardware Test

#### November 30, 2018 (201811_Stable)

This is the first tag with SM Atlas running with a lower pressure of 2300 PSI. Only one log got saved probably because of heavy logging traffic in folder `Atlas/SM Atlas/20181130_161353_AtlasControllerFactory_201811_Stable`.

- Skipped sand bed test.
- One mystery fall during flat ground walking. Unfortunately no log of that run. Robot looked good overall.
- Hand homing in the UI works only when a single hand is selected. The UI seems to get flaky with threading issues in general.
- Power supply gave out one with oscillating voltage on the output. E-stopped quickly and powercycled everything.
- Stepping in place (and walking in general) is slightly unsymmetrical.

## [0.11.0] - 2018-01-04
### Unit Test Results

https://bamboo.ihmc.us/browse/LIBS-IHMCOPENROBOTICSSOFTWAREFAST-400

3,046 tests in total
1 test failed
20 tests were skipped
311 minutes taken in total.

Failing test: VisibilityGraphsFrameworkTest.testDatasetsWithoutOcclusion

### Atlas Hardware Test

Log video: https://youtu.be/rq6zIrvAts4

*** NOTE: The user interface referred to is proprietary software. Please contact IHMC for a license.

- Robot starts correctly and arm, chest, and head move to their default configurations.
- LIDAR works in the user interface.
- REA works and planar regions show up in the user interface.
- Flat ground walking works well consistently without shaking.
   * Turn in place
   * Walk forwards
   * Walk backwards
   * Walk sideways
- Chest and head motions can be commanded from the user interface.
- Arm motions can be commanded from the user interface and arm home options in the interface work as expected.
- **Hands not working**
- Pelvis motions can be commanded from the user interfaces and the center of mass height slider works.
- Foot motions in the air can be commanded from the user interface.
   * **~1.5 Hz oscillation during single support**
- The robot can run through the final tab motions: Running Man, Karate Kid 1, and Karate Kid 2 repeatedly without falling.
   * **Falls over when changing too quickly**
- The robot can reach an object on the ground using the whole body IK from the user interface (F9).
- When the robot executes a step, the footstep reaches the position specified in the user interface without offset.
- The robot can walk over the sand test bed.
- The robot can walk over simple cinders using the planar regions module with a rough terrain footstep planner.
- The robot can walk over slanted conders when operated by a human.

### Valkyrie Hardware Test

The Valkyrie hardware platform is not supported in this release.

[Unreleased]: https://github.com/ihmcrobotics/ihmc-open-robotics-software/compare/0.12.0...HEAD
[0.12.0]: https://github.com/ihmcrobotics/ihmc-open-robotics-software/compare/0.11.0...0.12.0
[0.11.0]: https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/0.11.0