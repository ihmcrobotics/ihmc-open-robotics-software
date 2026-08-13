# IHMC State Estimation

## Package Layout

```
.
├── ekf
│   ├── FootWrenchSensorUpdater.java
│   ├── HumanoidRobotEKFWithSimpleJoints.java
│   └── LeggedRobotEKF.java
├── head
│   ├── EKFHeadPoseEstimator.java
│   └── PositionSensor.java
├── humanoid
│   ├── DRCSimulatedSensorNoiseParameters.java
│   ├── kinematicsBasedStateEstimation
│   ├── StateEstimatorControllerFactory.java
│   ├── StateEstimatorController.java
│   └── StateEstimatorModeSubscriber.java
├── invariantEstimator
│   ├── ConstantContactMeasurementNoiseProvider.java
│   ├── CONTACT_DETECTION.md
│   ├── ContactMeasurementNoiseProvider.java
│   ├── ContactProbabilityProvider.java
│   ├── ContactUpdater.java
│   ├── FootReferencedYawCorrector.java
│   ├── FootSwitchContactProbabilityProvider.java
│   ├── GravityLevelingUpdater.java
│   ├── InvariantCenterOfMassUpdater.java
│   ├── InvariantContactSource.java
│   ├── InvariantEKF.java
│   ├── InvariantEKFStateEstimatorFactory.java
│   ├── InvariantEKFStateEstimator.java
│   ├── InvariantMainStateEstimator.java
│   ├── InvariantPropagator.java
│   ├── InvariantState.java
│   ├── InvariantUpdater.java
│   ├── KinematicContactDetector.java
│   ├── SEK3Utils.java
│   └── TouchdownReseedLatch.java
├── jointLevel
│   ├── AlphaComplementaryPreFilter.java
│   ├── CHANGES.md
│   ├── JointLevelKFPreFilter.java
│   ├── OneDoFJointStateSource.java
│   ├── PassThroughPreFilter.java
│   ├── ProprioceptivePreFilterFactory.java
│   ├── ProprioceptivePreFilter.java
│   ├── SwitchableJointLevelSource.java
└── └── ZeroIMUBiasProvider.java
```

## Invariant Extended Kalman Filter Framework (InEKF + JointKF)

The new contribution to this section is the new InEKF for the Alex humanoid. This is split into two places - the `invariantEstimator` package, which handles the pelvis pose along with the robot's center of mass, and the `jointLevel` package, which handles the individual joint states, along with the IMU bias estimation of those around the robot.

### Tuning the Filters

For the combination of both filters, there are parameters that allow the user to easily tune each, and to verify the quality of their performance. We go over each filter below, and then discuss how to tune them together.

#### Invariant EKF

All relevant estimator parameters at the high level are defined in `InvariantEKFStateEstimatorFactory.java`

The Invariant EKF uses the following variances for the process noise, i.e. modeling error:
- `gyroVariance` - Process noise on the gyroscope measurement model.
- `accelVariance` - Process noise on the accelerometer measurement model.
- `contactVariance`- Process noise on the contact anchor slip
- `initialCovariance` - Initial covariance scalar of the state, such that `P_0 = initialCovariance * I`, where I is the identity matrix of the state dimension.

These are the process noise variables that act on the *prediction* step of the filter, which is detailed in Section III-A of the attached derivation paper.

Similarly, the Invariant EKF uses the following variances for the measurement noise, i.e. sensor noise *and* measurement modeling error:
- `contactMeasurementVariance`
- `DEFAULT_ROLL_MEASUREMENT_VARIANCE`
- `DEFAULT_PITCH_MEASUREMENT_VARIANCE`
- `swingMeasurementInflation`
- `swingSlipInflation`


