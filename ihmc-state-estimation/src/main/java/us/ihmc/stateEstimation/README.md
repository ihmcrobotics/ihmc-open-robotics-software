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

