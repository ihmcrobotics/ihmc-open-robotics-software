package us.ihmc.zulu.parameters.controller;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitchFactory;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.zulu.ZuluJointMap;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.ArrayList;
import java.util.List;

public class ZuluStateEstimatorParameters extends StateEstimatorParameters
{
   private final double estimatorDT;
   private final ZuluJointMap jointMap;

   private final double neckJointPositionFrequency;
   private final double neckJointVelocityFrequency;
   private final double spineJointPositionFrequency;
   private final double spineJointVelocityFrequency;
   private final double lowerBodyJointPositionFrequency;
   private final double lowerBodyJointVelocityFrequency;
   private final double upperArmJointPositionFrequency;
   private final double upperArmJointVelocityFrequency;
   private final double forearmJointPositionFrequency;
   private final double forearmJointVelocityFrequency;

   private final double orientationFrequency;
   private final double angularVelocityFrequency;
   private final double linearAccelerationFrequency;

   public ZuluStateEstimatorParameters(double estimatorDT, RobotTarget target, ZuluJointMap jointMap)
   {
      this.estimatorDT = estimatorDT;
      this.jointMap = jointMap;

      spineJointPositionFrequency = target == RobotTarget.REAL_ROBOT ? 1000.0 : Double.POSITIVE_INFINITY;
      spineJointVelocityFrequency = target == RobotTarget.REAL_ROBOT ? 1000.0 : Double.POSITIVE_INFINITY;

      neckJointPositionFrequency = target == RobotTarget.REAL_ROBOT ? 1000.0 : Double.POSITIVE_INFINITY;
      neckJointVelocityFrequency = target == RobotTarget.REAL_ROBOT ? 1000.0 : Double.POSITIVE_INFINITY;

      lowerBodyJointPositionFrequency = target == RobotTarget.REAL_ROBOT ? 1000.0 : Double.POSITIVE_INFINITY;
      lowerBodyJointVelocityFrequency = target == RobotTarget.REAL_ROBOT ? 1000.0 : Double.POSITIVE_INFINITY;

      upperArmJointPositionFrequency = target == RobotTarget.REAL_ROBOT ? 1000.0 : Double.POSITIVE_INFINITY;
      upperArmJointVelocityFrequency = target == RobotTarget.REAL_ROBOT ? 1000.0 : Double.POSITIVE_INFINITY;

      forearmJointPositionFrequency = target == RobotTarget.REAL_ROBOT ? 1000.0 : Double.POSITIVE_INFINITY;
      forearmJointVelocityFrequency = target == RobotTarget.REAL_ROBOT ? 1000.0 : Double.POSITIVE_INFINITY;

      orientationFrequency = target == RobotTarget.REAL_ROBOT ? 25.0 : Double.POSITIVE_INFINITY;
      angularVelocityFrequency = target == RobotTarget.REAL_ROBOT ? 25.0 : Double.POSITIVE_INFINITY;
      linearAccelerationFrequency = target == RobotTarget.REAL_ROBOT ? 25.0 : Double.POSITIVE_INFINITY;
   }

   @Override
   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
      // 2 - Low pass filters on position and velocity
      DoubleProvider lowerBodyJointPositionAlphaFilter = sensorProcessing.createAlphaFilter("lowerBodyJointPositionFrequency", lowerBodyJointPositionFrequency);
      DoubleProvider lowerBodyJointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("lowerBodyJointVelocityFrequency", lowerBodyJointVelocityFrequency);
      sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(lowerBodyJointPositionAlphaFilter, false, SensorType.JOINT_POSITION, lowerBodyJoints());
      sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(lowerBodyJointVelocityAlphaFilter, false, SensorType.JOINT_VELOCITY, lowerBodyJoints());

      DoubleProvider spineJointPositionAlphaFilter = sensorProcessing.createAlphaFilter("spineJointPositionFrequency", spineJointPositionFrequency);
      DoubleProvider spineJointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("spineJointVelocityFrequency", spineJointVelocityFrequency);
      sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(spineJointPositionAlphaFilter, false, SensorType.JOINT_POSITION, spineJoints());
      sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(spineJointVelocityAlphaFilter, false, SensorType.JOINT_VELOCITY, spineJoints());

      DoubleProvider neckJointPositionAlphaFilter = sensorProcessing.createAlphaFilter("neckJointPositionFrequency", neckJointPositionFrequency);
      DoubleProvider neckJointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("neckJointVelocityFrequency", neckJointVelocityFrequency);
      sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(neckJointPositionAlphaFilter, false, SensorType.JOINT_POSITION, neckJoints());
      sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(neckJointVelocityAlphaFilter, false, SensorType.JOINT_VELOCITY, neckJoints());

      DoubleProvider upperArmJointPositionAlphaFilter = sensorProcessing.createAlphaFilter("upperArmJointPositionFrequency", upperArmJointPositionFrequency);
      DoubleProvider upperArmJointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("upperArmJointVelocityFrequency", upperArmJointVelocityFrequency);
      sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(upperArmJointPositionAlphaFilter, false, SensorType.JOINT_POSITION, upperArmJoints());
      sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(upperArmJointVelocityAlphaFilter, false, SensorType.JOINT_VELOCITY, upperArmJoints());

      DoubleProvider forearmJointPositionAlphaFilter = sensorProcessing.createAlphaFilter("forearmJointPositionFrequency", forearmJointPositionFrequency);
      DoubleProvider forearmJointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("forearmJointVelocityFrequency", forearmJointVelocityFrequency);

      sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(forearmJointPositionAlphaFilter, false, SensorType.JOINT_POSITION, forearmJoints());
      sensorProcessing.addSensorAlphaFilterOnlyForSpecifiedSensors(forearmJointVelocityAlphaFilter, false, SensorType.JOINT_VELOCITY, forearmJoints());

      // IMU
      DoubleProvider orientationAlphaFilter = sensorProcessing.createAlphaFilter("orientationBreakFrequency", orientationFrequency);
      DoubleProvider angularVelocityAlphaFilter = sensorProcessing.createAlphaFilter("angularVelocityBreakFrequency", angularVelocityFrequency);
      DoubleProvider linearAccelerationAlphaFilter = sensorProcessing.createAlphaFilter("linearAccelerationBreakFrequency", linearAccelerationFrequency);

      sensorProcessing.addSensorAlphaFilter(orientationAlphaFilter, false, SensorType.IMU_ORIENTATION);
      sensorProcessing.addSensorAlphaFilter(angularVelocityAlphaFilter, false, SensorType.IMU_ANGULAR_VELOCITY);
      sensorProcessing.addSensorAlphaFilter(linearAccelerationAlphaFilter, false, SensorType.IMU_LINEAR_ACCELERATION);
   }

   private String[] lowerBodyJoints()
   {
      return toJointNameStrings(LegJointName.HIP_ROLL,
                                LegJointName.HIP_YAW,
                                LegJointName.HIP_PITCH,
                                LegJointName.KNEE_PITCH,
                                LegJointName.ANKLE_PITCH,
                                LegJointName.ANKLE_ROLL);
   }

   private String[] spineJoints()
   {
      return toJointNameStrings(SpineJointName.SPINE_YAW);
   }

   private String[] neckJoints()
   {
      return toJointNameStrings(NeckJointName.DISTAL_NECK_YAW, NeckJointName.DISTAL_NECK_PITCH);
   }

   private String[] upperArmJoints()
   {
      return toJointNameStrings(ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.SHOULDER_YAW, ArmJointName.ELBOW_PITCH);
   }

   private String[] forearmJoints()
   {
      return toJointNameStrings(ArmJointName.ELBOW_YAW, ArmJointName.WRIST_ROLL, ArmJointName.WRIST_YAW);
   }

   protected String[] toJointNameStrings(LegJointName... legJointNames)
   {
      List<String> names = new ArrayList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : legJointNames)
            names.add(jointMap.getLegJointName(robotSide, legJointName));
      }

      return names.toArray(new String[0]);
   }

   private String[] toJointNameStrings(ArmJointName... armJointNames)
   {
      List<String> names = new ArrayList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         for (ArmJointName armJointName : armJointNames)
            names.add(jointMap.getArmJointName(robotSide, armJointName));
      }

      return names.toArray(new String[0]);
   }

   private String[] toJointNameStrings(SpineJointName... spineJointNames)
   {
      List<String> names = new ArrayList<>();

      for (SpineJointName spineJointName : spineJointNames)
         names.add(jointMap.getSpineJointName(spineJointName));

      return names.toArray(new String[0]);
   }

   private String[] toJointNameStrings(NeckJointName... neckJointNames)
   {
      List<String> names = new ArrayList<>();

      for (NeckJointName neckJointName : neckJointNames)
         names.add(jointMap.getNeckJointName(neckJointName));

      return names.toArray(new String[0]);
   }

   @Override
   public FootSwitchFactory getFootSwitchFactory()
   {
      WrenchBasedFootSwitchFactory factory = new WrenchBasedFootSwitchFactory();
      factory.setDefaultContactThresholdForce(50.0);
      factory.setDefaultCoPThresholdDistance(4.0e-3);
      factory.setDefaultSecondContactThresholdForceIgnoringCoP(75.0);
      return factory;
   }


   @Override
   public double getEstimatorDT()
   {
      return estimatorDT;
   }
}
