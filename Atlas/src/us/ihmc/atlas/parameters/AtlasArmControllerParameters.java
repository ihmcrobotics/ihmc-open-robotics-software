package us.ihmc.atlas.parameters;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.GainCalculator;
import us.ihmc.robotics.controllers.YoIndependentSE3PIDGains;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.wholeBodyController.DRCRobotJointMap;


public class AtlasArmControllerParameters extends ArmControllerParameters
{
   private final boolean runningOnRealRobot;
   private final DRCRobotJointMap jointMap;

   public AtlasArmControllerParameters(boolean runningOnRealRobot, DRCRobotJointMap jointMap)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      this.jointMap = jointMap;
   }

   @Override
   public YoPIDGains createJointspaceControlGains(YoVariableRegistry registry)
   {
      YoPIDGains jointspaceControlGains = new YoPIDGains("ArmJointspace", registry);

      double kp = runningOnRealRobot ? 40.0 : 80.0;
      double zeta = runningOnRealRobot ? 0.3 : 0.6;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      jointspaceControlGains.setKp(kp);
      jointspaceControlGains.setZeta(zeta);
      jointspaceControlGains.setKi(ki);
      jointspaceControlGains.setMaximumIntegralError(maxIntegralError);
      jointspaceControlGains.setMaximumFeedback(maxAccel);
      jointspaceControlGains.setMaximumFeedbackRate(maxJerk);
      jointspaceControlGains.createDerivativeGainUpdater(true);

      return jointspaceControlGains;
   }

   @Override
   public YoSE3PIDGainsInterface createTaskspaceControlGains(YoVariableRegistry registry)
   {
      YoSymmetricSE3PIDGains taskspaceControlGains = new YoSymmetricSE3PIDGains("ArmTaskspace", registry);

      double kp = runningOnRealRobot ? 40.0 :100.0;
      // When doing position control, the damping here seems to result into some kind of spring.
      double zeta = runningOnRealRobot ? 0.0 : 1.0;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      taskspaceControlGains.setProportionalGain(kp);
      taskspaceControlGains.setDampingRatio(zeta);
      taskspaceControlGains.setIntegralGain(ki);
      taskspaceControlGains.setMaximumIntegralError(maxIntegralError);
      taskspaceControlGains.setMaximumFeedback(maxAccel);
      taskspaceControlGains.setMaximumFeedbackRate(maxJerk);
      taskspaceControlGains.createDerivativeGainUpdater(true);

      return taskspaceControlGains;
   }

   @Override
   public YoSE3PIDGainsInterface createTaskspaceControlGainsForLoadBearing(YoVariableRegistry registry)
   {
      YoIndependentSE3PIDGains taskspaceControlGains = new YoIndependentSE3PIDGains("ArmLoadBearing", registry);
      taskspaceControlGains.reset();

      double kp = 100.0;
      double zeta = runningOnRealRobot ? 0.6 : 1.0;
      double kd = GainCalculator.computeDerivativeGain(kp, zeta);
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      taskspaceControlGains.setOrientationProportionalGains(0.0, 0.0, 0.0);
      taskspaceControlGains.setOrientationDerivativeGains(kd, kd, kd);
      taskspaceControlGains.setOrientationMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);
      taskspaceControlGains.setPositionMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return taskspaceControlGains;
   }

   /** {@inheritDoc} */
   @Override
   public String[] getPositionControlledJointNames(RobotSide robotSide)
   {
      if (runningOnRealRobot)
      {
         ArmJointName[] armJointNames = jointMap.getArmJointNames();
         int numberOfArmJoints = armJointNames.length;
         String[] positionControlledJointNames = new String[numberOfArmJoints];

         for (int i  = 0; i < numberOfArmJoints; i++)
            positionControlledJointNames[i] = jointMap.getArmJointName(robotSide, armJointNames[i]);

         return positionControlledJointNames;
      }
      else
      {
         return null;
      }
   }

   private Map<ArmJointName, DoubleYoVariable> jointAccelerationIntegrationAlphaPosition;

   /** {@inheritDoc} */
   @Override
   public Map<ArmJointName, DoubleYoVariable> getOrCreateAccelerationIntegrationAlphaPosition(YoVariableRegistry registry)
   {
      if (jointAccelerationIntegrationAlphaPosition != null)
         return jointAccelerationIntegrationAlphaPosition;

      DoubleYoVariable shoulder = new DoubleYoVariable("shoulderAccelerationIntegrationAlphaPosition", registry);
      DoubleYoVariable elbow = new DoubleYoVariable("elbowAccelerationIntegrationAlphaPosition", registry);
      DoubleYoVariable wrist = new DoubleYoVariable("wristAccelerationIntegrationAlphaPosition", registry);

      shoulder.set(0.9998);
      elbow.set(0.9996);
      wrist.set(0.9999);

      jointAccelerationIntegrationAlphaPosition = new HashMap<>();
      jointAccelerationIntegrationAlphaPosition.put(ArmJointName.SHOULDER_YAW, shoulder);
      jointAccelerationIntegrationAlphaPosition.put(ArmJointName.SHOULDER_ROLL, shoulder);
      jointAccelerationIntegrationAlphaPosition.put(ArmJointName.ELBOW_PITCH, elbow);
      jointAccelerationIntegrationAlphaPosition.put(ArmJointName.ELBOW_ROLL, elbow);
      jointAccelerationIntegrationAlphaPosition.put(ArmJointName.FIRST_WRIST_PITCH, wrist);
      jointAccelerationIntegrationAlphaPosition.put(ArmJointName.WRIST_ROLL, wrist);
      jointAccelerationIntegrationAlphaPosition.put(ArmJointName.SECOND_WRIST_PITCH, wrist);

      return jointAccelerationIntegrationAlphaPosition;
   }

   private Map<ArmJointName, DoubleYoVariable> jointAccelerationIntegrationAlphaVelocity;

   /** {@inheritDoc} */
   @Override
   public Map<ArmJointName, DoubleYoVariable> getOrCreateAccelerationIntegrationAlphaVelocity(YoVariableRegistry registry)
   {
      if (jointAccelerationIntegrationAlphaVelocity != null)
         return jointAccelerationIntegrationAlphaVelocity;

      DoubleYoVariable shoulder = new DoubleYoVariable("shoulderAccelerationIntegrationAlphaVelocity", registry);
      DoubleYoVariable elbow = new DoubleYoVariable("elbowAccelerationIntegrationAlphaVelocity", registry);
      DoubleYoVariable wrist = new DoubleYoVariable("wristAccelerationIntegrationAlphaVelocity", registry);

      shoulder.set(0.95);
      elbow.set(0.95);
      wrist.set(0.95);

      jointAccelerationIntegrationAlphaVelocity = new HashMap<>();
      jointAccelerationIntegrationAlphaVelocity.put(ArmJointName.SHOULDER_YAW, shoulder);
      jointAccelerationIntegrationAlphaVelocity.put(ArmJointName.SHOULDER_ROLL, shoulder);
      jointAccelerationIntegrationAlphaVelocity.put(ArmJointName.ELBOW_PITCH, elbow);
      jointAccelerationIntegrationAlphaVelocity.put(ArmJointName.ELBOW_ROLL, elbow);
      jointAccelerationIntegrationAlphaVelocity.put(ArmJointName.FIRST_WRIST_PITCH, wrist);
      jointAccelerationIntegrationAlphaVelocity.put(ArmJointName.WRIST_ROLL, wrist);
      jointAccelerationIntegrationAlphaVelocity.put(ArmJointName.SECOND_WRIST_PITCH, wrist);

      return jointAccelerationIntegrationAlphaVelocity;
   }

   private Map<ArmJointName, DoubleYoVariable> jointAccelerationIntegrationMaxPositionError;

   /** {@inheritDoc} */
   @Override
   public Map<ArmJointName, DoubleYoVariable> getOrCreateAccelerationIntegrationMaxPositionError(YoVariableRegistry registry)
   {
      if (jointAccelerationIntegrationMaxPositionError != null)
         return jointAccelerationIntegrationMaxPositionError;

      DoubleYoVariable shoulder = new DoubleYoVariable("shoulderAccelerationIntegrationMaxPositionError", registry);
      DoubleYoVariable elbow = new DoubleYoVariable("elbowAccelerationIntegrationMaxPositionError", registry);
      DoubleYoVariable wrist = new DoubleYoVariable("wristAccelerationIntegrationMaxPositionError", registry);

      shoulder.set(0.2);
      elbow.set(0.2);
      wrist.set(0.2);

      jointAccelerationIntegrationMaxPositionError = new HashMap<>();
      jointAccelerationIntegrationMaxPositionError.put(ArmJointName.SHOULDER_YAW, shoulder);
      jointAccelerationIntegrationMaxPositionError.put(ArmJointName.SHOULDER_ROLL, shoulder);
      jointAccelerationIntegrationMaxPositionError.put(ArmJointName.ELBOW_PITCH, elbow);
      jointAccelerationIntegrationMaxPositionError.put(ArmJointName.ELBOW_ROLL, elbow);
      jointAccelerationIntegrationMaxPositionError.put(ArmJointName.FIRST_WRIST_PITCH, wrist);
      jointAccelerationIntegrationMaxPositionError.put(ArmJointName.WRIST_ROLL, wrist);
      jointAccelerationIntegrationMaxPositionError.put(ArmJointName.SECOND_WRIST_PITCH, wrist);

      return jointAccelerationIntegrationMaxPositionError;
   }

   private Map<ArmJointName, DoubleYoVariable> jointAccelerationIntegrationMaxVelocity;

   /** {@inheritDoc} */
   @Override
   public Map<ArmJointName, DoubleYoVariable> getOrCreateAccelerationIntegrationMaxVelocity(YoVariableRegistry registry)
   {
      if (jointAccelerationIntegrationMaxVelocity != null)
         return jointAccelerationIntegrationMaxVelocity;

      DoubleYoVariable shoulder = new DoubleYoVariable("shoulderAccelerationIntegrationMaxVelocity", registry);
      DoubleYoVariable elbow = new DoubleYoVariable("elbowAccelerationIntegrationMaxVelocity", registry);
      DoubleYoVariable wrist = new DoubleYoVariable("wristAccelerationIntegrationMaxVelocity", registry);

      shoulder.set(2.0);
      elbow.set(2.0);
      wrist.set(2.0);

      jointAccelerationIntegrationMaxVelocity = new HashMap<>();
      jointAccelerationIntegrationMaxVelocity.put(ArmJointName.SHOULDER_YAW, shoulder);
      jointAccelerationIntegrationMaxVelocity.put(ArmJointName.SHOULDER_ROLL, shoulder);
      jointAccelerationIntegrationMaxVelocity.put(ArmJointName.ELBOW_PITCH, elbow);
      jointAccelerationIntegrationMaxVelocity.put(ArmJointName.ELBOW_ROLL, elbow);
      jointAccelerationIntegrationMaxVelocity.put(ArmJointName.FIRST_WRIST_PITCH, wrist);
      jointAccelerationIntegrationMaxVelocity.put(ArmJointName.WRIST_ROLL, wrist);
      jointAccelerationIntegrationMaxVelocity.put(ArmJointName.SECOND_WRIST_PITCH, wrist);

      return jointAccelerationIntegrationMaxVelocity;
   }

   @Override
   public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide)
   {
      Map<OneDoFJoint, Double> jointPositions = new LinkedHashMap<OneDoFJoint, Double>();

      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_YAW), robotSide.negateIfRightSide(0.785398));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(-0.1));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH), 3.00);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_ROLL), robotSide.negateIfRightSide(1.8));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.FIRST_WRIST_PITCH), -0.30);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.WRIST_ROLL), robotSide.negateIfRightSide(0.70));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SECOND_WRIST_PITCH), 0.15);

      return jointPositions;
   }
}
