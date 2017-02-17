package us.ihmc.valkyrie.parameters;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
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

public class ValkyrieArmControllerParameters extends ArmControllerParameters
{
   private final boolean runningOnRealRobot;
   private final DRCRobotJointMap jointMap;

   public ValkyrieArmControllerParameters(DRCRobotJointMap jointMap, DRCRobotModel.RobotTarget target)
   {
      this.runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT || target == DRCRobotModel.RobotTarget.GAZEBO;
      this.jointMap = jointMap;
   }

   @Override
   public YoPIDGains createJointspaceControlGains(YoVariableRegistry registry)
   {
      YoPIDGains jointspaceControlGains = new YoPIDGains("ArmJointspace", registry);

      double kp = runningOnRealRobot ? 200.0 : 120.0; // 200.0
      double zeta = runningOnRealRobot ? 1.0 : 0.7;
      double ki = runningOnRealRobot ? 0.0 : 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 50.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 750.0 : Double.POSITIVE_INFINITY;

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

      double kp = 100.0;
      double zeta = runningOnRealRobot ? 0.6 : 1.0;
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
      return taskspaceControlGains;
   }

   @Override
   public String[] getPositionControlledJointNames(RobotSide robotSide)
   {
      if (runningOnRealRobot)
      {
         String[] positionControlledJointNames = new String[3];

         int i = 0;
         positionControlledJointNames[i++] = jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL);
         positionControlledJointNames[i++] = jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH);
         positionControlledJointNames[i++] = jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL);

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

      DoubleYoVariable elbow = new DoubleYoVariable("elbowAccelerationIntegrationAlphaPosition", registry);
      DoubleYoVariable wrist = new DoubleYoVariable("wristAccelerationIntegrationAlphaPosition", registry);

      elbow.set(0.999);
      wrist.set(0.999);

      jointAccelerationIntegrationAlphaPosition = new HashMap<>();
      jointAccelerationIntegrationAlphaPosition.put(ArmJointName.ELBOW_ROLL, elbow);
      jointAccelerationIntegrationAlphaPosition.put(ArmJointName.FIRST_WRIST_PITCH, wrist);
      jointAccelerationIntegrationAlphaPosition.put(ArmJointName.WRIST_ROLL, wrist);

      return jointAccelerationIntegrationAlphaPosition;
   }

   private Map<ArmJointName, DoubleYoVariable> jointAccelerationIntegrationAlphaVelocity;

   /** {@inheritDoc} */
   @Override
   public Map<ArmJointName, DoubleYoVariable> getOrCreateAccelerationIntegrationAlphaVelocity(YoVariableRegistry registry)
   {
      if (jointAccelerationIntegrationAlphaVelocity != null)
         return jointAccelerationIntegrationAlphaVelocity;

      DoubleYoVariable elbow = new DoubleYoVariable("elbowAccelerationIntegrationAlphaVelocity", registry);
      DoubleYoVariable wrist = new DoubleYoVariable("wristAccelerationIntegrationAlphaVelocity", registry);

      elbow.set(0.83);
      wrist.set(0.83);

      jointAccelerationIntegrationAlphaVelocity = new HashMap<>();
      jointAccelerationIntegrationAlphaVelocity.put(ArmJointName.ELBOW_ROLL, elbow);
      jointAccelerationIntegrationAlphaVelocity.put(ArmJointName.FIRST_WRIST_PITCH, wrist);
      jointAccelerationIntegrationAlphaVelocity.put(ArmJointName.WRIST_ROLL, wrist);

      return jointAccelerationIntegrationAlphaVelocity;
   }

   @Override
   public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide)
   {
      Map<OneDoFJoint, Double> jointPositions = new LinkedHashMap<OneDoFJoint, Double>();

      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(-1.2));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_PITCH), -0.2);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_YAW), 0.7);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH), robotSide.negateIfRightSide(-1.5));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_ROLL), 1.3);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.0);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.WRIST_ROLL), 0.0);

      return jointPositions;
   }
}
