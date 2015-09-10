package us.ihmc.atlas.parameters;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.controllers.GainCalculator;
import us.ihmc.yoUtilities.controllers.YoIndependentSE3PIDGains;
import us.ihmc.yoUtilities.controllers.YoPIDGains;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.controllers.YoSymmetricSE3PIDGains;


public class AtlasArmControllerParameters implements ArmControllerParameters
{
   private final boolean runningOnRealRobot;
   private final double handCenterOffset;

   public AtlasArmControllerParameters()
   {
      this(false, 0.0);
   }

   public AtlasArmControllerParameters(boolean runningOnRealRobot, double handCenterOffset)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      this.handCenterOffset = handCenterOffset;
   }

   @Override
   public YoPIDGains createJointspaceControlGains(YoVariableRegistry registry)
   {
      YoPIDGains jointspaceControlGains = new YoPIDGains("ArmJointspace", registry);

      double kp = runningOnRealRobot ? 40.0 : 80.0;
      double zeta = runningOnRealRobot ? 0.0 : 0.6;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      jointspaceControlGains.setKp(kp);
      jointspaceControlGains.setZeta(zeta);
      jointspaceControlGains.setKi(ki);
      jointspaceControlGains.setMaximumIntegralError(maxIntegralError);
      jointspaceControlGains.setMaximumAcceleration(maxAccel);
      jointspaceControlGains.setMaximumJerk(maxJerk);
      jointspaceControlGains.createDerivativeGainUpdater(true);
      
      return jointspaceControlGains;
   }

   @Override
   public YoSE3PIDGains createTaskspaceControlGains(YoVariableRegistry registry)
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
      taskspaceControlGains.setMaximumAcceleration(maxAccel);
      taskspaceControlGains.setMaximumJerk(maxJerk);
      taskspaceControlGains.createDerivativeGainUpdater(true);

      return taskspaceControlGains;
   }

   @Override
   public YoSE3PIDGains createTaskspaceControlGainsForLoadBearing(YoVariableRegistry registry)
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
      taskspaceControlGains.setOrientationMaxAccelerationAndJerk(maxAccel, maxJerk);
      taskspaceControlGains.setPositionMaxAccelerationAndJerk(maxAccel, maxJerk);
      
      return taskspaceControlGains;
   }

   @Override
   public boolean useInverseKinematicsTaskspaceControl()
   {
      return true;
   }

   @Override
   public boolean doLowLevelPositionControl()
   {
      return runningOnRealRobot; // Set to false for torque control
   }

   @Override
   public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide)
   {
      Map<OneDoFJoint, Double> jointPositions = new LinkedHashMap<OneDoFJoint, Double>();

      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(-1.30));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_YAW), robotSide.negateIfRightSide(0.1));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_ROLL), robotSide.negateIfRightSide(1.18));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH), 1.94);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.0);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.WRIST_ROLL), robotSide.negateIfRightSide(-0.07));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SECOND_WRIST_PITCH), robotSide.negateIfRightSide(0.0));

      return jointPositions;
   }

   @Override
   public double getWristHandCenterOffset()
   {
      return handCenterOffset;
   }
}
