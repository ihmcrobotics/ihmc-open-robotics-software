package us.ihmc.atlas.parameters;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.controller.GainCalculator;
import com.yobotics.simulationconstructionset.util.controller.YoIndependentSE3PIDGains;
import com.yobotics.simulationconstructionset.util.controller.YoPIDGains;
import com.yobotics.simulationconstructionset.util.controller.YoSE3PIDGains;
import com.yobotics.simulationconstructionset.util.controller.YoSymmetricSE3PIDGains;

public class AtlasArmControllerParameters implements ArmControllerParameters
{
   private final boolean runningOnRealRobot;
   // TODO Must disappear
   private static final double[] kp = { 40.0, 20.0, 30.0, 30.0, 20.0, 30.0}; //{ 1.8, 1.5, 7.8, 6, 30, 24 };
   private static final double[] ki = { 1.5, 0.8, 5, 10, 5, 10 };
   private static final double[] kd = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; //{ 11, 17, 39, 47, 26, 25 };
   private static final double[] ff_qd_ds = { 60.0, 80.0, 50.0, 50.0, 30.0, 30.0 }; //{ 35, 35, 35, 35, 35, 35 };
   private static final double[] qerr_maxs = { 0.1, 0.2, 0.015, 0.05, 0.1, 0.05 };

   public AtlasArmControllerParameters()
   {
      this(false);
   }

   public AtlasArmControllerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   @Override
   public double[] getLowLevelArmJointspaceKp()
   {
      return kp;
   }
   @Override
   public double[] getLowLevelArmJointspaceKi()
   {
      return ki;
   }
   @Override
   public double[] getLowLevelArmJointspaceKd()
   {
      return kd;
   }
   @Override
   public double[] getLowLevelArmJointspaceFfqd_d()
   {
      return ff_qd_ds;
   }
   @Override
   public double[] getLowLevelArmJointspaceQerrMax()
   {
      return qerr_maxs;
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
      return false;
   }

   @Override
   public boolean doLowLevelPositionControl()
   {
      return runningOnRealRobot;
   }

   @Override
   public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      Map<OneDoFJoint, Double> jointPositions = new LinkedHashMap<OneDoFJoint, Double>();

      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(-1.30));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_PITCH), 0.34);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_ROLL), robotSide.negateIfRightSide(1.18));
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH), 1.94);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.WRIST_PITCH), -0.19);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.WRIST_ROLL), robotSide.negateIfRightSide(-0.07));

      return jointPositions;
   }
}
