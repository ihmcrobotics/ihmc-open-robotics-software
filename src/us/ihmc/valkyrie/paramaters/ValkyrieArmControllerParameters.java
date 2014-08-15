package us.ihmc.valkyrie.paramaters;

import java.util.LinkedHashMap;
import java.util.Map;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.controller.YoIndependentSE3PIDGains;
import com.yobotics.simulationconstructionset.util.controller.YoPIDGains;
import com.yobotics.simulationconstructionset.util.controller.YoSE3PIDGains;
import com.yobotics.simulationconstructionset.util.controller.YoSymmetricSE3PIDGains;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class ValkyrieArmControllerParameters implements ArmControllerParameters
{
   private final boolean runningOnRealRobot;

   public ValkyrieArmControllerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   public static final double[] lowLevelArmJointSpace_kp = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   public static final double[] lowLevelArmJointSpace_ki = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
   public static final double[] lowLevelArmJointSpace_kd = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
   public static final double[] lowLevelArmJointSpace_ff_qd_ds = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   public static final double[] lowLevelArmJointSpace_qerr_maxs = { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 };

   @Override
   public double[] getLowLevelArmJointspaceKp()
   {
      return lowLevelArmJointSpace_kp;
   }
   @Override
   public double[] getLowLevelArmJointspaceKi()
   {
      return lowLevelArmJointSpace_ki;
   }
   @Override
   public double[] getLowLevelArmJointspaceKd()
   {
      return lowLevelArmJointSpace_kd;
   }
   @Override
   public double[] getLowLevelArmJointspaceFfqd_d()
   {
      return lowLevelArmJointSpace_ff_qd_ds;
   }
   @Override
   public double[] getLowLevelArmJointspaceQerrMax()
   {
      return lowLevelArmJointSpace_qerr_maxs;
   }

   @Override
   public YoPIDGains createJointspaceControlGains(YoVariableRegistry registry)
   {
      YoPIDGains jointspaceControlGains = new YoPIDGains("ArmJointspace", registry);

      double kp = runningOnRealRobot ? 80.0 : 80.0;
      double zeta = runningOnRealRobot ? 0.2 : 0.6;
      double ki = runningOnRealRobot ? 20.0 : 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 5.0 : Double.POSITIVE_INFINITY;
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
      return false;
   }

   @Override
   public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide)
   {
      Map<OneDoFJoint, Double> jointPositions = new LinkedHashMap<OneDoFJoint, Double>();

      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL), -0.25);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_PITCH), 0.3);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_YAW), 0.0);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH), -1.0);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_YAW), 0.0);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.WRIST_PITCH), 0.0);
      jointPositions.put(fullRobotModel.getArmJoint(robotSide, ArmJointName.WRIST_ROLL), 0.0);

      return jointPositions;
   }
}
