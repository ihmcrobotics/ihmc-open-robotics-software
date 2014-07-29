package us.ihmc.valkyrie.paramaters;

import java.util.LinkedHashMap;
import java.util.Map;

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
   
   public double[] getLowLevelArmJointspaceKp()
   {
      return lowLevelArmJointSpace_kp; 
   }
   public double[] getLowLevelArmJointspaceKi()
   {
      return lowLevelArmJointSpace_ki; 
   }
   public double[] getLowLevelArmJointspaceKd()
   {
      return lowLevelArmJointSpace_kd; 
   }
   public double[] getLowLevelArmJointspaceFfqd_d()
   {
      return lowLevelArmJointSpace_ff_qd_ds; 
   }
   public double[] getLowLevelArmJointspaceQerrMax()
   {
      return lowLevelArmJointSpace_qerr_maxs; 
   }

   public double getArmJointspaceKp()
   {
      if (!runningOnRealRobot)  return 80.0;
      return 80.0; 
   }

   public double getArmJointspaceZeta()
   {
      if (!runningOnRealRobot)  return 0.6;
      return 0.2;  // Lots of natural damping in the arms. Don't need to damp the controllers.
   }

   public double getArmJointspaceKi()
   {
      if (!runningOnRealRobot)  return 0.0;
      return 20.0; //0.0 
   }

   public double getArmJointspaceMaxIntegralError()
   {
      if (!runningOnRealRobot)  return 0.0;
      return 0.2; //0.0;
   }

   public double getArmJointspaceMaxAcceleration()
   {
      if (!runningOnRealRobot)  return Double.POSITIVE_INFINITY;
      return 5.0; //1.0; //20.0;
   }

   public double getArmJointspaceMaxJerk()
   {
      if (!runningOnRealRobot)  return Double.POSITIVE_INFINITY;
      return 100.0;
   }

   public double getArmTaskspaceKp()
   {
      if (!runningOnRealRobot)  return 100.0;
      return 100.0; 
   }

   public double getArmTaskspaceZeta()
   {
      if (!runningOnRealRobot)  return 1.0;
      return 0.6; 
   }

   public double getArmTaskspaceKi()
   {
      if (!runningOnRealRobot)  return 0.0;
      return 0.0; 
   }

   public double getArmTaskspaceMaxIntegralError()
   {
      if (!runningOnRealRobot)  return 0.0;
      return 0.0; 
   }

   public double getArmTaskspaceMaxAcceleration()
   {
      if (!runningOnRealRobot)  return Double.POSITIVE_INFINITY;
      return 10.0; 
   }

   public double getArmTaskspaceMaxJerk()
   {
      if (!runningOnRealRobot)  return Double.POSITIVE_INFINITY;
      return 100.0; 
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
