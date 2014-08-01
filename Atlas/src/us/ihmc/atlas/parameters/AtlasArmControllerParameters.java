package us.ihmc.atlas.parameters;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

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
   
   public double[] getLowLevelArmJointspaceKp()
   {
      return kp; 
   }
   public double[] getLowLevelArmJointspaceKi()
   {
      return ki; 
   }
   public double[] getLowLevelArmJointspaceKd()
   {
      return kd; 
   }
   public double[] getLowLevelArmJointspaceFfqd_d()
   {
      return ff_qd_ds; 
   }
   public double[] getLowLevelArmJointspaceQerrMax()
   {
      return qerr_maxs; 
   }

   public double getArmJointspaceKp()
   {
      if (!runningOnRealRobot)  return 80.0;
      return 40.0; 
   }

   public double getArmJointspaceZeta()
   {
      if (!runningOnRealRobot)  return 0.6;
      return 0.0;  // Lots of natural damping in the arms. Don't need to damp the controllers.
   }

   public double getArmJointspaceKi()
   {
      if (!runningOnRealRobot)  return 0.0;
      return 0.0;
   }

   public double getArmJointspaceMaxIntegralError()
   {
      if (!runningOnRealRobot)  return 0.0;
      return 0.0;
   }

   public double getArmJointspaceMaxAcceleration()
   {
      if (!runningOnRealRobot)  return Double.POSITIVE_INFINITY;
      return 20.0;
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
