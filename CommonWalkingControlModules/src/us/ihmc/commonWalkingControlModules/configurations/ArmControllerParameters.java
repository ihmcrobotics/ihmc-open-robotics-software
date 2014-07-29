package us.ihmc.commonWalkingControlModules.configurations;

import java.util.Map;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public interface ArmControllerParameters
{
   public abstract double getArmJointspaceKp();
   public abstract double getArmJointspaceZeta();
   public abstract double getArmJointspaceKi();
   public abstract double getArmJointspaceMaxIntegralError();
   public abstract double getArmJointspaceMaxAcceleration();
   public abstract double getArmJointspaceMaxJerk();

   public abstract double getArmTaskspaceKp();
   public abstract double getArmTaskspaceZeta();
   public abstract double getArmTaskspaceKi();
   public abstract double getArmTaskspaceMaxIntegralError();
   public abstract double getArmTaskspaceMaxAcceleration();
   public abstract double getArmTaskspaceMaxJerk();
   
   public abstract double[] getLowLevelArmJointspaceKp();
   public abstract double[] getLowLevelArmJointspaceKi();
   public abstract double[] getLowLevelArmJointspaceKd();
   public abstract double[] getLowLevelArmJointspaceFfqd_d();
   public abstract double[] getLowLevelArmJointspaceQerrMax();

   public abstract boolean useInverseKinematicsTaskspaceControl();
   
   public abstract boolean doLowLevelPositionControl();

   public abstract Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullRobotModel fullRobotModel, RobotSide robotSide);
}
