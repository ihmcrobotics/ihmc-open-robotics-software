package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;

public class AtlasArmControllerParameters implements ArmControllerParameters
{
   private final boolean runningOnRealRobot;
   
   // PD parameters tuned with Ziegler Nichols on June 22, 2014
   public static final double[] kp = { 3.2, 2.56, 10.4, 10.4, 65.6, 39.2};
   public static final double[] ki = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   public static final double[] kd = { 0.1, 0.156, 0.268, 0.321, 0.394, 0.206};
   public static final double[] ff_qd_ds = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   public static final double[] qerr_maxs = { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 };
   
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
      return DRCConfigParameters.USE_INVERSE_KINEMATICS_TASKSPACE_CONTROL;
   }

   @Override
   public boolean doLowLevelPositionControl()
   {
      return runningOnRealRobot && DRCConfigParameters.USE_LOW_LEVEL_POSITION_CONTROL_FOR_HANDS;
   }
}
