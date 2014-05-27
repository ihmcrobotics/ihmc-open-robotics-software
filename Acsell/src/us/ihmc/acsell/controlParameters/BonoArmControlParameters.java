package us.ihmc.acsell.controlParameters;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;

/**
 * Created by dstephen on 2/14/14.
 */
public class BonoArmControlParameters implements ArmControllerParameters
{
   private final boolean runningOnRealRobot;
   
   public BonoArmControlParameters()
   {
      this(false);
   }
   
   public BonoArmControlParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   public double[] getLowLevelArmJointspaceKp()
   {
      return new double[0]; 
   }
   public double[] getLowLevelArmJointspaceKi()
   {
      return new double[0]; 
   }
   public double[] getLowLevelArmJointspaceKd()
   {
      return new double[0]; 
   }
   public double[] getLowLevelArmJointspaceFfqd_d()
   {
      return new double[0]; 
   }
   public double[] getLowLevelArmJointspaceQerrMax()
   {
      return new double[0];  
   }
   
   public double getArmJointspaceKp()
   {
      return 0;
   }

   public double getArmJointspaceZeta()
   {
      return 0;
   }

   public double getArmJointspaceKi()
   {
      return 0;
   }

   public double getArmJointspaceMaxIntegralError()
   {
      return 0;
   }

   public double getArmJointspaceMaxAcceleration()
   {
      return 0;
   }

   public double getArmJointspaceMaxJerk()
   {
      return 0;
   }

   public double getArmTaskspaceKp()
   {
      return 0;
   }

   public double getArmTaskspaceZeta()
   {
      return 0;
   }

   public double getArmTaskspaceKi()
   {
      return 0;
   }

   public double getArmTaskspaceMaxIntegralError()
   {
      return 0;
   }

   public double getArmTaskspaceMaxAcceleration()
   {
      return 0;
   }

   public double getArmTaskspaceMaxJerk()
   {
      return 0;
   }

   public boolean useInverseKinematicsTaskspaceControl()
   {
      return false;
   }

   public boolean doLowLevelPositionControl()
   {
      return false;
   }
}
