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
   
   @Override
   public double getArmJointspaceKp()
   {
      return 0;
   }

   @Override
   public double getArmJointspaceZeta()
   {
      return 0;
   }

   @Override
   public double getArmJointspaceKi()
   {
      return 0;
   }

   @Override
   public double getArmJointspaceMaxIntegralError()
   {
      return 0;
   }

   @Override
   public double getArmJointspaceMaxAcceleration()
   {
      return 0;
   }

   @Override
   public double getArmJointspaceMaxJerk()
   {
      return 0;
   }

   @Override
   public double getArmTaskspaceKp()
   {
      return 0;
   }

   @Override
   public double getArmTaskspaceZeta()
   {
      return 0;
   }

   @Override
   public double getArmTaskspaceKi()
   {
      return 0;
   }

   @Override
   public double getArmTaskspaceMaxIntegralError()
   {
      return 0;
   }

   @Override
   public double getArmTaskspaceMaxAcceleration()
   {
      return 0;
   }

   @Override
   public double getArmTaskspaceMaxJerk()
   {
      return 0;
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
}
