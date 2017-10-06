package us.ihmc.steppr.controlParameters;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;

public class BonoSwingTrajectoryParameters extends SwingTrajectoryParameters
{
   private final boolean runningOnRealRobot;

   public BonoSwingTrajectoryParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   @Override
   public boolean doToeTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getToeTouchdownAngle()
   {
      return Math.toRadians(20.0);
   }

   @Override
   public boolean doHeelTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getHeelTouchdownAngle()
   {
      return Math.toRadians(-20.0);
   }


   @Override
   public double getMinMechanicalLegLength()
   {
      return 0.1;
   }

   @Override
   public double getDesiredTouchdownHeightOffset()
   {
      return runningOnRealRobot ? -0.02 : 0.0;
   }

   @Override
   public double getDesiredTouchdownVelocity()
   {
      return runningOnRealRobot ? -0.1 : -0.3;
   }

   @Override
   public double getDesiredTouchdownAcceleration()
   {
      return 0;
   }

}
