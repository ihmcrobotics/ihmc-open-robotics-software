package us.ihmc.wanderer.controlParameters;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;

public class WandererSwingTrajectoryParameters extends SwingTrajectoryParameters
{
   private final boolean runningOnRealRobot;

   public WandererSwingTrajectoryParameters(boolean runningOnRealRobot)
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
      return -0.02;
   }

   @Override
   public double getDesiredTouchdownVelocity()
   {
      return -0.1;
   }

   @Override
   public double getDesiredTouchdownAcceleration()
   {
      return 0;
   }

}
