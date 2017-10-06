package us.ihmc.thor.parameters;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;

public class ThorSwingTrajectoryParameters extends SwingTrajectoryParameters
{
   private final double min_mechanical_leg_length = 0.420;    // corresponds to a q_kny that is close to knee limit
   private final RobotTarget target;

   public ThorSwingTrajectoryParameters(RobotTarget target)
   {
      this.target = target;
   }

   @Override
   public double getMinMechanicalLegLength()
   {
      return min_mechanical_leg_length;
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


   /** {@inheritDoc} */
   @Override
   public double getSwingFootVelocityAdjustmentDamping()
   {
      boolean realRobot = target == RobotTarget.REAL_ROBOT;
      return realRobot ? 0.8 : 0.5; // Robert: 0.8
   }

   @Override
   public double getDesiredTouchdownHeightOffset()
   {
      return 0;
   }

   @Override
   public double getDesiredTouchdownVelocity()
   {
      return -0.12;
   }

   @Override
   public double getDesiredTouchdownAcceleration()
   {
      switch (target)
      {
      case SCS:
         return -2.0;

      default :
         return -1.0;
      }
   }

}
