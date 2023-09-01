package us.ihmc.atlas.parameters;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;

public class AtlasSwingTrajectoryParameters extends SwingTrajectoryParameters
{
   private final RobotTarget target;
   private final double modelScale;
   private final boolean runningOnRealRobot;

   public AtlasSwingTrajectoryParameters(RobotTarget target, double modelScale)
   {
      this.target = target;
      this.modelScale = modelScale;

      runningOnRealRobot = target == RobotTarget.REAL_ROBOT;
   }

   @Override
   public double getDesiredTouchdownHeightOffset()
   {
      return 0;
   }

   @Override
   public double getDesiredTouchdownVelocity()
   {
      return modelScale * -0.3;
   }

   @Override
   public double getDesiredTouchdownAcceleration()
   {
      switch (target)
      {
      case SCS:
         return modelScale * -2.0;

      default :
         return modelScale * -1.0;
      }
   }

   /** {@inheritDoc} */
   @Override
   public boolean addOrientationMidpointForObstacleClearance()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useSingularityAvoidanceInSupport()
   {
      return true;
   }
}
