package us.ihmc.atlas.parameters;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;

public class AtlasSwingTrajectoryParameters extends SwingTrajectoryParameters
{
   protected final RobotTarget target;
   protected final double modelScale;
   protected final boolean runningOnRealRobot;

   public AtlasSwingTrajectoryParameters(RobotTarget target, double modelScale)
   {
      this.target = target;
      this.modelScale = modelScale;

      runningOnRealRobot = target == RobotTarget.REAL_ROBOT;
   }

   @Override
   public double getMinSwingHeight()
   {
      return 0.10 * modelScale;
   }

   @Override
   public double getDefaultSwingHeight()
   {
      return getMinSwingHeight();
   }
   
   @Override
   public double getMaxSwingHeight()
   {
      return 0.40 * modelScale;
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
