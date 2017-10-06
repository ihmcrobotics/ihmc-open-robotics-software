package us.ihmc.thor.parameters;

import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;
import java.util.List;

/** {@inheritDoc} */
public class ThorCapturePointPlannerParameters extends ContinuousCMPICPPlannerParameters
{
   private final boolean runningOnRealRobot;
   private final boolean useTwoCMPsPerSupport;

   private List<Vector2D> copOffsets;
   private List<Vector2D> copForwardOffsetBounds;

   public ThorCapturePointPlannerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      useTwoCMPsPerSupport = false;
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfCoPWayPointsPerFoot()
   {
      if (useTwoCMPsPerSupport)
         return 2;
      else
         return 1;
   }

   /** {@inheritDoc} */
   @Override
   public List<Vector2D> getCoPOffsets()
   {
      if (copOffsets != null)
         return copOffsets;

      Vector2D entryOffset, exitOffset;
      if (runningOnRealRobot)
         entryOffset = new Vector2D(0.01, 0.02);
      else
         entryOffset = new Vector2D(0.0, 0.006);

      exitOffset = new Vector2D(0.0, 0.025);

      copOffsets = new ArrayList<>();
      copOffsets.add(entryOffset);
      copOffsets.add(exitOffset);

      return copOffsets;
   }

   /** {@inheritDoc} */
   @Override
   public List<Vector2D> getCoPForwardOffsetBounds()
   {
      if (copForwardOffsetBounds != null)
         return copForwardOffsetBounds;

      Vector2D entryBounds = new Vector2D(0.0, 0.03);
      Vector2D exitBounds = new Vector2D(-0.04, 0.06);

      copForwardOffsetBounds = new ArrayList<>();
      copForwardOffsetBounds.add(entryBounds);
      copForwardOffsetBounds.add(exitBounds);

      return copForwardOffsetBounds;
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityDecayDurationWhenDone()
   {
      return 0.5;
   }
}
