package us.ihmc.steppr.controlParameters;

import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;
import java.util.List;

/** {@inheritDoc} */
public class BonoCapturePointPlannerParameters extends ContinuousCMPICPPlannerParameters
{
   private final boolean runningOnRealRobot;
   // TODO Try using new ICP planner with two CMPs.
   private final boolean useTwoCMPsPerSupport;

   private List<Vector2D> copOffsets;
   private List<Vector2D> copForwardOffsetBounds;

   public BonoCapturePointPlannerParameters(boolean runningOnRealRobot)
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

      Vector2D entryOffset = new Vector2D(0.0, 0.005);
      Vector2D exitOffset = new Vector2D(0.0, 0.025);

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
      Vector2D exitBounds = new Vector2D(-0.04, 0.08);

      copForwardOffsetBounds = new ArrayList<>();
      copForwardOffsetBounds.add(entryBounds);
      copForwardOffsetBounds.add(exitBounds);

      return copForwardOffsetBounds;
   }

   /** {@inheritDoc} */
   @Override
   public double getCoPSafeDistanceAwayFromSupportEdges()
   {
      return 0.03;
   }
}
