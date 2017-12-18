package us.ihmc.escher.parameters;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

/** {@inheritDoc} */
public class EscherCapturePointPlannerParameters extends ContinuousCMPICPPlannerParameters
{
   private final boolean runningOnRealRobot;
   private final boolean useTwoCMPsPerSupport;

   private EnumMap<CoPPointName, Vector2D> copOffsets;
   private EnumMap<CoPPointName, Vector2D> copForwardOffsetBounds;

   public EscherCapturePointPlannerParameters(boolean runningOnRealRobot)
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

   /**{@inheritDoc} */
   @Override
   public CoPPointName getExitCoPName()
   {
      return exitCoPName;
   }

   /**{@inheritDoc} */
   @Override
   public CoPPointName getEntryCoPName()
   {
      return entryCoPName;
   }

   /** {@inheritDoc} */
   @Override
   public EnumMap<CoPPointName, Vector2D> getCoPOffsetsInFootFrame()
   {
      if (copOffsets != null)
         return copOffsets;

      Vector2D entryOffset, exitOffset;
      if (runningOnRealRobot)
         entryOffset = new Vector2D(0.01, 0.01);
      else
         entryOffset = new Vector2D(0.0, 0.006);

      exitOffset = new Vector2D(0.0, 0.025);

      copOffsets = new EnumMap<>(CoPPointName.class);
      copOffsets.put(entryCoPName, entryOffset);
      copOffsets.put(exitCoPName, exitOffset);

      return copOffsets;
   }


   /** {@inheritDoc} */
   @Override
   public EnumMap<CoPPointName, Vector2D> getCoPForwardOffsetBoundsInFoot()
   {
      if (copForwardOffsetBounds != null)
         return copForwardOffsetBounds;

      Vector2D entryBounds = new Vector2D(0.0, 0.03);
      Vector2D exitBounds = new Vector2D(-0.04, 0.06);

      copForwardOffsetBounds = new EnumMap<>(CoPPointName.class);
      copForwardOffsetBounds.put(entryCoPName, entryBounds);
      copForwardOffsetBounds.put(exitCoPName, exitBounds);

      return copForwardOffsetBounds;
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityDecayDurationWhenDone()
   {
      return 0.5;
   }
}
