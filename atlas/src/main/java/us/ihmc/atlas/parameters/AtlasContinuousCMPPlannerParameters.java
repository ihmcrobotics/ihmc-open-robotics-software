package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

/** {@inheritDoc} */
public class AtlasContinuousCMPPlannerParameters extends ContinuousCMPICPPlannerParameters
{
   private final double scale;
   private final boolean useTwoCMPsPerSupport;
   private final AtlasPhysicalProperties atlasPhysicalProperties;

   private EnumMap<CoPPointName, Vector2D> copOffsets;
   private EnumMap<CoPPointName, Vector2D> copForwardOffsetBounds;

   public AtlasContinuousCMPPlannerParameters(AtlasPhysicalProperties atlasPhysicalProperties)
   {
      super(atlasPhysicalProperties.getModelScale());
      scale = atlasPhysicalProperties.getModelScale();
      this.atlasPhysicalProperties = atlasPhysicalProperties;
      useTwoCMPsPerSupport = true;
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfFootstepsToConsider()
   {
      return 4;
   }

   /** {@inheritDoc} */
   @Override
   public EnumMap<CoPPointName, Vector2D> getCoPOffsetsInFootFrame()
   {
      if (copOffsets != null)
         return copOffsets;

      Vector2D entryOffset = new Vector2D(0.0, -0.005);
      Vector2D exitOffset = new Vector2D(0.0, 0.025);

      entryOffset.scale(scale);
      exitOffset.scale(scale);

      copOffsets = new EnumMap<>(CoPPointName.class);
      copOffsets.put(entryCoPName, entryOffset);
      copOffsets.put(exitCoPName, exitOffset);

      return copOffsets;
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
   public CoPPointName getExitCoPName()
   {
      return exitCoPName;
   }

   /** {@inheritDoc} */
   @Override
   public CoPPointName getEntryCoPName()
   {
      return entryCoPName;
   }

   /** {@inheritDoc} */
   @Override
   public EnumMap<CoPPointName, Vector2D> getCoPForwardOffsetBoundsInFoot()
   {
      if (copForwardOffsetBounds != null)
         return copForwardOffsetBounds;

      Vector2D entryBounds = new Vector2D(0.0, 0.03);
      Vector2D exitBounds = new Vector2D(-0.04, 0.08);

      entryBounds.scale(scale);
      exitBounds.scale(scale);

      copForwardOffsetBounds = new EnumMap<>(CoPPointName.class);
      copForwardOffsetBounds.put(entryCoPName, entryBounds);
      copForwardOffsetBounds.put(exitCoPName,exitBounds);

      return copForwardOffsetBounds;
   }

   /** {@inheritDoc} */
   @Override
   public double getCoPSafeDistanceAwayFromSupportEdges()
   {
      return scale * 0.02;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useExitCoPOnToesForSteppingDown()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepLengthThresholdForExitCoPOnToesWhenSteppingDown()
   {
      return atlasPhysicalProperties.getFootLengthForControl();
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityDecayDurationWhenDone()
   {
      return 0.5;
   }
}
