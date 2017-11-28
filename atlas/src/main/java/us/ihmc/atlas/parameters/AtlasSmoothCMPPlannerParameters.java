package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.AngularMomentumEstimationParameters;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.CoPSupportPolygonNames;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.euclid.tuple2D.Vector2D;

public class AtlasSmoothCMPPlannerParameters extends SmoothCMPPlannerParameters
{
   private final double scale;
   private final AtlasPhysicalProperties atlasPhysicalProperties;

   public AtlasSmoothCMPPlannerParameters(AtlasPhysicalProperties atlasPhysicalProperties)
   {
      super(atlasPhysicalProperties.getModelScale());
      scale = atlasPhysicalProperties.getModelScale();
      this.atlasPhysicalProperties = atlasPhysicalProperties;
      endCoPName = CoPPointName.MIDFEET_COP;
      entryCoPName = CoPPointName.HEEL_COP;
      exitCoPName = CoPPointName.TOE_COP;
      swingCopPointsToPlan = new CoPPointName[]{CoPPointName.BALL_COP, CoPPointName.TOE_COP};
      transferCoPPointsToPlan = new CoPPointName[]{CoPPointName.MIDFEET_COP, CoPPointName.HEEL_COP};

      copOffsetFrameNames.clear();
      stepLengthOffsetPolygon.clear();
      constrainToMinMax.clear();
      constrainToSupportPolygon.clear();
      stepLengthToCoPOffsetFactor.clear();
      copOffsetsInFootFrame.clear();
      copOffsetBoundsInFootFrame.clear();

      copOffsetFrameNames.put(CoPPointName.HEEL_COP, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON);
      copOffsetFrameNames.put(CoPPointName.BALL_COP, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON);
      copOffsetFrameNames.put(CoPPointName.TOE_COP, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON);
      copOffsetFrameNames.put(CoPPointName.FLAMINGO_STANCE_FINAL_COP, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON);
      copOffsetFrameNames.put(CoPPointName.MIDFEET_COP, CoPSupportPolygonNames.INITIAL_DOUBLE_SUPPORT_POLYGON);

      stepLengthOffsetPolygon.put(CoPPointName.MIDFEET_COP, CoPSupportPolygonNames.NULL);
      stepLengthOffsetPolygon.put(CoPPointName.HEEL_COP, CoPSupportPolygonNames.INITIAL_SWING_POLYGON);
      stepLengthOffsetPolygon.put(CoPPointName.BALL_COP, CoPSupportPolygonNames.FINAL_SWING_POLYGON);
      stepLengthOffsetPolygon.put(CoPPointName.TOE_COP, CoPSupportPolygonNames.FINAL_SWING_POLYGON);
      stepLengthOffsetPolygon.put(CoPPointName.FLAMINGO_STANCE_FINAL_COP, CoPSupportPolygonNames.NULL);

      constrainToMinMax.put(CoPPointName.MIDFEET_COP, false);
      constrainToMinMax.put(CoPPointName.HEEL_COP, true);
      constrainToMinMax.put(CoPPointName.BALL_COP, true);
      constrainToMinMax.put(CoPPointName.TOE_COP, true);
      constrainToMinMax.put(CoPPointName.FLAMINGO_STANCE_FINAL_COP, true);

      constrainToSupportPolygon.put(CoPPointName.MIDFEET_COP, false);
      constrainToSupportPolygon.put(CoPPointName.HEEL_COP, true);
      constrainToSupportPolygon.put(CoPPointName.BALL_COP, true);
      constrainToSupportPolygon.put(CoPPointName.TOE_COP, true);
      constrainToSupportPolygon.put(CoPPointName.FLAMINGO_STANCE_FINAL_COP, true);

      stepLengthToCoPOffsetFactor.put(CoPPointName.MIDFEET_COP, 0.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.HEEL_COP, 1.0 / 3.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.BALL_COP, 1.0 / 8.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.TOE_COP, 1.0 / 3.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.FLAMINGO_STANCE_FINAL_COP, 0.0);

      copOffsetsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(0.0, 0.0));
      copOffsetsInFootFrame.put(CoPPointName.HEEL_COP, new Vector2D(0.0, -0.005));
      copOffsetsInFootFrame.put(CoPPointName.BALL_COP, new Vector2D(0.0, 0.01));
      copOffsetsInFootFrame.put(CoPPointName.TOE_COP, new Vector2D(0.0, 0.025));
      copOffsetsInFootFrame.put(CoPPointName.FLAMINGO_STANCE_FINAL_COP, new Vector2D(0.0, 0.000));

      copOffsetBoundsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY));
      copOffsetBoundsInFootFrame.put(CoPPointName.HEEL_COP, new Vector2D(-0.04, 0.03));
      copOffsetBoundsInFootFrame.put(CoPPointName.BALL_COP, new Vector2D(0.0, 0.055));
      copOffsetBoundsInFootFrame.put(CoPPointName.TOE_COP, new Vector2D(0.0, 0.08));
      copOffsetBoundsInFootFrame.put(CoPPointName.FLAMINGO_STANCE_FINAL_COP, new Vector2D(0.0, 0.00));
   }

   @Override
   public boolean planWithAngularMomentum()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepLengthThresholdForExitCoPOnToesWhenSteppingDown()
   {
      return atlasPhysicalProperties.getFootLengthForControl();
   }

   @Override
   public double getTransferSplitFraction()
   {
      return 0.5;
   }

   @Override
   public double getSwingSplitFraction()
   {
      return 0.5;
   }

   @Override
   public double getSwingDurationShiftFraction()
   {
      return 0.8;
   }

   @Override
   public AngularMomentumEstimationParameters getAngularMomentumEstimationParameters()
   {
      return new AngularMomentumEstimationParameters()
      {
         @Override
         public double getPercentageSwingLegMass()
         {
            return 0.02;
         }

         @Override
         public double getPercentageSupportLegMass()
         {
            return 0.02;
         }
      };
   }
}
