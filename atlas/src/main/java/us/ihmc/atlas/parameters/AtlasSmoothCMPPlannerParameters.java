package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.AngularMomentumEstimationParameters;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
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
      entryCoPName = CoPPointName.ENTRY_COP;
      exitCoPName = CoPPointName.EXIT_COP;
      swingCopPointsToPlan = new CoPPointName[]{CoPPointName.MIDFOOT_COP, CoPPointName.EXIT_COP, CoPPointName.TOE_COP};
      transferCoPPointsToPlan = new CoPPointName[]{CoPPointName.MIDFEET_COP, CoPPointName.ENTRY_COP};

      stepLengthToCoPOffsetFactor.clear();
      copOffsetsInFootFrame.clear();
      copOffsetBoundsInFootFrame.clear();

      stepLengthToCoPOffsetFactor.put(CoPPointName.MIDFEET_COP, 0.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.ENTRY_COP, 1.0 / 3.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.MIDFOOT_COP, 1.0 / 8.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.EXIT_COP, 1.0 / 3.0);
      stepLengthToCoPOffsetFactor.put(CoPPointName.FLAMINGO_STANCE_FINAL_COP, 0.0);

      copOffsetsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(0.0, 0.0));
      copOffsetsInFootFrame.put(CoPPointName.ENTRY_COP, new Vector2D(0.0, -0.005));
      copOffsetsInFootFrame.put(CoPPointName.MIDFOOT_COP, new Vector2D(0.0, 0.01));
      copOffsetsInFootFrame.put(CoPPointName.EXIT_COP, new Vector2D(0.0, 0.025));
      copOffsetsInFootFrame.put(CoPPointName.FLAMINGO_STANCE_FINAL_COP, new Vector2D(0.0, 0.000));
      copOffsetsInFootFrame.put(CoPPointName.TOE_COP, new Vector2D(0.0, 0.03));

      copOffsetBoundsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY));
      copOffsetBoundsInFootFrame.put(CoPPointName.ENTRY_COP, new Vector2D(-0.04, 0.03));
      copOffsetBoundsInFootFrame.put(CoPPointName.MIDFOOT_COP, new Vector2D(0.0, 0.055));
      copOffsetBoundsInFootFrame.put(CoPPointName.EXIT_COP, new Vector2D(0.0, 0.08));
      copOffsetBoundsInFootFrame.put(CoPPointName.FLAMINGO_STANCE_FINAL_COP, new Vector2D(0.0, 0.0));
      copOffsetBoundsInFootFrame.put(CoPPointName.TOE_COP, new Vector2D(0.0, 0.0));
   }

   @Override
   public boolean planSwingAngularMomentum()
   {
      return true;
   }

   @Override
   public boolean planTransferAngularMomentum()
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
      return 0.85;
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

   @Override
   public boolean adjustCoPPlanForSingleSupportContinuity()
   {
      return false;
   }

   @Override
   public boolean adjustInitialCoPPlanForDoubleSupportContinuity()
   {
      return true;
   }

   @Override
   public boolean adjustEveryCoPPlanForDoubleSupportContinuity()
   {
      return true;
   }

   @Override
   public boolean adjustCoPPlanForStandingContinuity()
   {
      return true;
   }

   @Override
   public boolean doContinuousReplanningForStanding()
   {
      return true;
   }

   @Override
   public boolean doContinuousReplanningForTransfer()
   {
      return true;
   }

   @Override
   public boolean doContinuousReplanningForSwing()
   {
      return true;
   }

   @Override
   public boolean useExitCoPOnToesForSteppingDown()
   {
      return true;
   }
}
