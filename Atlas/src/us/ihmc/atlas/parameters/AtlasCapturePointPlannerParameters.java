package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;

/** {@inheritDoc} */
public class AtlasCapturePointPlannerParameters extends CapturePointPlannerParameters
{
   private final double scale;
   private final boolean useTwoCMPsPerSupport;
   private final AtlasPhysicalProperties atlasPhysicalProperties;
   
   public AtlasCapturePointPlannerParameters(AtlasPhysicalProperties atlasPhysicalProperties)
   {
      super(atlasPhysicalProperties.getModelScale());
      scale = atlasPhysicalProperties.getModelScale();
      this.atlasPhysicalProperties = atlasPhysicalProperties;
      useTwoCMPsPerSupport = true;
   }

   /** {@inheritDoc} */
   @Override
   public double getDoubleSupportInitialTransferDuration()
   {
      return 1.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getAdditionalTimeForSingleSupport()
   {
      return 0.1;
   }

   /** {@inheritDoc} */
   @Override
   public double getEntryCMPInsideOffset()
   {
      return scale * -0.005; //0.006;
   }

   /** {@inheritDoc} */
   @Override
   public double getExitCMPInsideOffset()
   {
      return scale * 0.025;
   }

   /** {@inheritDoc} */
   @Override
   public double getEntryCMPForwardOffset()
   {
      return scale * 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getExitCMPForwardOffset()
   {
      return scale * 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useTwoCMPsPerSupport()
   {
      return useTwoCMPsPerSupport;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxEntryCMPForwardOffset()
   {
      return scale * 0.03;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinEntryCMPForwardOffset()
   {
      return 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxExitCMPForwardOffset()
   {
      return scale * 0.08;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinExitCMPForwardOffset()
   {
      return scale * -0.04;
   }

   /** {@inheritDoc} */
   @Override
   public double getCMPSafeDistanceAwayFromSupportEdges()
   {
      return scale * 0.02;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useExitCMPOnToesForSteppingDown()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepLengthThresholdForExitCMPOnToesWhenSteppingDown()
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
