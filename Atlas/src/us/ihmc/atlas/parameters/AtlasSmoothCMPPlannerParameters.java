package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;

public class AtlasSmoothCMPPlannerParameters extends SmoothCMPPlannerParameters
{
   private final double scale;
   private final AtlasPhysicalProperties atlasPhysicalProperties;

   public AtlasSmoothCMPPlannerParameters(AtlasPhysicalProperties atlasPhysicalProperties)
   {
      super(atlasPhysicalProperties.getModelScale());
      scale = atlasPhysicalProperties.getModelScale();
      this.atlasPhysicalProperties = atlasPhysicalProperties;
   }

   /** {@inheritDoc} */
   @Override
   public double getStepLengthThresholdForExitCoPOnToesWhenSteppingDown()
   {
      return atlasPhysicalProperties.getFootLengthForControl();
   }
}
