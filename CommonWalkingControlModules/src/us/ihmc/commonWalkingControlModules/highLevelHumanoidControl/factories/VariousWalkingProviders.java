package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;

public class VariousWalkingProviders
{
   private final FootstepProvider footstepProvider;

   public VariousWalkingProviders(FootstepProvider footstepProvider)
   {
      this.footstepProvider = footstepProvider;

   }

   public FootstepProvider getFootstepProvider()
   {
      return footstepProvider;
   }
}
