package us.ihmc.atlas.behaviors;

import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepBehaviorUI;

public class AtlasLookAndStepBehaviorUIAndModule
{
   public static void main(String[] args)
   {
      new AtlasBehaviorUIAndModule(BehaviorUIRegistry.of(LookAndStepBehaviorUI.DEFINITION));
   }
}
