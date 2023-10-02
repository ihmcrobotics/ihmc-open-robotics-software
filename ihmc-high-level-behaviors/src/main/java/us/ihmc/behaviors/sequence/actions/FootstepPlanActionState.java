package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class FootstepPlanActionState extends BehaviorActionState
{
   private final FootstepPlanActionDefinition definition = new FootstepPlanActionDefinition();

   private ReferenceFrame soleFrame;

   @Override
   public FootstepPlanActionDefinition getDefinition()
   {
      return definition;
   }

   public ReferenceFrame getSoleFrame()
   {
      return soleFrame;
   }
}
