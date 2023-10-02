package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class ChestOrientationActionState extends BehaviorActionState
{
   private final ChestOrientationActionDefinition definition = new ChestOrientationActionDefinition();

   private ReferenceFrame chestFrame;

   @Override
   public ChestOrientationActionDefinition getDefinition()
   {
      return definition;
   }

   public ReferenceFrame getChestFrame()
   {
      return chestFrame;
   }
}
