package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;

public class ChestOrientationActionState extends BehaviorActionState
{
   private final ChestOrientationActionDefinition definition = new ChestOrientationActionDefinition();
   private final DetachableReferenceFrame chestFrame = new DetachableReferenceFrame(definition.getTransformToParent());

   @Override
   public ChestOrientationActionDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getChestFrame()
   {
      return chestFrame;
   }
}
