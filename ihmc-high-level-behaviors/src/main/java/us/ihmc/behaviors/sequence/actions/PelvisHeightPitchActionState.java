package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class PelvisHeightPitchActionState extends BehaviorActionState
{
   private final PelvisHeightPitchActionDefinition definition = new PelvisHeightPitchActionDefinition();

   private ReferenceFrame pelvisFrame;

   @Override
   public PelvisHeightPitchActionDefinition getDefinition()
   {
      return definition;
   }

   public ReferenceFrame getPelvisFrame()
   {
      return pelvisFrame;
   }
}
