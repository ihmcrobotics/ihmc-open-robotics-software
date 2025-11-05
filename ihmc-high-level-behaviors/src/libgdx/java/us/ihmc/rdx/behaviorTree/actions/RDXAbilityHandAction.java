package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXAbilityHandAction extends RDXActionNode<AbilityHandActionState, AbilityHandActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public RDXAbilityHandAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new AbilityHandActionState(id, rootNode.getState()), rootNode);
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.text("Ability Hand action for " + definition.getSide().getLowerCaseName() + " side");
      // TODO: Add ImGui widgets for Ability Hand control
   }

   @Override
   public String getLeafTypeTitle()
   {
      return definition.getSide().getPascalCaseName() + " Ability Hand";
   }
}
