package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionState;
import us.ihmc.handsros2.abilityHand.AbilityHandManager;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.widgets.ImGuiHandWidget;

public class RDXAbilityHandAction extends RDXActionNode<AbilityHandActionState, AbilityHandActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiHandWidget handWidget = new ImGuiHandWidget();

   public RDXAbilityHandAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new AbilityHandActionState(id, rootNode.getState()), rootNode);
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.text("Ability Hand action for " + definition.getSide().getLowerCaseName() + " side");

      if (ImGui.beginCombo(labels.get("Grip"), definition.getGrip().name()))
      {
         for (AbilityHandManager.Grip grip : AbilityHandManager.Grip.values)
         {
            boolean isSelected = definition.getGrip() == grip;
            if (ImGui.selectable(grip.name(), isSelected))
            {
               definition.setGrip(grip);
            }
            if (isSelected)
            {
               ImGui.setItemDefaultFocus();
            }
         }
         ImGui.endCombo();
      }
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();

      ImGui.sameLine();
      handWidget.render(definition.getSide(), ImGui.getFrameHeight(), false);
   }

   @Override
   public String getLeafTypeTitle()
   {
      return definition.getSide().getPascalCaseName() + " Ability Hand";
   }
}
