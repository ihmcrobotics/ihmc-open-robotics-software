package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImString;
import us.ihmc.behaviors.behaviorTree.action.actions.FollowActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.FollowActionState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXFollowAction extends RDXActionNode<FollowActionState, FollowActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString targetLabel = new ImString("", 128);
   private final ImBooleanWrapper closestPersonWidget;

   public RDXFollowAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new FollowActionState(id, rootNode.getState()), rootNode);
      targetLabel.set(definition.getTargetLabel());
      closestPersonWidget = new ImBooleanWrapper(definition::getClosestPerson,
                                                 definition::setClosestPerson,
                                                 imBoolean -> ImGui.checkbox(labels.get("Closest person"), imBoolean));
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();
      ImGui.sameLine();
      ImGui.textDisabled(getLeafTypeTitle());
      renderRowEnd();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      if (!targetLabel.get().equals(definition.getTargetLabel()) && !ImGui.isAnyItemActive())
         targetLabel.set(definition.getTargetLabel());
      if (ImGui.inputText(labels.get("Target"), targetLabel, ImGuiInputTextFlags.EnterReturnsTrue))
         definition.setTargetLabel(targetLabel.get().trim());
      closestPersonWidget.renderImGuiWidget();
   }

   @Override
   public String getLeafTypeTitle()
   {
      if (definition.getClosestPerson())
         return "Follow closest person";
      String label = definition.getTargetLabel();
      if (label == null || label.isBlank())
         return "Follow";
      return "Follow " + label;
   }
}
