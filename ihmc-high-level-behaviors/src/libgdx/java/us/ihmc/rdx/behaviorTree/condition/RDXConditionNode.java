package us.ihmc.rdx.behaviorTree.condition;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition.ConditionNodeType;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXLeafNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.widgets.ImGuiConditionNodeWidget;

public class RDXConditionNode extends RDXLeafNode<ConditionNodeState, ConditionNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiConditionNodeWidget conditionIconWidget = new ImGuiConditionNodeWidget();

   private final RDXCounterCondition counter;
   private final RDXLLMCondition llm;
   private final RDXProximityCondition proximityCheck;

   public RDXConditionNode(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new ConditionNodeState(id, rootNode.getState()), rootNode);

      counter = new RDXCounterCondition(state);
      llm = new RDXLLMCondition(state);
      proximityCheck = new RDXProximityCondition(state, scene);
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();

      ImGui.sameLine();
      conditionIconWidget.render();

      renderRowEnd();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ConditionNodeType currentConditionType = definition.getConditionType().getValue();
      if (ImGui.beginCombo(labels.get("Condition Type"), currentConditionType.name()))
      {
         for (ConditionNodeType value : ConditionNodeType.values)
         {
            if (ImGui.selectable(value.name(), value == currentConditionType))
            {
               definition.getConditionType().setValue(value);
            }
         }

         ImGui.endCombo();
      }


      switch (currentConditionType)
      {
         case COUNTER -> counter.renderImGuiWidgetsInternal();
         case LLM -> llm.renderImGuiWidgetsInternal();
         case PROXIMITY -> proximityCheck.renderImGuiWidgetsInternal();
      }
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Condition Node";
   }
}