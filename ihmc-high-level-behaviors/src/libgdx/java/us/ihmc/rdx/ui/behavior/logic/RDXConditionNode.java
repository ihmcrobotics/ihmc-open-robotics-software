package us.ihmc.rdx.ui.behavior.logic;

import imgui.ImGui;
import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeDefinition.Type;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.logic.condition.RDXCounterCondition;
import us.ihmc.rdx.ui.behavior.logic.condition.RDXLLMCondition;
import us.ihmc.rdx.ui.behavior.logic.condition.RDXProximityCondition;
import us.ihmc.rdx.ui.behavior.sequence.RDXLeafNode;
import us.ihmc.rdx.ui.widgets.ImGuiConditionNodeWidget;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXConditionNode extends RDXLeafNode<ConditionNodeState, ConditionNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiConditionNodeWidget conditionIconWidget = new ImGuiConditionNodeWidget();

   private final RDXCounterCondition counter;
   private final RDXLLMCondition llm;
   private final RDXProximityCondition proximityCheck;

   public RDXConditionNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(new ConditionNodeState(id, crdtInfo, saveFileDirectory));

      counter = new RDXCounterCondition(state);
      llm = new RDXLLMCondition(state);
      proximityCheck = new RDXProximityCondition(state, referenceFrameLibrary);
   }

   @Override
   public void renderTreeViewIconArea()
   {
      super.renderTreeViewIconArea();

      conditionIconWidget.render();
      ImGui.sameLine();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      Type currentType = definition.getType().getValue();
      if (ImGui.beginCombo(labels.get("Type"), currentType.name()))
      {
         for (Type value : Type.values)
         {
            if (ImGui.selectable(value.name(), value == currentType))
            {
               definition.getType().setValue(value);
            }
         }

         ImGui.endCombo();
      }


      switch (currentType)
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