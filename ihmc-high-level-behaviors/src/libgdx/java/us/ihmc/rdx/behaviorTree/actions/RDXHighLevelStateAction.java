package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.action.actions.HighLevelStateActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.HighLevelStateActionState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.util.Arrays;

public class RDXHighLevelStateAction extends RDXActionNode<HighLevelStateActionState, HighLevelStateActionDefinition>
{
   private static final String KEEP_CURRENT_MODEL = "(keep current model)";

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper timeoutWidget;

   public RDXHighLevelStateAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new HighLevelStateActionState(id, rootNode.getState()), rootNode);

      timeoutWidget = new ImDoubleWrapper(definition::getTimeout,
                                          definition::setTimeout,
                                          imDouble -> ImGui.inputDouble(labels.get("Timeout"), imDouble));
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();

      ImGui.sameLine();
      ImGui.textDisabled(getSummary());

      renderRowEnd();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      HighLevelControllerName current = definition.getHighLevelControllerName();
      ImGui.pushItemWidth(200.0f);
      if (ImGui.beginCombo(labels.get("Controller state"), current.name()))
      {
         for (HighLevelControllerName value : HighLevelControllerName.values)
         {
            if (ImGui.selectable(value.name(), value == current))
               definition.setHighLevelControllerName(value);
         }
         ImGui.endCombo();
      }
      ImGui.popItemWidth();

      if (definition.getHighLevelControllerName() == HighLevelControllerName.RL_CONTROL)
         renderRLModelSelection();

      ImGui.pushItemWidth(80.0f);
      timeoutWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   /** The models come from the behavior executor through this action's state, once it has the controller's RL policy state. */
   private void renderRLModelSelection()
   {
      String availableModels = state.getAvailableRLModels().getValue();
      String[] modelNames = availableModels.isEmpty() ? new String[0] : availableModels.split("\n");
      String selected = definition.getRLModelName();
      boolean selectedIsAvailable = selected.isEmpty() || Arrays.asList(modelNames).contains(selected);
      String preview = selected.isEmpty() ? KEEP_CURRENT_MODEL : selectedIsAvailable ? selected : selected + " (not available)";

      ImGui.pushItemWidth(200.0f);
      if (ImGui.beginCombo(labels.get("RL model"), preview))
      {
         if (ImGui.selectable(KEEP_CURRENT_MODEL, selected.isEmpty()))
            definition.setRLModelName("");
         for (String modelName : modelNames)
         {
            if (ImGui.selectable(modelName, modelName.equals(selected)))
               definition.setRLModelName(modelName);
         }
         ImGui.endCombo();
      }
      ImGui.popItemWidth();

      if (modelNames.length == 0)
         ImGui.textDisabled("Waiting for the RL models from the controller");
   }

   private String getSummary()
   {
      HighLevelControllerName requested = definition.getHighLevelControllerName();
      if (requested == HighLevelControllerName.RL_CONTROL && !definition.getRLModelName().isEmpty())
         return requested.name() + " " + definition.getRLModelName();
      return requested.name();
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Controller State " + getSummary();
   }
}
