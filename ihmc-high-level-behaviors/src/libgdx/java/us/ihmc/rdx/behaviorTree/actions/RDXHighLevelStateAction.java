package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.action.actions.HighLevelStateActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.HighLevelStateActionState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXHighLevelStateAction extends RDXActionNode<HighLevelStateActionState, HighLevelStateActionDefinition>
{
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
      ImGui.textDisabled(definition.getHighLevelControllerName().name());

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

      ImGui.pushItemWidth(80.0f);
      timeoutWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Controller State " + definition.getHighLevelControllerName().name();
   }
}
