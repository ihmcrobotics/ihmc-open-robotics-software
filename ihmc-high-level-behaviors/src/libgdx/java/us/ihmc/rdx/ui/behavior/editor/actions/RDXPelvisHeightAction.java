package us.ihmc.rdx.ui.behavior.editor.actions;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import us.ihmc.behaviors.sequence.actions.PelvisHeightActionData;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

public class RDXPelvisHeightAction extends RDXBehaviorAction
{
   private final PelvisHeightActionData actionData = new PelvisHeightActionData();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper heightInWorldWidget = new ImDoubleWrapper(actionData::getHeightInWorld,
                                                                           actionData::setHeightInWorld,
                                                                           imDouble -> ImGui.inputDouble(labels.get("Height in world"), imDouble));
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(actionData::getTrajectoryDuration,
                                                                                actionData::setTrajectoryDuration,
                                                                                imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));

   public RDXPelvisHeightAction()
   {
      super("Pelvis Height");
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      heightInWorldWidget.renderImGuiWidget();
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      actionData.saveToFile(jsonNode);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      actionData.loadFromFile(jsonNode);
   }
}
