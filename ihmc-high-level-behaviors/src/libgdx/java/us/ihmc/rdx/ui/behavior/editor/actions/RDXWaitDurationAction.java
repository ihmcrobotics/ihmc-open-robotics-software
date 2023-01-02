package us.ihmc.rdx.ui.behavior.editor.actions;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionData;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

public class RDXWaitDurationAction extends RDXBehaviorAction
{
   private final WaitDurationActionData actionData = new WaitDurationActionData();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper waitDurationWidget = new ImDoubleWrapper(actionData::getWaitDuration,
                                                                          actionData::setWaitDuration,
                                                                          imDouble -> ImGui.inputDouble(labels.get("Wait duration"), imDouble));

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      waitDurationWidget.renderImGuiWidget();
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

   @Override
   public String getNameForDisplay()
   {
      return String.format("Wait %.1f s", actionData.getWaitDuration());
   }
}
