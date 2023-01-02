package us.ihmc.rdx.ui.behavior.editor.actions;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.internal.ImGui;
import imgui.type.ImInt;
import us.ihmc.behaviors.sequence.actions.HandConfigurationActionData;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;

public class RDXHandConfigurationAction extends RDXBehaviorAction
{
   private final HandConfigurationActionData actionData = new HandConfigurationActionData();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImIntegerWrapper sideWidget = new ImIntegerWrapper(actionData::getSide, actionData::setSide, labels.get("Side"));
   private final ImInt handConfigurationIndex = new ImInt(6);
   private final String[] handConfigurationNames = new String[HandConfiguration.values.length];

   public RDXHandConfigurationAction()
   {
      super("Hand Configuration");

      HandConfiguration[] values = HandConfiguration.values;
      for (int i = 0; i < values.length; i++)
      {
         handConfigurationNames[i] = values[i].name();
      }
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(100.0f);
      sideWidget.renderImGuiWidget();
      handConfigurationIndex.set(actionData.getHandConfigurationIndex());
      ImGui.combo(labels.get("Grip"), handConfigurationIndex, handConfigurationNames);
      actionData.setHandConfigurationIndex(handConfigurationIndex.get());
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
