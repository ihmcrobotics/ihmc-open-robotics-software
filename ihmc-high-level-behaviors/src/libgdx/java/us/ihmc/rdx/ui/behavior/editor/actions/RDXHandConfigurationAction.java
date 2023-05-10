package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.internal.ImGui;
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
   private final String[] handConfigurationNames = new String[HandConfiguration.values.length];
   private final ImIntegerWrapper handConfigurationIndex = new ImIntegerWrapper(actionData::getHandConfigurationIndex,
                                                                                actionData::setHandConfigurationIndex,
                                                                                imInt -> ImGui.combo(labels.get("Grip"),
                                                                                                     imInt,
                                                                                                     handConfigurationNames));

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
      handConfigurationIndex.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public HandConfigurationActionData getActionData()
   {
      return actionData;
   }
}
