package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.internal.ImGui;
import us.ihmc.behaviors.sequence.actions.HandConfigurationActionDescription;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;

public class RDXHandConfigurationAction extends RDXBehaviorAction
{
   private final HandConfigurationActionDescription actionDescription = new HandConfigurationActionDescription();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImIntegerWrapper sideWidget = new ImIntegerWrapper(actionDescription::getSide, actionDescription::setSide, labels.get("Side"));
   private final String[] handConfigurationNames = new String[HandConfiguration.values.length];
   private final ImIntegerWrapper handConfigurationIndex = new ImIntegerWrapper(actionDescription::getHandConfigurationIndex,
                                                                                actionDescription::setHandConfigurationIndex,
                                                                                imInt -> ImGui.combo(labels.get("Grip"),
                                                                                                     imInt,
                                                                                                     handConfigurationNames));
   private final ImBooleanWrapper executeWithNextActionWrapper = new ImBooleanWrapper(actionDescription::getExecuteWithNextAction,
                                                                                      actionDescription::setExecuteWithNextAction,
                                                                                      imBoolean -> imgui.ImGui.checkbox(labels.get("Execute with next action"), imBoolean));

   public RDXHandConfigurationAction()
   {
      HandConfiguration[] values = HandConfiguration.values;
      for (int i = 0; i < values.length; i++)
      {
         handConfigurationNames[i] = values[i].name();
      }
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      imgui.ImGui.sameLine();
      executeWithNextActionWrapper.renderImGuiWidget();
      ImGui.pushItemWidth(100.0f);
      sideWidget.renderImGuiWidget();
      handConfigurationIndex.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public HandConfigurationActionDescription getActionDescription()
   {
      return actionDescription;
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Hand Configuration";
   }
}
