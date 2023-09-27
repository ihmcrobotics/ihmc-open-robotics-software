package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.flag.ImGuiCol;
import imgui.internal.ImGui;
import us.ihmc.behaviors.sequence.BehaviorActionDescription;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionDescription;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

import static us.ihmc.avatar.sakeGripper.SakeHandParameters.*;

public class RDXSakeHandCommandAction extends RDXBehaviorAction
{
   private final SakeHandCommandActionDescription actionData = new SakeHandCommandActionDescription();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImIntegerWrapper sideWidget = new ImIntegerWrapper(actionData::getSide, actionData::setSide, labels.get("Side"));

   private final String[] handConfigurationNames = new String[SakeCommandOption.values.length];
   private final ImIntegerWrapper handCommandOptionIndex = new ImIntegerWrapper(actionData::getHandConfigurationIndex,
                                                                                actionData::setHandConfigurationIndex,
                                                                                imInt -> ImGui.combo(labels.get("Predefined Options"), imInt, handConfigurationNames));
   private final float[] positionValue = {0.0f};
   private final float[] torqueValue = {0.0f};

   private final ImBooleanWrapper executeWithNextActionWrapper = new ImBooleanWrapper(actionData::getExecuteWithNextAction,
                                                                                      actionData::setExecuteWithNextAction,
                                                                                      imBoolean -> imgui.ImGui.checkbox(labels.get("Execute with next action"), imBoolean));

   public RDXSakeHandCommandAction()
   {
      for (int i = 0; i < SakeCommandOption.values.length; ++i)
      {
         handConfigurationNames[i] = SakeCommandOption.values[i].name();
      }
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      imgui.ImGui.sameLine();
      executeWithNextActionWrapper.renderImGuiWidget();
      
      ImGui.pushItemWidth(100.0f);
      sideWidget.renderImGuiWidget();
      ImGui.sameLine();
      handCommandOptionIndex.renderImGuiWidget();
      ImGui.popItemWidth();

      // Set position and torque slider values to match selected command option
      SakeCommandOption command = SakeCommandOption.values[actionData.getHandConfigurationIndex()];
      if (command.getGoalPosition() >= 0)
      {
         positionValue[0] = (float) (command.getGoalPosition() * Math.toRadians(MAX_ANGLE_BETWEEN_FINGERS));
      }
      if (command.getGoalTorque() >= 0)
      {
         torqueValue[0] = (float) command.getGoalTorque();
      }

      float lastPositionValue = positionValue[0];
      float lastTorqueValue = torqueValue[0];

      ImGui.sliderAngle(labels.get("Angle Between Fingers"), positionValue, 0.0f, MAX_ANGLE_BETWEEN_FINGERS);
      ImGui.pushStyleColor(ImGuiCol.SliderGrab, ImGuiTools.greenToRedGradiatedColor(torqueValue[0], 0.5, 0.7, 0.9));
      ImGui.sliderFloat(labels.get("Goal Torque"), torqueValue, 0.0f, 1.0f, String.format("%.1f N", (torqueValue[0] * MAX_TORQUE_NEWTONS)));
      ImGui.popStyleColor();

      // if user moved slider, set command to GOTO
      if (positionValue[0] != lastPositionValue || torqueValue[0] != lastTorqueValue)
      {
         actionData.setHandConfigurationIndex(SakeCommandOption.GOTO.ordinal());
      }
   }

   @Override
   public SakeHandCommandActionDescription getActionDescription()
   {
      actionData.setGoalPosition(positionValue[0] / Math.toRadians(MAX_ANGLE_BETWEEN_FINGERS));
      actionData.setGoalTorque(torqueValue[0]);

      return actionData;
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Hand Configuration";
   }
}
