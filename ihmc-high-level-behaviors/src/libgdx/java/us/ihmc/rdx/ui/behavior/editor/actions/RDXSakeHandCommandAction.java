package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.flag.ImGuiCol;
import imgui.internal.ImGui;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionData;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

import static us.ihmc.avatar.sakeGripper.SakeHandParameters.*;

public class RDXSakeHandCommandAction extends RDXBehaviorAction
{
   private final SakeHandCommandActionData actionData = new SakeHandCommandActionData();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImIntegerWrapper sideWidget = new ImIntegerWrapper(actionData::getSide, actionData::setSide, labels.get("Side"));
   private float[] positionValue = {0.0f};
   private float[] torqueValue = {0.0f};

   public RDXSakeHandCommandAction()
   {
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(100.0f);
      sideWidget.renderImGuiWidget();
      ImGui.popItemWidth();

      ImGui.sliderAngle(labels.get("Angle Between Fingers"), positionValue, 0.0f, MAX_ANGLE_BETWEEN_FINGERS);
      ImGui.pushStyleColor(ImGuiCol.SliderGrab, ImGuiTools.greenToRedGradiatedColor(torqueValue[0], 0.5, 0.7, 0.9));
      ImGui.sliderFloat(labels.get("Goal Torque"), torqueValue, 0.0f, 1.0f, String.format("%.1f N", (torqueValue[0] * MAX_TORQUE_NEWTONS)));
      ImGui.popStyleColor();
   }

   @Override
   public SakeHandCommandActionData getActionData()
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
