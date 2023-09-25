package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.internal.ImGui;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionData;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

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

      // TODO: Get rid of magical 210.0
      ImGui.sliderAngle(labels.get("Finger Angle"), positionValue, 0.0f, 210.0f);
      ImGui.sliderFloat(labels.get("Goal Torque"), torqueValue, 0.0f, 1.0f);
   }

   @Override
   public SakeHandCommandActionData getActionData()
   {
      actionData.setGoalPosition(positionValue[0] / Math.toRadians(210.0));
      actionData.setGoalTorque(torqueValue[0]);

      return actionData;
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Hand Configuration";
   }
}
