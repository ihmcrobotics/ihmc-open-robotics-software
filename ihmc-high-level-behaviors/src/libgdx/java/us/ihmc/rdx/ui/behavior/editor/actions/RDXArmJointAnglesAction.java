package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.behaviors.sequence.actions.ArmJointAnglesAction;
import us.ihmc.behaviors.sequence.actions.ArmJointAnglesActionData;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

public class RDXArmJointAnglesAction extends RDXBehaviorAction
{
   private final ArmJointAnglesActionData actionData = new ArmJointAnglesActionData();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImIntegerWrapper sideWidget = new ImIntegerWrapper(actionData::getSide, actionData::setSide, labels.get("Side"));
   private final String[] configurations = new String[PresetArmConfiguration.values().length + 1];
   private final ImInt currentConfiguration = new ImInt(PresetArmConfiguration.HOME.ordinal() + 1);
   private final ImDoubleWrapper[] jointAngleWidgets = new ImDoubleWrapper[ArmJointAnglesAction.NUMBER_OF_JOINTS];
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(actionData::getTrajectoryDuration,
                                                                                actionData::setTrajectoryDuration,
                                                                                imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));
   public RDXArmJointAnglesAction()
   {
      int c = 0;
      configurations[c++] = ArmJointAnglesAction.CUSTOM_ANGLES_NAME;
      for (PresetArmConfiguration preset : PresetArmConfiguration.values())
      {
         configurations[c++] = preset.name();
      }

      for (int i = 0; i < ArmJointAnglesAction.NUMBER_OF_JOINTS; i++)
      {
         int jointIndex = i;
         jointAngleWidgets[i] = new ImDoubleWrapper(() -> actionData.getJointAngles()[jointIndex],
                                                    jointAngle -> actionData.getJointAngles()[jointIndex] = jointAngle,
                                                    imDouble -> ImGui.inputDouble(labels.get("j" + jointIndex), imDouble));
      }
   }

   @Override
   public void update()
   {
      currentConfiguration.set(actionData.getPreset() == null ? 0 : actionData.getPreset().ordinal() + 1);
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(100.0f);
      sideWidget.renderImGuiWidget();
      ImGui.popItemWidth();
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();

      ImGui.pushItemWidth(200.0f);
      if (ImGui.combo(labels.get("Configuration"), currentConfiguration, configurations))
         actionData.setPreset(currentConfiguration.get() == 0 ? null : PresetArmConfiguration.values()[currentConfiguration.get() - 1]);
      ImGui.popItemWidth();

      if (actionData.getPreset() == null)
      {
         ImGui.pushItemWidth(80.0f);
         for (int i = 0; i < ArmJointAnglesAction.NUMBER_OF_JOINTS; i++)
         {
            jointAngleWidgets[i].renderImGuiWidget();
         }
         ImGui.popItemWidth();
      }
   }

   public ArmJointAnglesActionData getActionData()
   {
      return actionData;
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Arm Joint Angles";
   }
}
