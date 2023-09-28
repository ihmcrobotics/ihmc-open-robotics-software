package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.ArmJointAnglesAction;
import us.ihmc.behaviors.sequence.actions.ArmJointAnglesActionDescription;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

public class RDXArmJointAnglesAction extends RDXBehaviorAction
{
   private final DRCRobotModel robotModel;
   private final ArmJointAnglesActionDescription actionDescription = new ArmJointAnglesActionDescription();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImIntegerWrapper sideWidget = new ImIntegerWrapper(actionDescription::getSide, actionDescription::setSide, labels.get("Side"));
   private final String[] configurations = new String[PresetArmConfiguration.values().length + 1];
   private final ImInt currentConfiguration = new ImInt(PresetArmConfiguration.HOME.ordinal() + 1);
   private final ImDoubleWrapper[] jointAngleWidgets = new ImDoubleWrapper[ArmJointAnglesAction.NUMBER_OF_JOINTS];
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(actionDescription::getTrajectoryDuration,
                                                                                actionDescription::setTrajectoryDuration,
                                                                                imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));
   public RDXArmJointAnglesAction(DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;
      int c = 0;
      configurations[c++] = ArmJointAnglesAction.CUSTOM_ANGLES_NAME;
      for (PresetArmConfiguration preset : PresetArmConfiguration.values())
      {
         configurations[c++] = preset.name();
      }

      for (int i = 0; i < ArmJointAnglesAction.NUMBER_OF_JOINTS; i++)
      {
         int jointIndex = i;
         jointAngleWidgets[i] = new ImDoubleWrapper(() -> actionDescription.getJointAngles()[jointIndex],
                                                    jointAngle -> actionDescription.getJointAngles()[jointIndex] = jointAngle,
                                                    imDouble -> ImGui.inputDouble(labels.get("j" + jointIndex), imDouble));
      }
   }

   @Override
   public void update(boolean executeWithPreviousAction)
   {
      PresetArmConfiguration preset = actionDescription.getPreset();
      currentConfiguration.set(preset == null ? 0 : preset.ordinal() + 1);

      // Copy the preset values into the custom data fields so they can be tweaked
      // relatively when switching to custom angles.
      if (preset != null)
         robotModel.getPresetArmConfiguration(actionDescription.getSide(), preset, actionDescription.getJointAngles());
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
         actionDescription.setPreset(currentConfiguration.get() == 0 ? null : PresetArmConfiguration.values()[currentConfiguration.get() - 1]);
      ImGui.popItemWidth();

      if (actionDescription.getPreset() == null)
      {
         ImGui.pushItemWidth(80.0f);
         for (int i = 0; i < ArmJointAnglesAction.NUMBER_OF_JOINTS; i++)
         {
            jointAngleWidgets[i].renderImGuiWidget();
         }
         ImGui.popItemWidth();
      }
   }

   public ArmJointAnglesActionDescription getActionDescription()
   {
      return actionDescription;
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Arm Joint Angles";
   }
}
