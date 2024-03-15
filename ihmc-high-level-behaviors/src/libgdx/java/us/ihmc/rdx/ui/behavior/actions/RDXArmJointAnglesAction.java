package us.ihmc.rdx.ui.behavior.actions;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.ArmJointAnglesActionDefinition;
import us.ihmc.behaviors.sequence.actions.ArmJointAnglesActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXArmJointAnglesAction extends RDXActionNode<ArmJointAnglesActionState, ArmJointAnglesActionDefinition>
{
   private final DRCRobotModel robotModel;
   private final ArmJointAnglesActionState state;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImIntegerWrapper sideWidget;
   private final String[] configurations = new String[PresetArmConfiguration.values().length + 1];
   private final ImInt currentConfiguration = new ImInt(PresetArmConfiguration.HOME.ordinal() + 1);
   private final ImDoubleWrapper[] jointAngleWidgets = new ImDoubleWrapper[ArmJointAnglesActionDefinition.MAX_NUMBER_OF_JOINTS];
   private final ImDoubleWrapper trajectoryDurationWidget;

   public RDXArmJointAnglesAction(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, DRCRobotModel robotModel)
   {
      super(new ArmJointAnglesActionState(id, crdtInfo, saveFileDirectory));

      state = getState();

      this.robotModel = robotModel;

      getDefinition().setName("Arm joint angles");

      sideWidget = new ImIntegerWrapper(getDefinition()::getSide, getDefinition()::setSide, labels.get("Side"));
      trajectoryDurationWidget = new ImDoubleWrapper(getDefinition()::getTrajectoryDuration,
                                                     getDefinition()::setTrajectoryDuration,
                                                     imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));

      int c = 0;
      configurations[c++] = ArmJointAnglesActionDefinition.CUSTOM_ANGLES_NAME;
      for (PresetArmConfiguration preset : PresetArmConfiguration.values())
      {
         configurations[c++] = preset.name();
      }

      for (int i = 0; i < ArmJointAnglesActionDefinition.MAX_NUMBER_OF_JOINTS; i++)
      {
         int jointIndex = i;
         jointAngleWidgets[i] = new ImDoubleWrapper(() -> getDefinition().getJointAngles().getValue()[jointIndex],
                                                    jointAngle -> getDefinition().getJointAngles().getValue()[jointIndex] = jointAngle,
                                                    imDouble -> ImGui.inputDouble(labels.get("j" + jointIndex), imDouble));
      }
   }

   @Override
   public void update()
   {
      super.update();

      PresetArmConfiguration preset = getDefinition().getPreset();
      currentConfiguration.set(preset == null ? 0 : preset.ordinal() + 1);

      // Copy the preset values into the custom data fields so they can be tweaked
      // relatively when switching to custom angles.
      if (preset != null)
      {
         // TODO: Would be great if there was a #getPresetArmConfiguration that accepts an array to pack
         double[] jointAngles = robotModel.getPresetArmConfiguration(getDefinition().getSide(), preset);
         for (int i = 0; i < jointAngles.length; i++)
         {
            getDefinition().getJointAngles().getValue()[i] = jointAngles[i];
         }
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.pushItemWidth(100.0f);
      sideWidget.renderImGuiWidget();
      ImGui.popItemWidth();
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();

      ImGui.pushItemWidth(200.0f);
      if (ImGui.combo(labels.get("Configuration"), currentConfiguration, configurations))
         getDefinition().setPreset(currentConfiguration.get() == 0 ? null : PresetArmConfiguration.values()[currentConfiguration.get() - 1]);
      ImGui.popItemWidth();

      if (getDefinition().getPreset() == null)
      {
         ImGui.pushItemWidth(80.0f);
         for (int i = 0; i < ArmJointAnglesActionDefinition.MAX_NUMBER_OF_JOINTS; i++)
         {
            jointAngleWidgets[i].renderImGuiWidget();
         }
         ImGui.popItemWidth();
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Arm Joint Angles";
   }
}
