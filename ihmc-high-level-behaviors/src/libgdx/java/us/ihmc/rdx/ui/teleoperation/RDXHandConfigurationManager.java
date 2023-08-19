package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.*;
import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Manages the UI for a humanoid robot's hands. A hand configuration is like "open", "closed", etc.
 */
public class RDXHandConfigurationManager
{
   private CommunicationHelper communicationHelper;
   private final SideDependentList<ImInt> handConfigurationIndices = new SideDependentList<>(new ImInt(9), new ImInt(9));
   private final String[] handConfigurationNames = new String[HandConfiguration.values.length];
   private final SideDependentList<RDXIconTexture> handIcons = new SideDependentList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final SideDependentList<RDXSakeHandInformation> sakeHandInfo = new SideDependentList<>();

   public void create(RDXBaseUI baseUI, CommunicationHelper communicationHelper, ROS2SyncedRobotModel syncedRobotModel)
   {
      this.communicationHelper = communicationHelper;

      HandConfiguration[] values = HandConfiguration.values;
      for (int i = 0; i < values.length; i++)
      {
         handConfigurationNames[i] = values[i].name();
      }

      for (RobotSide side : RobotSide.values)
      {
         handIcons.put(side, new RDXIconTexture("icons/" + side.getLowerCaseName() + "Hand.png"));

         RDX3DPanelToolbarButton calibrateButton = baseUI.getPrimary3DPanel().addToolbarButton();
         calibrateButton.loadAndSetIcon("icons/calibrate.png");
         calibrateButton.setTooltipText("Calibrate %s hand".formatted(side.getLowerCaseName()));
         calibrateButton.setOnPressed(() -> publishHandCommand(side, HandConfiguration.CALIBRATE));

         RDX3DPanelToolbarButton openHandButton = baseUI.getPrimary3DPanel().addToolbarButton();
         openHandButton.loadAndSetIcon("icons/openGripper.png");
         openHandButton.setTooltipText("Open %s hand".formatted(side.getLowerCaseName()));
         openHandButton.setOnPressed(() -> publishHandCommand(side, HandConfiguration.OPEN));

         RDX3DPanelToolbarButton closeHandButton = baseUI.getPrimary3DPanel().addToolbarButton();
         closeHandButton.loadAndSetIcon("icons/closeGripper.png");
         closeHandButton.setTooltipText("Close %s hand".formatted(side.getLowerCaseName()));
         closeHandButton.setOnPressed(() -> publishHandCommand(side, HandConfiguration.CLOSE));
      }

      if (syncedRobotModel.getRobotModel().getHandModels().toString().contains("SakeHand"))
         setupForSakeHands();
   }

   public void publishArmHomeCommand(RobotSide side)
   {
      double trajectoryTime = 3.5;
      GoHomeMessage homeArm = new GoHomeMessage();
      homeArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeArm.setRobotSide(side.toByte());
      homeArm.setTrajectoryTime(trajectoryTime);
      communicationHelper.publishToController(homeArm);
   }

   public void setupForSakeHands()
   {
      for (RobotSide side : RobotSide.values)
      {
         sakeHandInfo.put(side, new RDXSakeHandInformation(side, communicationHelper));
      }
   }

   public void renderImGuiWidgets()
   {
      for (RobotSide side : RobotSide.values)
      {
         ImGui.image(handIcons.get(side).getTexture().getTextureObjectHandle(), 22.0f, 22.0f);
         ImGui.sameLine();
         if (ImGui.button(labels.get("Calibrate", side.getCamelCaseName())))
         {
            publishHandCommand(side, HandConfiguration.CALIBRATE);
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Open", side.getCamelCaseName())))
         {
            publishHandCommand(side, HandConfiguration.OPEN);
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Close", side.getCamelCaseName())))
         {
            publishHandCommand(side, HandConfiguration.CLOSE);
         }
         ImGui.sameLine();
         ImGui.pushItemWidth(100.0f);
         ImGui.combo(labels.get("Grip", side.getCamelCaseName()), handConfigurationIndices.get(side), handConfigurationNames);
         ImGui.popItemWidth();
         ImGui.sameLine();
         if (ImGui.button(labels.get("Send", side.getCamelCaseName())))
         {
            HandDesiredConfigurationMessage message
                  = HumanoidMessageTools.createHandDesiredConfigurationMessage(side, HandConfiguration.values[handConfigurationIndices.get(side).get()]);
            communicationHelper.publish(ROS2Tools::getHandConfigurationTopic, message);
         }
      }
      if (!sakeHandInfo.isEmpty())
         ImGui.text("Sake EZGrippers:");
      for (RobotSide side : sakeHandInfo.sides())
      {
         sakeHandInfo.get(side).renderImGuiWidgets();
      }
   }

   public void publishHandCommand(RobotSide side, HandConfiguration handDesiredConfiguration)
   {
      communicationHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                  HumanoidMessageTools.createHandDesiredConfigurationMessage(side, handDesiredConfiguration));
   }
}
