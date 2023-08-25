package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.*;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.SakeHandCommandOption;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXSakeHandPositionSlider;
import us.ihmc.rdx.ui.interactable.RDXSakeHandTorqueSlider;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Manages the UI for a humanoid robot's hands. A hand configuration is like "open", "closed", etc.
 */
public class RDXHandConfigurationManager
{
   private CommunicationHelper communicationHelper;
   private final SideDependentList<RDXIconTexture> handIcons = new SideDependentList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final SideDependentList<RDXSakeHandInformation> sakeHandInfo = new SideDependentList<>();
   private final SideDependentList<RDXHandQuickAccessButtons> handQuickAccessButtons = new SideDependentList<>();
   private final SideDependentList<RDXSakeHandPositionSlider> handPositionSliders = new SideDependentList<>();
   private final SideDependentList<RDXSakeHandTorqueSlider> handTorqueSliders = new SideDependentList<>();

   public void create(RDXBaseUI baseUI, CommunicationHelper communicationHelper, ROS2SyncedRobotModel syncedRobotModel)
   {
      this.communicationHelper = communicationHelper;


      for (RobotSide side : RobotSide.values)
      {
         handIcons.put(side, new RDXIconTexture("icons/" + side.getLowerCaseName() + "Hand.png"));

         Runnable openHand = () -> publishHandCommand(side, SakeHandCommandOption.OPEN);
         Runnable closeHand = () -> publishHandCommand(side, SakeHandCommandOption.CLOSE);
         handQuickAccessButtons.put(side, new RDXHandQuickAccessButtons(baseUI, side, openHand, closeHand));

         handPositionSliders.put(side, new RDXSakeHandPositionSlider(syncedRobotModel, communicationHelper, side));
         handTorqueSliders.put(side, new RDXSakeHandTorqueSlider(communicationHelper, side));
      }

      if (syncedRobotModel.getRobotModel().getHandModels().toString().contains("SakeHand"))
         setupForSakeHands();
   }

   private void setupForSakeHands()
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
         if (ImGui.button(labels.get("Close", side.getCamelCaseName())))
         {
            publishHandCommand(side, SakeHandCommandOption.CLOSE);
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Open", side.getCamelCaseName())))
         {
            publishHandCommand(side, SakeHandCommandOption.OPEN);
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Grip", side.getCamelCaseName())))
         {
            publishHandCommand(side, SakeHandCommandOption.GRIP);
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Calibrate", side.getCamelCaseName())))
         {
            publishHandCommand(side, SakeHandCommandOption.CALIBRATE);
         }

         sakeHandInfo.get(side).renderImGuiWidgets();

         handPositionSliders.get(side).renderImGuiWidgets();
         handTorqueSliders.get(side).renderImGuiWidgets();

         ImGui.separator();
      }
   }

   public void publishHandCommand(RobotSide side, SakeHandCommandOption handCommandOption)
   {
      communicationHelper.publish(ROS2Tools::getHandSakeCommandTopic,
                                  HumanoidMessageTools.createHandSakeDesiredCommandMessage(side, handCommandOption, 0.0, 0.0));
   }
}
