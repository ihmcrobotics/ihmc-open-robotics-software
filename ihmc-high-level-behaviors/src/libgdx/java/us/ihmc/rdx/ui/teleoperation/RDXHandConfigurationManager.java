package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.sakeHandCommunication.ROS2SakeHandCommander;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.ROS2Tools;
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
   private ROS2SakeHandCommander handCommander;
   private final SideDependentList<RDXIconTexture> handIcons = new SideDependentList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final SideDependentList<RDXSakeHandInformation> sakeHandInfo = new SideDependentList<>();
   private final SideDependentList<RDXHandQuickAccessButtons> handQuickAccessButtons = new SideDependentList<>();
   private final SideDependentList<RDXSakeHandPositionSlider> handPositionSliders = new SideDependentList<>();
   private final SideDependentList<RDXSakeHandTorqueSlider> handTorqueSliders = new SideDependentList<>();

   public void create(RDXBaseUI baseUI, CommunicationHelper communicationHelper, ROS2SyncedRobotModel syncedRobotModel)
   {
      this.communicationHelper = communicationHelper;
      handCommander = ROS2SakeHandCommander.getSakeHandCommander(communicationHelper);

      for (RobotSide side : RobotSide.values)
      {
         handIcons.put(side, new RDXIconTexture("icons/" + side.getLowerCaseName() + "Hand.png"));

         Runnable openHand = () -> handCommander.sendPredefinedCommand(side, SakeHandCommandOption.OPEN);
         Runnable closeHand = () -> handCommander.sendPredefinedCommand(side, SakeHandCommandOption.CLOSE);
         Runnable calibrateHand = () -> handCommander.sendPredefinedCommand(side, SakeHandCommandOption.CALIBRATE);
         Runnable resetHand = () -> handCommander.sendPredefinedCommand(side, SakeHandCommandOption.RESET);
         handQuickAccessButtons.put(side, new RDXHandQuickAccessButtons(baseUI, side, openHand, closeHand, calibrateHand, resetHand));

         handPositionSliders.put(side, new RDXSakeHandPositionSlider(communicationHelper, side));
         handTorqueSliders.put(side, new RDXSakeHandTorqueSlider(syncedRobotModel, communicationHelper, side));
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

   public void update()
   {
      for (RobotSide side : sakeHandInfo.sides())
      {
         handQuickAccessButtons.get(side).update(sakeHandInfo.get(side));
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
            handCommander.sendPredefinedCommand(side, SakeHandCommandOption.CALIBRATE);
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Open", side.getCamelCaseName())))
         {
            handCommander.sendPredefinedCommand(side, SakeHandCommandOption.OPEN);
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Close", side.getCamelCaseName())))
         {
            handCommander.sendPredefinedCommand(side, SakeHandCommandOption.CLOSE);
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Grip", side.getCamelCaseName())))
         {
            handCommander.sendPredefinedCommand(side, SakeHandCommandOption.GRIP);
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Reset", side.getCamelCaseName())))
         {
            handCommander.sendPredefinedCommand(side, SakeHandCommandOption.RESET);
            handCommander.sendErrorConfirmation(side);
         }

         handPositionSliders.get(side).renderImGuiWidgets();
         handTorqueSliders.get(side).renderImGuiWidgets();
      }
      if (!sakeHandInfo.isEmpty())
         ImGui.text("Sake EZGrippers:");
      for (RobotSide side : sakeHandInfo.sides())
      {
         sakeHandInfo.get(side).renderImGuiWidgets();
      }
   }
}
