package us.ihmc.rdx.ui.teleoperation;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXSakeHandWidgets;
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
   private final SideDependentList<RDXHandQuickAccessButtons> handQuickAccessButtons = new SideDependentList<>();
   private final SideDependentList<RDXSakeHandWidgets> sakeHandWidgets = new SideDependentList<>();

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

         Runnable openHand = () -> publishHandCommand(side, HandConfiguration.OPEN);
         Runnable closeHand = () -> publishHandCommand(side, HandConfiguration.CLOSE);
         Runnable calibrateHand = () -> publishHandCommand(side, HandConfiguration.CALIBRATE);
         Runnable resetHand = () -> publishHandCommand(side, HandConfiguration.RESET);
         handQuickAccessButtons.put(side, new RDXHandQuickAccessButtons(baseUI, side, openHand, closeHand, calibrateHand, resetHand));

         sakeHandWidgets.put(side, new RDXSakeHandWidgets(communicationHelper, side));
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
         sakeHandInfo.get(side).update();
         handQuickAccessButtons.get(side).update(sakeHandInfo.get(side));
         sakeHandWidgets.get(side).update();
      }
   }

   public void renderImGuiWidgets()
   {
      for (RobotSide side : RobotSide.values)
      {
         ImGui.image(handIcons.get(side).getTexture().getTextureObjectHandle(), 22.0f, 22.0f);
         ImGui.sameLine();
//         if (ImGui.button(labels.get("Calibrate", side.getCamelCaseName())))
//         {
//            publishHandCommand(side, HandConfiguration.CALIBRATE);
//         }
//         ImGui.sameLine();
//         if (ImGui.button(labels.get("Open", side.getCamelCaseName())))
//         {
//            publishHandCommand(side, HandConfiguration.OPEN);
//         }
//         ImGui.sameLine();
//         if (ImGui.button(labels.get("Close", side.getCamelCaseName())))
//         {
//            publishHandCommand(side, HandConfiguration.CLOSE);
//         }
//         ImGui.sameLine();
//         ImGui.pushItemWidth(100.0f);
//         ImGui.combo(labels.get("Grip", side.getCamelCaseName()), handConfigurationIndices.get(side), handConfigurationNames);
//         ImGui.popItemWidth();
//         ImGui.sameLine();
//         if (ImGui.button(labels.get("Send", side.getCamelCaseName())))
//         {
//            HandDesiredConfigurationMessage message
//                  = HumanoidMessageTools.createHandDesiredConfigurationMessage(side, HandConfiguration.values[handConfigurationIndices.get(side).get()]);
//            communicationHelper.publish(ROS2Tools::getHandConfigurationTopic, message);
//         }

         sakeHandWidgets.get(side).renderImGuiWidgets();
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
