package us.ihmc.gdx.ui.teleoperation;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.tools.GDXIconTexture;
import us.ihmc.gdx.ui.affordances.GDXWholeBodyInteractable;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceDirectory;

public class GDXHandManager
{
   private CommunicationHelper communicationHelper;
   private final SideDependentList<ImInt> handConfigurationIndices = new SideDependentList<>(new ImInt(6), new ImInt(6));
   private final String[] handConfigurationNames = new String[HandConfiguration.values.length];

   private final WorkspaceDirectory iconDirectory = new WorkspaceDirectory("ihmc-open-robotics-software",
                                                                           "ihmc-high-level-behaviors/src/libgdx/resources/icons");
   private final SideDependentList<GDXIconTexture> handIcons = new SideDependentList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public void create(CommunicationHelper communicationHelper)
   {
      this.communicationHelper = communicationHelper;
      for (RobotSide side : RobotSide.values)
      {
         handIcons.put(side, new GDXIconTexture(iconDirectory.file(side.getLowerCaseName() + "Hand.png")));
      }
      HandConfiguration[] values = HandConfiguration.values;
      for (int i = 0; i < values.length; i++)
      {
         handConfigurationNames[i] = values[i].name();
      }
   }

   private void publishHandCommand(RobotSide side, HandConfiguration handDesiredConfiguration)
   {
      communicationHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                  HumanoidMessageTools.createHandDesiredConfigurationMessage(side, handDesiredConfiguration));
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
   }
}
