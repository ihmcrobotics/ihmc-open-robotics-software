package us.ihmc.gdx.ui.teleoperation;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.tools.GDXIconTexture;
import us.ihmc.gdx.ui.GDX3DPanelToolbarButton;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class GDXHandConfigurationManager
{
   private CommunicationHelper communicationHelper;
   private final SideDependentList<ImInt> handConfigurationIndices = new SideDependentList<>(new ImInt(6), new ImInt(6));
   private final String[] handConfigurationNames = new String[HandConfiguration.values.length];
   private final SideDependentList<GDXIconTexture> handIcons = new SideDependentList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RobotSide toolbarSelectedSide = RobotSide.LEFT;

   public void create(GDXImGuiBasedUI baseUI, CommunicationHelper communicationHelper)
   {
      this.communicationHelper = communicationHelper;

      for (RobotSide side : RobotSide.values)
      {
         handIcons.put(side, new GDXIconTexture("icons/" + side.getLowerCaseName() + "Hand.png"));
      }
      HandConfiguration[] values = HandConfiguration.values;
      for (int i = 0; i < values.length; i++)
      {
         handConfigurationNames[i] = values[i].name();
      }

      SideDependentList<GDXIconTexture> toggleIcons = new SideDependentList<>(new GDXIconTexture("icons/leftToggle.jpg"),
                                                                              new GDXIconTexture("icons/rightToggle.jpg"));
      GDX3DPanelToolbarButton leftRightHandToggleButton = baseUI.getPrimary3DPanel().addToolbarButton();
      leftRightHandToggleButton.setTooltipText("Toggle hand. Red: Left | Green: Right");
      leftRightHandToggleButton.setIcon(toggleIcons.get(toolbarSelectedSide));
      leftRightHandToggleButton.setOnPressed(() ->
      {
         toolbarSelectedSide = toolbarSelectedSide.getOppositeSide();
         leftRightHandToggleButton.setIcon(toggleIcons.get(toolbarSelectedSide));
      });

      SideDependentList<Runnable> calibrateCommands = new SideDependentList<>(() -> publishHandCommand(RobotSide.LEFT, HandConfiguration.CALIBRATE),
                                                                              () -> publishHandCommand(RobotSide.RIGHT, HandConfiguration.CALIBRATE));
      GDX3DPanelToolbarButton calibrateButton = baseUI.getPrimary3DPanel().addToolbarButton();
      calibrateButton.loadAndSetIcon("icons/calibrate.png");
      calibrateButton.setTooltipText("Calibrate hand");
      calibrateButton.setOnPressed(() -> calibrateCommands.get(toolbarSelectedSide).run());

      SideDependentList<Runnable> openCommands = new SideDependentList<>(() -> publishHandCommand(RobotSide.LEFT, HandConfiguration.OPEN),
                                                                          () -> publishHandCommand(RobotSide.RIGHT, HandConfiguration.OPEN));
      GDX3DPanelToolbarButton openHandButton = baseUI.getPrimary3DPanel().addToolbarButton();
      openHandButton.loadAndSetIcon("icons/openGripper.jpg");
      openHandButton.setTooltipText("Open hand");
      openHandButton.setOnPressed(() -> openCommands.get(toolbarSelectedSide).run());

      SideDependentList<Runnable> closeCommands = new SideDependentList<>(() -> publishHandCommand(RobotSide.LEFT, HandConfiguration.CLOSE),
                                                                          () -> publishHandCommand(RobotSide.RIGHT, HandConfiguration.CLOSE));
      GDX3DPanelToolbarButton closeHandButton = baseUI.getPrimary3DPanel().addToolbarButton();
      closeHandButton.loadAndSetIcon("icons/closeGripper.jpg");
      closeHandButton.setTooltipText("Close hand");
      closeHandButton.setOnPressed(() -> closeCommands.get(toolbarSelectedSide).run());
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

   private void publishHandCommand(RobotSide side, HandConfiguration handDesiredConfiguration)
   {
      communicationHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                  HumanoidMessageTools.createHandDesiredConfigurationMessage(side, handDesiredConfiguration));
   }
}
