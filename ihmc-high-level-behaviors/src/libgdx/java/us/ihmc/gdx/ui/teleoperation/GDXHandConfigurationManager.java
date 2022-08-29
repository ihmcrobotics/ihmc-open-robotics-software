package us.ihmc.gdx.ui.teleoperation;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.tools.GDXIconTexture;
import us.ihmc.gdx.tools.GDXToolButton;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.util.ArrayList;
import java.util.Arrays;

public class GDXHandConfigurationManager
{
   private CommunicationHelper communicationHelper;
   private final SideDependentList<ImInt> handConfigurationIndices = new SideDependentList<>(new ImInt(6), new ImInt(6));
   private final String[] handConfigurationNames = new String[HandConfiguration.values.length];
   private final SideDependentList<GDXIconTexture> handIcons = new SideDependentList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public void create(GDXImGuiBasedUI baseUI, WorkspaceDirectory iconDirectory, CommunicationHelper communicationHelper)
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

      GDXToolButton button;
      // TOGGLING.
      ArrayList<String> fileNames = new ArrayList<>(Arrays.asList("leftToggle.jpg", "rightToggle.jpg"));
      button = new GDXToolButton("leftRightToggleButton", iconDirectory, fileNames, null, true, false);
      button.setToolTipText("toggle: red - left | green - right");
      baseUI.getPrimary3DPanel().addHotButton(button);

      // CALIBRATING
      ArrayList<Runnable> calibrateRunnables = new ArrayList<>(Arrays.asList(() -> publishHandCommand(RobotSide.LEFT, HandConfiguration.CALIBRATE),
                                                                             () -> publishHandCommand(RobotSide.RIGHT, HandConfiguration.CALIBRATE)));
      button = new GDXToolButton("calibrateButton", iconDirectory, "calibrate.png", calibrateRunnables, false, true);
      button.setToolTipText("action: Calibrate");
      baseUI.getPrimary3DPanel().addHotButton(button);

      // OPEN, CLOSE
      ArrayList<Runnable> openRunnables = new ArrayList<>(Arrays.asList(() -> publishHandCommand(RobotSide.LEFT, HandConfiguration.OPEN),
                                                                        () -> publishHandCommand(RobotSide.RIGHT, HandConfiguration.OPEN)));
      ArrayList<Runnable> closeRunnables = new ArrayList<>(Arrays.asList(() -> publishHandCommand(RobotSide.LEFT, HandConfiguration.CLOSE),
                                                                         () -> publishHandCommand(RobotSide.RIGHT, HandConfiguration.CLOSE)));
      button = new GDXToolButton("openGripperButton", iconDirectory, "openGripper.jpg", openRunnables, false, true);
      button.setToolTipText("action: OPEN gripper based on toggle (left, right)");
      baseUI.getPrimary3DPanel().addHotButton(button);
      button = new GDXToolButton("closeGripperButton", iconDirectory, "closeGripper.jpg", closeRunnables, false, true);
      button.setToolTipText("action: CLOSE gripper based on toggle (left, right)");
      baseUI.getPrimary3DPanel().addHotButton(button);
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
