package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.*;
import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.function.Consumer;
import static us.ihmc.robotics.robotSide.RobotSide.LEFT;
import static us.ihmc.robotics.robotSide.RobotSide.RIGHT;

/**
 * Manages the UI for a humanoid robot's hands. A hand configuration is like "open", "closed", etc.
 */
public class RDXHandConfigurationManager
{
   private static final boolean ADD_SHIELD_BUTTON = false;

   private CommunicationHelper communicationHelper;
   private final SideDependentList<ImInt> handConfigurationIndices = new SideDependentList<>(new ImInt(9), new ImInt(9));
   private final String[] handConfigurationNames = new String[HandConfiguration.values.length];
   private final SideDependentList<RDXIconTexture> handIcons = new SideDependentList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RobotSide toolbarSelectedSide = RobotSide.LEFT;
   private final SideDependentList<IHMCROS2Input<HandSakeStatusMessage>> sakeStatuses = new SideDependentList<>();

   public void create(RDXBaseUI baseUI, CommunicationHelper communicationHelper, ROS2SyncedRobotModel syncedRobotModel)
   {
      this.communicationHelper = communicationHelper;

      for (RobotSide side : RobotSide.values)
      {
         handIcons.put(side, new RDXIconTexture("icons/" + side.getLowerCaseName() + "Hand.png"));
      }
      HandConfiguration[] values = HandConfiguration.values;
      for (int i = 0; i < values.length; i++)
      {
         handConfigurationNames[i] = values[i].name();
      }

      SideDependentList<RDXIconTexture> toggleIcons = new SideDependentList<>();
      RDX3DPanelToolbarButton leftRightHandToggleButton = baseUI.getPrimary3DPanel().addToolbarButton();
      toggleIcons.set(LEFT, leftRightHandToggleButton.loadAndSetIcon("icons/leftToggle.png"));
      toggleIcons.set(RIGHT, leftRightHandToggleButton.loadAndSetIcon("icons/rightToggle.png"));
      leftRightHandToggleButton.setTooltipText("Toggle hand. Red: Left | Green: Right");
      leftRightHandToggleButton.setIconTexture(toggleIcons.get(toolbarSelectedSide));
      leftRightHandToggleButton.setOnPressed(() ->
      {
         toolbarSelectedSide = toolbarSelectedSide.getOppositeSide();
         leftRightHandToggleButton.setIconTexture(toggleIcons.get(toolbarSelectedSide));
      });

      SideDependentList<Runnable> calibrateCommands = new SideDependentList<>(() -> publishHandCommand(RobotSide.LEFT, HandConfiguration.CALIBRATE),
                                                                              () -> publishHandCommand(RobotSide.RIGHT, HandConfiguration.CALIBRATE));
      RDX3DPanelToolbarButton calibrateButton = baseUI.getPrimary3DPanel().addToolbarButton();
      calibrateButton.loadAndSetIcon("icons/calibrate.png");
      calibrateButton.setTooltipText("Calibrate hand");
      calibrateButton.setOnPressed(() -> calibrateCommands.get(toolbarSelectedSide).run());

      SideDependentList<Runnable> openCommands = new SideDependentList<>(() -> publishHandCommand(RobotSide.LEFT, HandConfiguration.OPEN),
                                                                          () -> publishHandCommand(RobotSide.RIGHT, HandConfiguration.OPEN));
      RDX3DPanelToolbarButton openHandButton = baseUI.getPrimary3DPanel().addToolbarButton();
      openHandButton.loadAndSetIcon("icons/openGripper.png");
      openHandButton.setTooltipText("Open hand");
      openHandButton.setOnPressed(() -> openCommands.get(toolbarSelectedSide).run());

      SideDependentList<Runnable> closeCommands = new SideDependentList<>(() -> publishHandCommand(RobotSide.LEFT, HandConfiguration.CLOSE),
                                                                          () -> publishHandCommand(RobotSide.RIGHT, HandConfiguration.CLOSE));
      RDX3DPanelToolbarButton closeHandButton = baseUI.getPrimary3DPanel().addToolbarButton();
      closeHandButton.loadAndSetIcon("icons/closeGripper.png");
      closeHandButton.setTooltipText("Close hand");
      closeHandButton.setOnPressed(() -> closeCommands.get(toolbarSelectedSide).run());

      RDX3DPanelToolbarButton armHomeButton = baseUI.getPrimary3DPanel().addToolbarButton();
      armHomeButton.loadAndSetIcon("icons/home.png");
      armHomeButton.setTooltipText("left/right arm home pose");
      armHomeButton.setOnPressed(() -> publishArmHomeCommand(toolbarSelectedSide));

      if (ADD_SHIELD_BUTTON)
      {
         setupShieldButton(baseUI, communicationHelper);
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
         sakeStatuses.put(side, communicationHelper.subscribe(ROS2Tools.getControllerOutputTopic(communicationHelper.getRobotName())
                                                                       .withTypeName(HandSakeStatusMessage.class),
                                                              message -> message.getRobotSide() == side.toByte()));
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
      if (!sakeStatuses.isEmpty())
         ImGui.text("Sake EZGrippers:");
      for (RobotSide side : sakeStatuses.sides())
      {
         ImGui.text(side.getPascalCaseName() + ":");
         ImGui.sameLine();
         IHMCROS2Input<HandSakeStatusMessage> status = sakeStatuses.get(side);
         if (status.hasReceivedFirstMessage())
         {
            ImGui.text("Calibrated: " + status.getLatest().getCalibrated());
            ImGui.sameLine();
            ImGui.text("Needs reset: " + status.getLatest().getNeedsReset());
            ImGui.sameLine();
            ImGui.text("Temperature: " + FormattingTools.getFormattedDecimal1D(status.getLatest().getTemperature()));
         }
         else
         {
            ImGui.text("No status received.");
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Reset", side.getCamelCaseName())))
         {
            HandSakeDesiredCommandMessage sakeCommand = new HandSakeDesiredCommandMessage();
            sakeCommand.setRobotSide(side.toByte());
            sakeCommand.setDesiredHandConfiguration(HandSakeDesiredCommandMessage.HAND_CONFIGURATION_RESET);
            communicationHelper.publish(ROS2Tools.getControllerInputTopic(communicationHelper.getRobotName()).withTypeName(HandSakeDesiredCommandMessage.class),
                                        sakeCommand);
         }
      }
   }

   public void publishHandCommand(RobotSide side, HandConfiguration handDesiredConfiguration)
   {
      communicationHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                  HumanoidMessageTools.createHandDesiredConfigurationMessage(side, handDesiredConfiguration));
   }

   /**
    * Button for sending Nadia's arm to a position where it's holding a shield in front of the chest.
    */
   private void setupShieldButton(RDXBaseUI baseUI, CommunicationHelper communicationHelper)
   {
      RDX3DPanelToolbarButton shieldButton = baseUI.getPrimary3DPanel().addToolbarButton();
      shieldButton.loadAndSetIcon("icons/shield.png");
      shieldButton.setTooltipText("left/right side - testing shield lifting on Nadia");

      // Hand-tuned joint angles to hold the shield
      double[] leftJointAngles = new double[] {-1.01951, 0.72311, -1.29244, -1.26355, -0.51712, -0.04580, -0.00659};
      double[] rightJointAngles = new double[7];
      boolean[] invert = new boolean[] {false, true, true, false, true, false, false};

      for (int i = 0; i < leftJointAngles.length; i++)
      {
         rightJointAngles[i] = (invert[i] ? -1.0 : 1.0) * leftJointAngles[i];
      }

      SideDependentList<double[]> armJointAngles = new SideDependentList<>();
      armJointAngles.put(LEFT, leftJointAngles);
      armJointAngles.put(RIGHT, rightJointAngles);

      Consumer<RobotSide> armTrajectoryRunnable = robotSide ->
      {
         double trajectoryTime = 3.0;
         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime, armJointAngles.get(robotSide));
         communicationHelper.publishToController(armTrajectoryMessage);
      };

      shieldButton.setOnPressed(()-> armTrajectoryRunnable.accept(toolbarSelectedSide));
   }
}
