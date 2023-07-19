package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HandSakeStatusMessage;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.SakeHandCommandOption;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXSakeHandPositionSlider;
import us.ihmc.rdx.ui.interactable.RDXSakeHandTorqueSlider;
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
   private final SideDependentList<RDXIconTexture> handIcons = new SideDependentList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RobotSide toolbarSelectedSide = RobotSide.LEFT;
   private final SideDependentList<IHMCROS2Input<HandSakeStatusMessage>> sakeStatuses = new SideDependentList<>();
   private final SideDependentList<RDXSakeHandPositionSlider> handPositionSliders = new SideDependentList<>();
   private final SideDependentList<RDXSakeHandTorqueSlider> handTorqueSliders = new SideDependentList<>();

   public void create(RDXBaseUI baseUI, CommunicationHelper communicationHelper, ROS2SyncedRobotModel syncedRobot)
   {
      this.communicationHelper = communicationHelper;

      for (RobotSide side : RobotSide.values)
      {
         handIcons.put(side, new RDXIconTexture("icons/" + side.getLowerCaseName() + "Hand.png"));
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

      SideDependentList<Runnable> openCommands = new SideDependentList<>(() -> publishHandCommand(RobotSide.LEFT, SakeHandCommandOption.OPEN),
                                                                          () -> publishHandCommand(RobotSide.RIGHT, SakeHandCommandOption.OPEN));
      RDX3DPanelToolbarButton openHandButton = baseUI.getPrimary3DPanel().addToolbarButton();
      openHandButton.loadAndSetIcon("icons/openGripper.png");
      openHandButton.setTooltipText("Open hand");
      openHandButton.setOnPressed(() -> openCommands.get(toolbarSelectedSide).run());

      SideDependentList<Runnable> closeCommands = new SideDependentList<>(() -> publishHandCommand(RobotSide.LEFT, SakeHandCommandOption.CLOSE),
                                                                          () -> publishHandCommand(RobotSide.RIGHT, SakeHandCommandOption.CLOSE));
      RDX3DPanelToolbarButton closeHandButton = baseUI.getPrimary3DPanel().addToolbarButton();
      closeHandButton.loadAndSetIcon("icons/closeGripper.png");
      closeHandButton.setTooltipText("Close hand");
      closeHandButton.setOnPressed(() -> closeCommands.get(toolbarSelectedSide).run());

      RDX3DPanelToolbarButton armHomeButton = baseUI.getPrimary3DPanel().addToolbarButton();
      armHomeButton.loadAndSetIcon("icons/home.png");
      armHomeButton.setTooltipText("left/right arm home pose");
      armHomeButton.setOnPressed(() -> publishArmHomeCommand(toolbarSelectedSide));

      for (RobotSide handSide : RobotSide.values)
      {
         handPositionSliders.put(handSide, new RDXSakeHandPositionSlider(syncedRobot, communicationHelper, handSide));
         handTorqueSliders.put(handSide, new RDXSakeHandTorqueSlider(communicationHelper, handSide));
      }

      if (ADD_SHIELD_BUTTON)
      {
         setupShieldButton(baseUI, communicationHelper);
      }
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
         if (sakeStatuses.get(side).hasReceivedFirstMessage())
         {
            // TODO: Add ratio to unit conversions enum or something
            double temperature = 100 * sakeStatuses.get(side).getLatest().getTemperature();
            boolean error = sakeStatuses.get(side).getLatest().getIsInErrorState();
            double errorValue = error ? 1.0 : 0.0;
            renderGradiatedImGuiText("Temperature: " + FormattingTools.getFormattedDecimal1D(temperature) + " C", temperature, 45.0, 50.0, 55.0);
            ImGui.sameLine();
            renderGradiatedImGuiText("Status: " + (error ? "ERROR" : "all good"), errorValue, 1.0);
         }
         else
         {
            ImGui.text("No status received.");
         }
         handPositionSliders.get(side).renderImGuiWidgets();
         handTorqueSliders.get(side).renderImGuiWidgets();
      }
   }

   private void renderGradiatedImGuiText(String text, double value, double... colorSwitchValues)
   {
      int redValue = 0;
      int greenValue = 192;

      for (double switchValue : colorSwitchValues)
      {
         if (value < switchValue)
            break;

         redValue = 192;
         greenValue -= 192 / colorSwitchValues.length;
      }

      ImGui.textColored(redValue, greenValue, 0, 255, text);
   }

   private void publishHandCommand(RobotSide side, SakeHandCommandOption handCommandOption)
   {
      communicationHelper.publish(ROS2Tools::getHandSakeCommandTopic,
                                  HumanoidMessageTools.createHandSakeDesiredCommandMessage(side, handCommandOption, 0.0, 0.0));
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

      shieldButton.setOnPressed(() -> armTrajectoryRunnable.accept(toolbarSelectedSide));
   }
}
