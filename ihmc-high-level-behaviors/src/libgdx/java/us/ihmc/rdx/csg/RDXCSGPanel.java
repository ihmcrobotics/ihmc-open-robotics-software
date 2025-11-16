package us.ihmc.rdx.csg;

import controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.CSGROS2CommunicationHelper;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.ros2.ROS2Node;

public class RDXCSGPanel extends RDXPanel
{
   private static final double THROTTLER_THREAD_HERTZ = 10.0;

   private static final double FORWARD_VELOCITY_MIN_MAX = 0.9; // In m/s
   private static final double LATERAL_VELOCITY_MIN_MAX = 0.7; // In m/s
   private static final double TURNING_VELOCITY_MIN_MAX = Math.PI / 2.0; // In rad/s

   private static final double MAX_STEP_LENGTH_MIN = 0.2; // In m
   private static final double MAX_STEP_LENGTH_MAX = 0.5; // In m
   
   private static final double MAX_STEP_WIDTH_MIN = 0.2; // In m
   private static final double MAX_STEP_WIDTH_MAX = 0.5; // In m

   private static final double TRANSFER_DURATION_MIN = 0.3; // In sec
   private static final double TRANSFER_DURATION_MAX = 0.6; // In sec

   private static final double SWING_DURATION_MIN = 0.6; // In sec
   private static final double SWING_DURATION_MAX = 0.9; // In sec

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final ImBoolean requestWalkCSG = new ImBoolean();

   private final ImDouble forwardVelocity = new ImDouble();
   private final ImDouble lateralVelocity = new ImDouble();
   private final ImDouble turningVelocity = new ImDouble();

   private final ImDouble maxStepLength = new ImDouble();
   private final ImDouble maxStepWidth = new ImDouble();

   private final ImDouble swingDuration = new ImDouble();
   private final ImDouble transferDuration = new ImDouble();

   // CSG ROS communication helper
   private final CSGROS2CommunicationHelper communicationHelper;

   // CSG command and status messages
   private final ContinuousStepGeneratorInputMessage csgInputCommand;
   private final ContinuousStepGeneratorParametersMessage csgParametersCommand;
   private final ContinuousStepGeneratorStatusMessage csgStatusMessage;

   // Xbox joystick walking plugin
   private RDXXboxOneCSGPlugin xBoxOneCSGPlugin;

   // Throttler for ensuring we aren't publishing too fast/often
   private final Throttler csgROS2PublisherThrottler;

   public RDXCSGPanel(DRCRobotModel robotModel, ROS2Node ros2Node, boolean createXboxPlugin)
   {
      this(new CSGROS2CommunicationHelper(robotModel.getSimpleRobotName(), ros2Node, robotModel.getWalkingControllerParameters()), createXboxPlugin);
   }

   public RDXCSGPanel(CSGROS2CommunicationHelper communicationHelper, boolean createXboxPlugin)
   {
      super("CSG Controls");
      super.setRenderMethod(this::update);

      this.communicationHelper = communicationHelper;

      csgInputCommand = communicationHelper.getCSGInputCommand();
      csgParametersCommand = communicationHelper.getCSGParametersCommand();
      csgStatusMessage = communicationHelper.getCSGStatusMessage();

      csgROS2PublisherThrottler = new Throttler().setFrequency(THROTTLER_THREAD_HERTZ);

      if (createXboxPlugin)
         xBoxOneCSGPlugin = new RDXXboxOneCSGPlugin(communicationHelper);
   }

   public void update()
   {
      // Update and publish CSG commands from controller first
      if (xBoxOneCSGPlugin != null)
         xBoxOneCSGPlugin.update(csgInputCommand);

      boolean requestCSGWalkingChanged = ImGui.checkbox(labels.get("Request Walk CSG"), requestWalkCSG);

      boolean desiredForwardVelocityChanged = ImGuiTools.sliderDouble(labels.getHidden("Forward Velocity (% min/max)"), forwardVelocity, -1.0, 1.0, "Forward Velocity (% min/max): %.2f");
      boolean desiredLateralVelocityChanged = ImGuiTools.sliderDouble(labels.getHidden("Lateral Velocity (% min/max)"), lateralVelocity, -1.0, 1.0, "Lateral Velocity (% min/max): %.2f");
      boolean desiredTurningVelocityChanged = ImGuiTools.sliderDouble(labels.getHidden("Turning Velocity (% min/max)"), turningVelocity, -1.0, 1.0, "Turning Velocity (% min/max): %.2f");

      boolean swingDurationChanged = ImGuiTools.volatileInputDouble(labels.get("Swing Duration"), swingDuration);//ImGuiTools.sliderDouble(labels.getHidden("Swing Duration"), swingDuration, SWING_DURATION_MIN, SWING_DURATION_MAX, "Swing Duration (s): %.2f");
      ImGui.sameLine();
      boolean transferDurationChanged = ImGuiTools.volatileInputDouble(labels.get("Transfer Duration"), transferDuration);//ImGuiTools.sliderDouble(labels.getHidden("Transfer Duration"), transferDuration, TRANSFER_DURATION_MIN, TRANSFER_DURATION_MAX, "Transfer Duration (s): %.2f");

      boolean maxStepLengthChanged = ImGuiTools.volatileInputDouble(labels.get("Max Step Length"), maxStepLength);//ImGuiTools.sliderDouble(labels.getHidden("Max Step Length"), maxStepLength, MAX_STEP_LENGTH_MIN, MAX_STEP_LENGTH_MAX, "Max Step Length (m): %.2f");
      ImGui.sameLine();
      boolean maxStepWidthChanged = ImGuiTools.volatileInputDouble(labels.get("Max Step Width"), maxStepWidth);//ImGuiTools.sliderDouble(labels.getHidden("Max Step Width"), maxStepWidth, MAX_STEP_WIDTH_MIN, MAX_STEP_WIDTH_MAX, "Max Step Width (m): %.2f");

      if (requestCSGWalkingChanged)
         csgInputCommand.setWalk(requestWalkCSG.get());

      if (desiredForwardVelocityChanged)
      {
         csgInputCommand.setForwardVelocity(forwardVelocity.get());
         forwardVelocity.set(0.0);
      }

      if (desiredLateralVelocityChanged)
      {
         csgInputCommand.setLateralVelocity(lateralVelocity.get());
         lateralVelocity.set(0.0);
      }

      if (desiredTurningVelocityChanged)
      {
         csgInputCommand.setTurnVelocity(turningVelocity.get());
         turningVelocity.set(0.0);
      }

      if (swingDurationChanged)
         csgParametersCommand.setSwingDuration(swingDuration.get());

      if (transferDurationChanged)
         csgParametersCommand.setTransferDuration(transferDuration.get());

      if (maxStepLengthChanged)
         csgParametersCommand.setMaxStepLength(maxStepLength.get());

      if (maxStepWidthChanged)
         csgParametersCommand.setMaxStepWidth(maxStepWidth.get());

      if (csgROS2PublisherThrottler.run())
      {
         communicationHelper.publish(csgInputCommand);
         communicationHelper.publish(csgParametersCommand);
      }

      reset();
   }

   private void reset()
   {
      requestWalkCSG.set(csgStatusMessage.getIsWalking());
      forwardVelocity.set(csgStatusMessage.getCurrentForwardVelocity());
      lateralVelocity.set(csgStatusMessage.getCurrentLateralVelocity());
      turningVelocity.set(csgStatusMessage.getCurrentTurnVelocity());

      swingDuration.set(csgStatusMessage.getCurrentSwingDuration());
      transferDuration.set(csgStatusMessage.getCurrentTransferDuration());

      maxStepLength.set(csgStatusMessage.getCurrentMaxStepLength());
      maxStepWidth.set(csgStatusMessage.getCurrentMaxStepWidth());
   }

   public void destroy()
   {
      if (xBoxOneCSGPlugin != null)
         xBoxOneCSGPlugin.shutDownXboxJoystick();
   }
}
