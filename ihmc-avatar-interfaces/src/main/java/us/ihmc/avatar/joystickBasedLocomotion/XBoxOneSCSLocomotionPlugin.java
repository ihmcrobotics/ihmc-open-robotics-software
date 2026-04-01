package us.ihmc.avatar.joystickBasedLocomotion;

import org.jcodec.containers.mp4.MP4Util.Atom;
import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.AvatarStepGeneratorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.joystickBasedJavaFXController.ButtonState;
import us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController;
import us.ihmc.avatar.joystickBasedLocomotion.AbstractJoystickLocomotionPlugin;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This is a plugin for an SCS application. It can convert inputs coming from an Xbox One
 * handheld controller into messages that get sent to the {@link AvatarControllerThread} and
 * {@link AvatarStepGeneratorThread} in order to enact and control walking movements.
 * <p>
 * For this plugin to work, ensure the Xbox controller is connected to the computer running
 * the SCS application via cable or wireless receiver dongle. Also make sure that the
 * {@link #update} method is called regularly every tick. It is additionally recommenced to
 * call {@link #shutdown()} upon SCS application termination.
 *
 * @author Stefan Fasano
 */
public class XBoxOneSCSLocomotionPlugin extends AbstractJoystickLocomotionPlugin
{
   public static final double DEFAULT_PARAMETER_INCREMENT = 0.01;

   private final double parameterIncrement;

   private final SharedMemoryMessager xboxJoystickMessager;
   private final XBoxOneJavaFXController xboxController;

   private final AtomicReference<Double> leftTriggerValue = new AtomicReference<>();
   private final AtomicReference<Double> leftJoystickXAxisValue = new AtomicReference<>();
   private final AtomicReference<Double> leftJoystickYAxisValue = new AtomicReference<>();
   private final AtomicReference<Double> rightJoystickXAxisValue = new AtomicReference<>();

   public XBoxOneSCSLocomotionPlugin(DRCRobotModel robotModel, ROS2Node ros2Node) throws JoystickNotFoundException
   {
      this(robotModel, ros2Node, DEFAULT_PARAMETER_INCREMENT);
   }

   public XBoxOneSCSLocomotionPlugin(DRCRobotModel robotModel, ROS2Node ros2Node, double parameterIncrement) throws JoystickNotFoundException
   {
      super(robotModel, ros2Node);
      this.parameterIncrement = parameterIncrement;

      xboxJoystickMessager = new SharedMemoryMessager(XBoxOneJavaFXController.XBoxOneControllerAPI);
      xboxController = new XBoxOneJavaFXController(xboxJoystickMessager);

      setupXboxJoystickControls();
   }

   @Override
   public void update()
   {
      // Reset everything to false and zero to be safe
      boolean walk = false;
      double forwardVelocity = 0;
      double lateralVelocity = 0;
      double turningVelocity = 0;

      // Check value of left trigger to determine whether to start stepping
      if (leftTriggerValue.get() != null)
         walk = leftTriggerValue.get() == -1.0;

      // Check value of joysticks to determine walking speeds/directions
      if (walk)
      {
         if (leftJoystickYAxisValue.get() != null)
            forwardVelocity = leftJoystickYAxisValue.get();
         if (leftJoystickXAxisValue.get() != null)
            lateralVelocity = leftJoystickXAxisValue.get();
         if (rightJoystickXAxisValue.get() != null)
            turningVelocity = rightJoystickXAxisValue.get();
      }

      // Pack the messages with the desired walking commands
      setDesiredWalkingCommands(walk, forwardVelocity, lateralVelocity, turningVelocity, true);

      // Publish the messages
      if (publisherThrottler.run() && !heartBeat.isExpired(PUBLISHER_HEARTBEAT_DURATION))
         publish();
   }

   public void startUp()
   {
      if (xboxJoystickMessager != null)
         xboxJoystickMessager.startMessager();

      if (xboxController != null)
         xboxController.reconnectJoystick();
   }

   @Override
   public void shutdown()
   {
      super.shutdown();

      if (xboxJoystickMessager != null)
         xboxJoystickMessager.closeMessager();

      if (xboxController != null)
         xboxController.stop();
   }

   private void setupXboxJoystickControls()
   {
      // Toggles between walking and standing
      xboxJoystickMessager.attachInput(XBoxOneJavaFXController.LeftTriggerAxis, leftTriggerValue);

      // Controls forwards walking
      xboxJoystickMessager.attachInput(XBoxOneJavaFXController.LeftStickYAxis, leftJoystickYAxisValue);

      // Controls lateral walking
      xboxJoystickMessager.attachInput(XBoxOneJavaFXController.LeftStickXAxis, leftJoystickXAxisValue);

      // Controls turning
      xboxJoystickMessager.attachInput(XBoxOneJavaFXController.RightStickXAxis, rightJoystickXAxisValue);

      // Decreases swing height
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.DPadDownState, state ->
      {
         csgParametersCommand.setSwingHeight(csgStatusMessage.getCurrentSwingHeight() - parameterIncrement);
         csgROS2CommunicationHelper.publish(csgParametersCommand);
      });

      // Increases swing height
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.DPadUpState, state ->
      {
         csgParametersCommand.setSwingHeight(csgStatusMessage.getCurrentSwingHeight() + parameterIncrement);
         csgROS2CommunicationHelper.publish(csgParametersCommand);
      });

      // Decreases swing duration
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.DPadLeftState, state ->
      {
         csgParametersCommand.setSwingDuration(csgStatusMessage.getCurrentSwingDuration() - parameterIncrement);
         csgROS2CommunicationHelper.publish(csgParametersCommand);
      });

      // Increases swing duration
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.DPadRightState, state ->
      {
         csgParametersCommand.setSwingDuration(csgStatusMessage.getCurrentSwingDuration() + parameterIncrement);
         csgROS2CommunicationHelper.publish(csgParametersCommand);
      });

      // Decreases transfer duration
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.ButtonLeftBumperState, state ->
      {
         csgParametersCommand.setTransferDuration(csgStatusMessage.getCurrentTransferDuration() - parameterIncrement);
         csgROS2CommunicationHelper.publish(csgParametersCommand);
      });

      // Increases transfer duration
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.ButtonRightBumperState, state ->
      {
         csgParametersCommand.setTransferDuration(csgStatusMessage.getCurrentTransferDuration() + parameterIncrement);
         csgROS2CommunicationHelper.publish(csgParametersCommand);
      });

      startUp();
   }
}
