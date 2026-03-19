package us.ihmc.avatar.joystickBasedLocomotion;

import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.AvatarStepGeneratorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.joystickBasedJavaFXController.ButtonState;
import us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController;
import us.ihmc.avatar.joystickBasedLocomotion.AbstractJoystickLocomotionPlugin;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;

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

   private boolean walk = false;
   private double forwardVelocity = 0;
   private double lateralVelocity = 0;
   private double turningVelocity = 0;

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
      // Pack the messages with the desired walking commands
      setDesiredWalkingCommands(walk, forwardVelocity, lateralVelocity, turningVelocity, true);

      // Publish the messages
      if (publisherThrottler.run() && !heartBeat.isExpired(PUBLISHER_HEARTBEAT_DURATION))
         publish();

      // Reset everything to false and zero to be safe
      walk = false;
      forwardVelocity = 0;
      lateralVelocity = 0;
      turningVelocity = 0;
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
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.LeftTriggerAxis, state -> walk = state == -1.0);

      // Controls forwards walking
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.LeftStickYAxis, state -> forwardVelocity = state);

      // Controls lateral walking
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.LeftStickXAxis, state -> lateralVelocity = state);

      // Controls turning
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.RightStickXAxis, state -> turningVelocity = state);

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
