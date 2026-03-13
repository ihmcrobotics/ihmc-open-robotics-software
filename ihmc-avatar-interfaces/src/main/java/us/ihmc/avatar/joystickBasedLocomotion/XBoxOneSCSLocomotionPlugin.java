package us.ihmc.avatar.joystickBasedLocomotion;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.joystickBasedJavaFXController.ButtonState;
import us.ihmc.avatar.joystickBasedJavaFXController.XBoxOneJavaFXController;
import us.ihmc.avatar.joystickBasedLocomotion.AbstractJoystickLocomotionPlugin;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;

/**
 * Plugin for using an xbox controller to send commands and receive status info to/from the
 * {@link us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator}
 * via the {@link us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition}
 * This requires no instantiation or direct interaction with the ContinuousStepGenerator, all communication is done
 * through ROS2. Additionally, this can be attached to any program or application with an update loop. Just make
 * sure the Xbox controller is physically connected to the computer running the application
 *
 * @author Stefan Fasano
 */
public class XBoxOneSCSLocomotionPlugin extends AbstractJoystickLocomotionPlugin
{
   public static final double DEFAULT_PARAMETER_INCREMENT = 0.01;
   private static final boolean DEFAULT_USE_DEADMAN_SWITCH = true;

   private final double parameterIncrement;
   private final boolean useDeadmanSwitch;

   private final SharedMemoryMessager xboxJoystickMessager;
   private final XBoxOneJavaFXController xboxController;

   private boolean walk = false;
   private double forwardVelocity = 0;
   private double lateralVelocity = 0;
   private double turningVelocity = 0;

   public XBoxOneSCSLocomotionPlugin(DRCRobotModel robotModel, ROS2Node ros2Node) throws JoystickNotFoundException
   {
      this(robotModel, ros2Node, DEFAULT_PARAMETER_INCREMENT, DEFAULT_USE_DEADMAN_SWITCH);
   }

   public XBoxOneSCSLocomotionPlugin(DRCRobotModel robotModel, ROS2Node ros2Node, double parameterIncrement, boolean useDeadmanSwitch) throws JoystickNotFoundException
   {
      super(robotModel, ros2Node);
      this.parameterIncrement = parameterIncrement;
      this.useDeadmanSwitch = useDeadmanSwitch;

      xboxJoystickMessager = new SharedMemoryMessager(XBoxOneJavaFXController.XBoxOneControllerAPI);
      xboxController = new XBoxOneJavaFXController(xboxJoystickMessager);

      setupXboxJoystickControls();
   }

   @Override
   public void update()
   {
      // Pack the messages with the desired walking commands
      setDesiredWalkingCommands(walk, forwardVelocity, lateralVelocity, turningVelocity);

      // Publish the messages
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
      if (useDeadmanSwitch)
      {
         xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.LeftTriggerAxis, state -> walk = state == -1.0);
      }
      else
      {
         xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.ButtonAState, state ->
         {
            if (state == ButtonState.PRESSED)
               walk = !csgStatusMessage.getIsWalking();
         });
      }

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
