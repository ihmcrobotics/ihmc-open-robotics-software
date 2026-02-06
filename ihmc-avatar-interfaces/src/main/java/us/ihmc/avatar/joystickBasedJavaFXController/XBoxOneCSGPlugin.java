package us.ihmc.avatar.joystickBasedJavaFXController;

import controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerPublisherMap;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGeneratorParametersBasics;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.ros2.QueuedROS2Subscription;
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
public class XBoxOneCSGPlugin
{
   public static final double DEFAULT_PARAMETER_INCREMENT = 0.01;
   private static final boolean DEFAULT_USE_DEADMAN_SWITCH = true;

   private final double parameterIncrement;
   private final boolean useDeadmanSwitch;

   private final SharedMemoryMessager xboxJoystickMessager;
   private final XBoxOneJavaFXController xboxController;

   private final ROS2ControllerPublisherMap ros2ControllerPublisherMap;
   private final ContinuousStepGeneratorInputMessage csgInputCommand;
   private final ContinuousStepGeneratorParametersMessage csgParametersCommand;

   private final QueuedROS2Subscription<ContinuousStepGeneratorStatusMessage> csgStatusSubscription;
   private final ContinuousStepGeneratorStatusMessage csgStatusMessage = new ContinuousStepGeneratorStatusMessage();

   public XBoxOneCSGPlugin(DRCRobotModel robotModel, ROS2Node ros2Node) throws JoystickNotFoundException
   {
      this(robotModel, ros2Node, DEFAULT_PARAMETER_INCREMENT, DEFAULT_USE_DEADMAN_SWITCH);
   }

   public XBoxOneCSGPlugin(DRCRobotModel robotModel, ROS2Node ros2Node, double parameterIncrement, boolean useDeadmanSwitch) throws JoystickNotFoundException
   {
      this.parameterIncrement = parameterIncrement;
      this.useDeadmanSwitch = useDeadmanSwitch;

      ros2ControllerPublisherMap = new ROS2ControllerPublisherMap(ros2Node, robotModel.getSimpleRobotName());
      csgInputCommand = new ContinuousStepGeneratorInputMessage();
      csgParametersCommand = new ContinuousStepGeneratorParametersMessage();
      csgStatusSubscription = ros2Node.createQueuedSubscription(HumanoidControllerAPI.getTopic(ContinuousStepGeneratorStatusMessage.class, robotModel.getSimpleRobotName()), 10);

      xboxJoystickMessager = new SharedMemoryMessager(XBoxOneJavaFXController.XBoxOneControllerAPI);
      xboxController = new XBoxOneJavaFXController(xboxJoystickMessager);

      configureCSGParameters(csgParametersCommand, robotModel.getWalkingControllerParameters());
      setupXboxJoystickControls();
   }

   public void update()
   {
      if (csgStatusSubscription.flushAndGetLatest(csgStatusMessage))
         setCSGCommandsToCurrentValues(csgStatusMessage);
   }

   public void startUpXboxJoystick()
   {
      if (xboxJoystickMessager != null)
         xboxJoystickMessager.startMessager();

      if (xboxController != null)
         xboxController.reconnectJoystick();
   }

   public void shutDownXboxJoystick()
   {
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
         xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.LeftTriggerAxis, state ->
         {
            if (state == -1.0)
               csgInputCommand.setWalk(true);
            else
               csgInputCommand.setWalk(false);

            ros2ControllerPublisherMap.publish(csgInputCommand);
         });
      }
      else
      {
         xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.ButtonAState, state ->
         {
            if (state == ButtonState.PRESSED)
               csgInputCommand.setWalk(!csgStatusMessage.getIsWalking());

            ros2ControllerPublisherMap.publish(csgInputCommand);
         });
      }

      // Controls forwards walking
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.LeftStickYAxis, state ->
      {
         csgInputCommand.setForwardVelocity(state);
         csgInputCommand.setUnitVelocities(false);
         ros2ControllerPublisherMap.publish(csgInputCommand);
      });

      // Controls lateral walking
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.LeftStickXAxis, state ->
      {
         csgInputCommand.setLateralVelocity(state);
         csgInputCommand.setUnitVelocities(false);
         ros2ControllerPublisherMap.publish(csgInputCommand);
      });

      // Controls turning
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.RightStickXAxis, state ->
      {
         csgInputCommand.setTurnVelocity(state);
         csgInputCommand.setUnitVelocities(false);
         ros2ControllerPublisherMap.publish(csgInputCommand);
      });

      // Decreases swing height
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.DPadDownState, state ->
      {
         csgParametersCommand.setSwingHeight(csgStatusMessage.getCurrentSwingHeight() - parameterIncrement);
         ros2ControllerPublisherMap.publish(csgParametersCommand);
      });

      // Increases swing height
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.DPadUpState, state ->
      {
         csgParametersCommand.setSwingHeight(csgStatusMessage.getCurrentSwingHeight() + parameterIncrement);
         ros2ControllerPublisherMap.publish(csgParametersCommand);
      });

      // Decreases swing duration
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.DPadLeftState, state ->
      {
         csgParametersCommand.setSwingDuration(csgStatusMessage.getCurrentSwingDuration() - parameterIncrement);
         ros2ControllerPublisherMap.publish(csgParametersCommand);
      });

      // Increases swing duration
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.DPadRightState, state ->
      {
         csgParametersCommand.setSwingDuration(csgStatusMessage.getCurrentSwingDuration() + parameterIncrement);
         ros2ControllerPublisherMap.publish(csgParametersCommand);
      });

      // Decreases transfer duration
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.ButtonLeftBumperState, state ->
      {
         csgParametersCommand.setTransferDuration(csgStatusMessage.getCurrentTransferDuration() - parameterIncrement);
         ros2ControllerPublisherMap.publish(csgParametersCommand);
      });

      // Increases transfer duration
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.ButtonRightBumperState, state ->
      {
         csgParametersCommand.setTransferDuration(csgStatusMessage.getCurrentTransferDuration() + parameterIncrement);
         ros2ControllerPublisherMap.publish(csgParametersCommand);
      });

      startUpXboxJoystick();
   }

   private void setCSGCommandsToCurrentValues(ContinuousStepGeneratorStatusMessage csgStatusMessage)
   {
      csgParametersCommand.setSwingDuration(csgStatusMessage.getCurrentSwingDuration());
      csgParametersCommand.setTransferDuration(csgStatusMessage.getCurrentTransferDuration());

      csgParametersCommand.setSwingHeight(csgStatusMessage.getCurrentSwingHeight());
      csgParametersCommand.setMaxStepLengthForwards(csgStatusMessage.getCurrentMaxStepLengthForwards());
      csgParametersCommand.setMaxStepLengthBackwards(csgStatusMessage.getCurrentMaxStepLengthBackwards());
      csgParametersCommand.setDefaultStepWidth(csgStatusMessage.getCurrentDefaultStepWidth());
      csgParametersCommand.setMinStepWidth(csgStatusMessage.getCurrentMinStepWidth());
      csgParametersCommand.setMaxStepWidth(csgStatusMessage.getCurrentMaxStepWidth());
      csgParametersCommand.setTurnMaxAngleInward(csgStatusMessage.getCurrentTurnMaxAngleInward());
      csgParametersCommand.setTurnMaxAngleOutward(csgStatusMessage.getCurrentTurnMaxAngleOutward());
   }

   private void configureCSGParameters(ContinuousStepGeneratorParametersMessage csgParametersCommand, WalkingControllerParameters walkingControllerParameters)
   {
      csgParametersCommand.setNumberOfFootstepsToPlan(ContinuousStepGeneratorParametersBasics.DEFAULT_NUMBER_OF_FOOTSTEPS_TO_PLAN);
      csgParametersCommand.setNumberOfFixedFootsteps(ContinuousStepGeneratorParametersBasics.DEFAULT_NUMBER_OF_FIXED_FOOTSTEPS);
      csgParametersCommand.setSwingDuration(walkingControllerParameters.getDefaultSwingTime());
      csgParametersCommand.setTransferDuration(walkingControllerParameters.getDefaultTransferTime());

      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParametersForStepGeneration();
      csgParametersCommand.setSwingHeight(walkingControllerParameters.getSwingTrajectoryParameters().getDefaultSwingHeight());
      csgParametersCommand.setMaxStepLengthForwards(steppingParameters.getMaxStepLength());
      csgParametersCommand.setMaxStepLengthBackwards(steppingParameters.getMaxBackwardStepLength());
      csgParametersCommand.setDefaultStepWidth(steppingParameters.getInPlaceWidth());
      csgParametersCommand.setMinStepWidth(steppingParameters.getMinStepWidth());
      csgParametersCommand.setMaxStepWidth(steppingParameters.getMaxStepWidth());
      csgParametersCommand.setTurnMaxAngleInward(steppingParameters.getMaxAngleTurnInwards());
      csgParametersCommand.setTurnMaxAngleOutward(steppingParameters.getMaxAngleTurnOutwards());
   }
}
