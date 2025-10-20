package us.ihmc.avatar.joystickBasedJavaFXController;

import controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerPublisherMap;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGeneratorParametersBasics;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;

public class XBoxOneCSGPlugin
{
   private static final double DEFAULT_PARAMETER_INCREMENT = 0.01;

   private final double parameterIncrement;

   private final SharedMemoryMessager xboxJoystickMessager;
   private final XBoxOneJavaFXController xboxController;

   private final ROS2ControllerPublisherMap ros2ControllerPublisherMap;
   private final ContinuousStepGeneratorInputMessage csgInputCommand;
   private final ContinuousStepGeneratorParametersMessage csgParametersCommand;

   public XBoxOneCSGPlugin(DRCRobotModel robotModel, ROS2Node ros2Node) throws JoystickNotFoundException
   {
      this(robotModel, ros2Node, DEFAULT_PARAMETER_INCREMENT);
   }

   public XBoxOneCSGPlugin(DRCRobotModel robotModel, ROS2Node ros2Node, double parameterIncrement) throws JoystickNotFoundException
   {
      this.parameterIncrement = parameterIncrement;

      ros2ControllerPublisherMap = new ROS2ControllerPublisherMap(ros2Node, robotModel.getSimpleRobotName());
      csgInputCommand = new ContinuousStepGeneratorInputMessage();
      csgParametersCommand = new ContinuousStepGeneratorParametersMessage();

      xboxJoystickMessager = new SharedMemoryMessager(XBoxOneJavaFXController.XBoxOneControllerAPI);
      xboxController = new XBoxOneJavaFXController(xboxJoystickMessager);

      configureCSGParameters(csgParametersCommand, robotModel.getWalkingControllerParameters());
      setupXboxJoystickControls();
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
//      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.ButtonAState, state ->
//      {
//         if (state == ButtonState.PRESSED)
//            csgInputCommand.setWalk(!isWalkingCSG);
//
//         ros2ControllerPublisherMap.publish(csgInputCommand);
//      });
      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.RightTriggerAxis, state ->
      {
         if (state == 1.0)
            csgInputCommand.setWalk(!isWalkingCSG);

         ros2ControllerPublisherMap.publish(csgInputCommand);
      });

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

//      // Decreases swing height
//      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.DPadDownState, state ->
//      {
//         csgParametersCommand.setSwingHeight(swingHeight - parameterIncrement);
//         ros2ControllerPublisherMap.publish(csgParametersCommand);
//      });
//
//      // Increases swing height
//      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.DPadUpState, state ->
//      {
//         csgParametersCommand.setSwingHeight(swingHeight + parameterIncrement);
//         ros2ControllerPublisherMap.publish(csgParametersCommand);
//      });
//
//      // Decreases swing duration
//      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.DPadLeftState, state ->
//      {
//         csgParametersCommand.setSwingDuration(swingDuration - parameterIncrement);
//         ros2ControllerPublisherMap.publish(csgParametersCommand);
//      });
//
//      // Increases swing duration
//      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.DPadRightState, state ->
//      {
//         csgParametersCommand.setSwingDuration(swingDuration + parameterIncrement);
//         ros2ControllerPublisherMap.publish(csgParametersCommand);
//      });
//
//      // Decreases transfer duration
//      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.ButtonLeftBumperState, state ->
//      {
//         csgParametersCommand.setTransferDuration(transferDuration - parameterIncrement);
//         ros2ControllerPublisherMap.publish(csgParametersCommand);
//      });
//
//      // Increases transfer duration
//      xboxJoystickMessager.addTopicListener(XBoxOneJavaFXController.ButtonRightBumperState, state ->
//      {
//         csgParametersCommand.setTransferDuration(transferDuration + parameterIncrement);
//         ros2ControllerPublisherMap.publish(csgParametersCommand);
//      });

      startUpXboxJoystick();
   }

   private void configureCSGParameters(ContinuousStepGeneratorParametersMessage csgParametersCommand, WalkingControllerParameters walkingControllerParameters)
   {
      csgParametersCommand.setNumberOfFootstepsToPlan(ContinuousStepGeneratorParametersBasics.DEFAULT_NUMBER_OF_FOOTSTEPS_TO_PLAN);
      csgParametersCommand.setNumberOfFixedFootsteps(ContinuousStepGeneratorParametersBasics.DEFAULT_NUMBER_OF_FIXED_FOOTSTEPS);
      csgParametersCommand.setSwingDuration(walkingControllerParameters.getDefaultSwingTime());
      csgParametersCommand.setTransferDuration(walkingControllerParameters.getDefaultTransferTime());

      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      csgParametersCommand.setSwingHeight(walkingControllerParameters.getSwingTrajectoryParameters().getDefaultSwingHeight());
      csgParametersCommand.setMaxStepLength(steppingParameters.getMaxStepLength());
      csgParametersCommand.setDefaultStepWidth(steppingParameters.getInPlaceWidth());
      csgParametersCommand.setMinStepWidth(steppingParameters.getMinStepWidth());
      csgParametersCommand.setMaxStepWidth(steppingParameters.getMaxStepWidth());
      csgParametersCommand.setTurnMaxAngleInward(steppingParameters.getMaxAngleTurnInwards());
      csgParametersCommand.setTurnMaxAngleOutward(steppingParameters.getMaxAngleTurnOutwards());
   }
}
