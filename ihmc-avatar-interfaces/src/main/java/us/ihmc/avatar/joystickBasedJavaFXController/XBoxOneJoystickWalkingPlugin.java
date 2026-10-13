package us.ihmc.avatar.joystickBasedJavaFXController;

import behavior_msgs.msg.dds.ContinuousHikingCommandMessage;
import com.badlogic.gdx.controllers.Controller;
import com.badlogic.gdx.controllers.ControllerListener;
import com.badlogic.gdx.controllers.Controllers;
import controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerPublisherMap;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGeneratorParametersBasics;
import us.ihmc.commons.DeadbandTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.ros2.QueuedROS2Subscription;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

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
public class XBoxOneJoystickWalkingPlugin
{
   public static final double DEFAULT_PARAMETER_INCREMENT = 0.01;
   private static final boolean DEFAULT_USE_DEADMAN_SWITCH = true;

   private enum JoystickWalkingMode {CSG, CONTINUOUS_HIKING}
   private JoystickWalkingMode currentJoystickWalkingMode = JoystickWalkingMode.CSG;

   private final double parameterIncrement;
   private final boolean useDeadmanSwitch;

   private Controller currentController;
   private final ControllerListener controllerListener;
   private boolean currentControllerConnected = false;
   private boolean controllerListenerHasBeenAdded = false;

   private final ROS2ControllerPublisherMap ros2CSGPublisherMap;
   private final ContinuousStepGeneratorInputMessage csgInputCommand;
   private final ContinuousStepGeneratorParametersMessage csgParametersCommand;

   private final QueuedROS2Subscription<ContinuousStepGeneratorStatusMessage> csgStatusSubscription;
   private final ContinuousStepGeneratorStatusMessage csgStatusMessage = new ContinuousStepGeneratorStatusMessage();

   private final ROS2Publisher<ContinuousHikingCommandMessage> continuousHikingCommandPublisher;
   private final ContinuousHikingCommandMessage continuousHikingCommand = new ContinuousHikingCommandMessage();

   public XBoxOneJoystickWalkingPlugin(DRCRobotModel robotModel, ROS2Node ros2Node)
   {
      this(robotModel, ros2Node, DEFAULT_PARAMETER_INCREMENT, DEFAULT_USE_DEADMAN_SWITCH);
   }

   public XBoxOneJoystickWalkingPlugin(DRCRobotModel robotModel, ROS2Node ros2Node, double parameterIncrement, boolean useDeadmanSwitch)
   {
      this.parameterIncrement = parameterIncrement;
      this.useDeadmanSwitch = useDeadmanSwitch;

      // Set up CSG commands, publisher, and status subscriber
      ros2CSGPublisherMap = new ROS2ControllerPublisherMap(ros2Node, robotModel.getSimpleRobotName());
      csgStatusSubscription = ros2Node.createQueuedSubscription(HumanoidControllerAPI.getTopic(ContinuousStepGeneratorStatusMessage.class, robotModel.getSimpleRobotName()), 10);
      csgInputCommand = new ContinuousStepGeneratorInputMessage();
      csgParametersCommand = new ContinuousStepGeneratorParametersMessage();

      // Set up CH publisher
      continuousHikingCommandPublisher = ros2Node.createPublisher(ContinuousHikingAPI.CONTINUOUS_HIKING_COMMAND);

      // Initialize CSG parameter command from default initial robot parameters
      initializeCSGParametersCommand(csgParametersCommand, robotModel.getWalkingControllerParameters());

      // Set up controller listener
      controllerListener = new ControllerListener()
      {
         @Override
         public void connected(Controller controller)
         {
         }

         @Override
         public void disconnected(Controller controller)
         {
            currentControllerConnected = false;

            sendStopWalkingCommands();
         }

         @Override
         public boolean buttonDown(Controller controller, int buttonCode)
         {
            if (controller != null)
            {
               if (buttonCode == controller.getMapping().buttonDpadLeft)
                  csgParametersCommand.setSwingDuration(csgStatusMessage.getCurrentSwingDuration() - parameterIncrement);
               else if (buttonCode == controller.getMapping().buttonDpadRight)
                  csgParametersCommand.setSwingDuration(csgStatusMessage.getCurrentSwingDuration() + parameterIncrement);

               else if (buttonCode == controller.getMapping().buttonX)
                  csgParametersCommand.setTransferDuration(csgStatusMessage.getCurrentTransferDuration() - parameterIncrement);
               else if (buttonCode == controller.getMapping().buttonB)
                  csgParametersCommand.setTransferDuration(csgStatusMessage.getCurrentTransferDuration() + parameterIncrement);

               else if (buttonCode == controller.getMapping().buttonDpadDown)
                  csgParametersCommand.setSwingHeight(csgStatusMessage.getCurrentSwingHeight() - parameterIncrement);
               else if (buttonCode == controller.getMapping().buttonDpadUp)
                  csgParametersCommand.setSwingHeight(csgStatusMessage.getCurrentSwingHeight() + parameterIncrement);

               else if (buttonCode == controller.getMapping().buttonL1)
                  continuousHikingCommand.setSquareUpToGoal(true);

               ros2CSGPublisherMap.publish(csgParametersCommand);
            }
            return false;
         }

         @Override
         public boolean buttonUp(Controller controller, int buttonCode)
         {
            return false;
         }

         @Override
         public boolean axisMoved(Controller controller, int axisCode, float value)
         {
            return false;
         }
      };
   }

   public void update()
   {
      // Get latest CSG status message, and reset CSG parameters command from that
      if (csgStatusSubscription.flushAndGetLatest(csgStatusMessage))
         setCSGCommandsToCurrentValues(csgStatusMessage);

      // Check if new joystick has been connected
      boolean newControllerConnected = false;
      if (currentController != null && currentController != Controllers.getCurrent())
         newControllerConnected = true;

      currentController = Controllers.getCurrent();
      currentControllerConnected = currentController != null;

      // Check if controller listener has been added
      if ((!controllerListenerHasBeenAdded || newControllerConnected) && currentControllerConnected)
      {
         currentController.addListener(controllerListener);
         controllerListenerHasBeenAdded = true;
      }

      // Assemble our command from joystick inputs and publish it
      switch (currentJoystickWalkingMode)
      {
         case CSG ->
         {
            // Assemble our CSG input command
            configureCSGInputCommand();

            // Publish our CSG input command
            ros2CSGPublisherMap.publish(csgInputCommand);
         }

         case CONTINUOUS_HIKING ->
         {
            // Assemble our continuous hiking command
            configureContinuousHikingCommand();

            // Publish our continuous hiking command
            continuousHikingCommandPublisher.publish(continuousHikingCommand);

            // Reset this to false each time
            continuousHikingCommand.setSquareUpToGoal(false);
         }
      }
   }

   private void configureCSGInputCommand()
   {
      // Default CSG input values
      boolean requestWalking = false;
      double forwardJoystickValue = 0.0;
      double lateralJoystickValue = 0.0;
      double turningJoystickValue = 0.0;
      double deadband = 0.09;

      // CSG input values we get from the xbox controller
      if (currentControllerConnected)
      {
         requestWalking = currentController.getAxis(4) == 1.0; // This is the left trigger value (1.0 = pressed in)
         forwardJoystickValue = DeadbandTools.applyDeadband(deadband, -currentController.getAxis(currentController.getMapping().axisLeftY));
         lateralJoystickValue = DeadbandTools.applyDeadband(deadband, -currentController.getAxis(currentController.getMapping().axisLeftX));
         turningJoystickValue = DeadbandTools.applyDeadband(deadband, -currentController.getAxis(currentController.getMapping().axisRightX));
      }

      configureCSGInputCommand(requestWalking, forwardJoystickValue, lateralJoystickValue, turningJoystickValue);
   }

   private void configureCSGInputCommand(boolean requestWalking, double desiredForwardVelocity, double desiredLateralVelocity, double desiredTurningVelocity)
   {
      csgInputCommand.setWalk(requestWalking);
      csgInputCommand.setUnitVelocities(false);
      csgInputCommand.setForwardVelocity(desiredForwardVelocity);
      csgInputCommand.setLateralVelocity(desiredLateralVelocity);
      csgInputCommand.setTurnVelocity(desiredTurningVelocity);
   }

   private void configureContinuousHikingCommand()
   {
      // Setup variables to be published in the message
      boolean requestWalking = false;
      boolean walkForwards = false;
      boolean walkBackwards = false;
      double forwardJoystickValue = 0.0;
      double lateralJoystickValue = 0.0;
      double turningJoystickValue = 0.0;
      double deadband = 0.09;

      // Continuous hiking input values we get from the xbox controller
      if (currentControllerConnected)
      {
         requestWalking = currentController.getAxis(4) == 1.0; // This is the left trigger value (1.0 = pressed in)
         forwardJoystickValue = requestWalking ? DeadbandTools.applyDeadband(deadband, -currentController.getAxis(currentController.getMapping().axisLeftY)) : 0.0;
         lateralJoystickValue = requestWalking ? DeadbandTools.applyDeadband(deadband, -currentController.getAxis(currentController.getMapping().axisLeftX)) : 0.0;
         turningJoystickValue = requestWalking ? DeadbandTools.applyDeadband(deadband, -currentController.getAxis(currentController.getMapping().axisRightX)) : 0.0;
         walkForwards = requestWalking && forwardJoystickValue > 0.0;
         walkBackwards = requestWalking && forwardJoystickValue < 0.0;
      }

      configureContinuousHikingCommand(true, walkForwards, walkBackwards, forwardJoystickValue, lateralJoystickValue, turningJoystickValue);
   }

   private void configureContinuousHikingCommand(boolean enableContinuousHiking, boolean walkForwards, boolean walkBackwards, double forwardValue, double lateralValue, double turningValue)
   {
      continuousHikingCommand.setEnableContinuousHiking(enableContinuousHiking);
      continuousHikingCommand.setUseJoystickController(true);
      continuousHikingCommand.setWalkForwards(walkForwards);
      continuousHikingCommand.setWalkBackwards(walkBackwards);
      continuousHikingCommand.setForwardValue(forwardValue);
      continuousHikingCommand.setLateralValue(lateralValue);
      continuousHikingCommand.setTurningValue(turningValue);
   }

   public void sendStopWalkingCommands()
   {
      // Safety command for CSG
      csgInputCommand.setWalk(false);
      csgInputCommand.setForwardVelocity(0.0);
      csgInputCommand.setLateralVelocity(0.0);
      csgInputCommand.setTurnVelocity(0.0);

      // Safety command for CH
      continuousHikingCommand.setEnableContinuousHiking(false); //TODO what do we wanna do here?
      continuousHikingCommand.setUseJoystickController(false);
      continuousHikingCommand.setWalkForwards(false);
      continuousHikingCommand.setWalkBackwards(false);
      continuousHikingCommand.setForwardValue(0.0);
      continuousHikingCommand.setLateralValue(0.0);
      continuousHikingCommand.setSquareUpToGoal(true);

      switch (currentJoystickWalkingMode)
      {
         case CSG -> ros2CSGPublisherMap.publish(csgInputCommand);
         case CONTINUOUS_HIKING -> continuousHikingCommandPublisher.publish(continuousHikingCommand);
      }
   }

   public void shutDownXboxJoystick()
   {
      sendStopWalkingCommands();
   }

   private void setCSGCommandsToCurrentValues(ContinuousStepGeneratorStatusMessage csgStatusMessage)
   {
      csgParametersCommand.setSwingDuration(csgStatusMessage.getCurrentSwingDuration());
      csgParametersCommand.setTransferDuration(csgStatusMessage.getCurrentTransferDuration());

      csgParametersCommand.setSwingHeight(csgStatusMessage.getCurrentSwingHeight());
      csgParametersCommand.setMaxStepLength(csgStatusMessage.getCurrentMaxStepLength());
      csgParametersCommand.setDefaultStepWidth(csgStatusMessage.getCurrentDefaultStepWidth());
      csgParametersCommand.setMinStepWidth(csgStatusMessage.getCurrentMinStepWidth());
      csgParametersCommand.setMaxStepWidth(csgStatusMessage.getCurrentMaxStepWidth());
      csgParametersCommand.setTurnMaxAngleInward(csgStatusMessage.getCurrentTurnMaxAngleInward());
      csgParametersCommand.setTurnMaxAngleOutward(csgStatusMessage.getCurrentTurnMaxAngleOutward());
   }

   private void initializeCSGParametersCommand(ContinuousStepGeneratorParametersMessage csgParametersCommand, WalkingControllerParameters walkingControllerParameters)
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

   public void enableCSGPublishing()
   {
      // Stop walking before we switch to CSG
      if (currentJoystickWalkingMode.equals(JoystickWalkingMode.CONTINUOUS_HIKING))
      {
         configureContinuousHikingCommand(false, false, false, 0.0, 0.0, 0.0);
         continuousHikingCommandPublisher.publish(continuousHikingCommand);
      }

      currentJoystickWalkingMode = JoystickWalkingMode.CSG;
   }

   public void enableContinuousHikingPublishing()
   {
      // Stop walking before we switch to CH
      if (currentJoystickWalkingMode.equals(JoystickWalkingMode.CSG))
      {
         configureCSGInputCommand(false, 0.0, 0.0, 0.0);
         ros2CSGPublisherMap.publish(csgInputCommand);
      }

      currentJoystickWalkingMode = JoystickWalkingMode.CONTINUOUS_HIKING;
   }

   public JoystickWalkingMode getCurrentJoystickWalkingMode()
   {
      return currentJoystickWalkingMode;
   }
}
