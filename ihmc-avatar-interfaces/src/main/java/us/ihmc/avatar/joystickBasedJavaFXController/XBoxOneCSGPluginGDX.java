package us.ihmc.avatar.joystickBasedJavaFXController;

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
import us.ihmc.log.LogTools;
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
public class XBoxOneCSGPluginGDX
{
   public static final double DEFAULT_PARAMETER_INCREMENT = 0.01;
   private static final boolean DEFAULT_USE_DEADMAN_SWITCH = true;

   private final double parameterIncrement;
   private final boolean useDeadmanSwitch;

   private Controller currentController;
   private final ControllerListener controllerListener;
   private boolean currentControllerConnected = false;
   private boolean controllerListenerHasBeenAdded = false;

   private final ROS2ControllerPublisherMap ros2ControllerPublisherMap;
   private final ContinuousStepGeneratorInputMessage csgInputCommand;
   private final ContinuousStepGeneratorParametersMessage csgParametersCommand;

   private final QueuedROS2Subscription<ContinuousStepGeneratorStatusMessage> csgStatusSubscription;
   private final ContinuousStepGeneratorStatusMessage csgStatusMessage = new ContinuousStepGeneratorStatusMessage();

   public XBoxOneCSGPluginGDX(DRCRobotModel robotModel, ROS2Node ros2Node) throws JoystickNotFoundException
   {
      this(robotModel, ros2Node, DEFAULT_PARAMETER_INCREMENT, DEFAULT_USE_DEADMAN_SWITCH);
   }

   public XBoxOneCSGPluginGDX(DRCRobotModel robotModel, ROS2Node ros2Node, double parameterIncrement, boolean useDeadmanSwitch) throws JoystickNotFoundException
   {
      this.parameterIncrement = parameterIncrement;
      this.useDeadmanSwitch = useDeadmanSwitch;

      ros2ControllerPublisherMap = new ROS2ControllerPublisherMap(ros2Node, robotModel.getSimpleRobotName());
      csgInputCommand = new ContinuousStepGeneratorInputMessage();
      csgParametersCommand = new ContinuousStepGeneratorParametersMessage();
      csgStatusSubscription = ros2Node.createQueuedSubscription(HumanoidControllerAPI.getTopic(ContinuousStepGeneratorStatusMessage.class, robotModel.getSimpleRobotName()), 10);

      configureCSGParameters(csgParametersCommand, robotModel.getWalkingControllerParameters());

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

            csgInputCommand.setWalk(false);
            csgInputCommand.setForwardVelocity(0.0);
            csgInputCommand.setLateralVelocity(0.0);
            csgInputCommand.setTurnVelocity(0.0);

            ros2ControllerPublisherMap.publish(csgInputCommand);
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

               ros2ControllerPublisherMap.publish(csgParametersCommand);
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
      if (csgStatusSubscription.flushAndGetLatest(csgStatusMessage))
         setCSGCommandsToCurrentValues(csgStatusMessage);

      boolean newControllerConnected = false;
      if (currentController != null && currentController != Controllers.getCurrent())
         newControllerConnected = true;

      currentController = Controllers.getCurrent();
      currentControllerConnected = currentController != null;

      if ((!controllerListenerHasBeenAdded || newControllerConnected) && currentControllerConnected)
      {
         currentController.addListener(controllerListener);
         controllerListenerHasBeenAdded = true;
      }

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

      csgInputCommand.setWalk(requestWalking);
      csgInputCommand.setUnitVelocities(false);
      csgInputCommand.setForwardVelocity(forwardJoystickValue);
      csgInputCommand.setLateralVelocity(lateralJoystickValue);
      csgInputCommand.setTurnVelocity(turningJoystickValue);
      ros2ControllerPublisherMap.publish(csgInputCommand);
   }

   public void shutDownXboxJoystick()
   {
      csgInputCommand.setWalk(false);
      csgInputCommand.setForwardVelocity(0.0);
      csgInputCommand.setLateralVelocity(0.0);
      csgInputCommand.setTurnVelocity(0.0);

      ros2ControllerPublisherMap.publish(csgInputCommand);
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
