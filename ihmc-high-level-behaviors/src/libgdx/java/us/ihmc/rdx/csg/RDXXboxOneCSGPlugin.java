package us.ihmc.rdx.csg;

import com.badlogic.gdx.controllers.Controller;
import com.badlogic.gdx.controllers.ControllerListener;
import com.badlogic.gdx.controllers.Controllers;
import controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.CSGROS2CommunicationHelper;
import us.ihmc.commons.DeadbandTools;
import us.ihmc.ros2.ROS2Node;

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
public class RDXXboxOneCSGPlugin
{
   public static final double DEFAULT_PARAMETER_INCREMENT = 0.01;
   private static final boolean DEFAULT_USE_DEADMAN_SWITCH = true;

   // Stuff related to the Xbox controller itself
   private Controller currentController;
   private final ControllerListener controllerListener;
   private boolean currentControllerConnected = false;
   private boolean controllerListenerHasBeenAdded = false;
   private boolean publishCSGInputCommand = false;

   // CSG ROS communication helper
   private final CSGROS2CommunicationHelper communicationHelper;

   // CSG command and status messages
   private final ContinuousStepGeneratorInputMessage csgInputCommand;
   private final ContinuousStepGeneratorParametersMessage csgParametersCommand;
   private final ContinuousStepGeneratorStatusMessage csgStatusMessage;

   public RDXXboxOneCSGPlugin(DRCRobotModel robotModel, ROS2Node ros2Node)
   {
      this(new CSGROS2CommunicationHelper(robotModel.getSimpleRobotName(), ros2Node, robotModel.getWalkingControllerParameters()));
   }

   public RDXXboxOneCSGPlugin(CSGROS2CommunicationHelper communicationHelper)
   {
      this(communicationHelper, DEFAULT_PARAMETER_INCREMENT, DEFAULT_USE_DEADMAN_SWITCH);
   }

   public RDXXboxOneCSGPlugin(CSGROS2CommunicationHelper communicationHelper, double parameterIncrement, boolean useDeadmanSwitch)
   {
      this.communicationHelper = communicationHelper;

      csgInputCommand = communicationHelper.getCSGInputCommand();
      csgParametersCommand = communicationHelper.getCSGParametersCommand();
      csgStatusMessage = communicationHelper.getCSGStatusMessage();

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

               communicationHelper.publish(csgParametersCommand);
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
            if (controller != null)
            {
               if (axisCode == 4.0 && controller.getAxis(4) == 1.0)
               {
                  publishCSGInputCommand = true;
               }
               else if (axisCode == 4.0 && controller.getAxis(4) != 1.0 && publishCSGInputCommand)
               {
                  sendStopWalkingCommands();
                  publishCSGInputCommand = false;
               }
            }

            return false;
         }
      };
   }

   public void updateAndPublish()
   {
      update(csgInputCommand);

      if (publishCSGInputCommand)
         communicationHelper.publishAtThrottledRate(csgInputCommand);
   }

   public void update(ContinuousStepGeneratorInputMessage csgInputCommandToPack)
   {
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

      csgInputCommandToPack.setWalk(requestWalking);
      csgInputCommandToPack.setUnitVelocities(false);
      csgInputCommandToPack.setForwardVelocity(forwardJoystickValue);
      csgInputCommandToPack.setLateralVelocity(lateralJoystickValue);
      csgInputCommandToPack.setTurnVelocity(turningJoystickValue);
   }

   public void sendStopWalkingCommands()
   {
      csgInputCommand.setWalk(false);
      csgInputCommand.setForwardVelocity(0.0);
      csgInputCommand.setLateralVelocity(0.0);
      csgInputCommand.setTurnVelocity(0.0);

      communicationHelper.publish(csgInputCommand);
   }

   public void shutDownXboxJoystick()
   {
      sendStopWalkingCommands();
   }
}
