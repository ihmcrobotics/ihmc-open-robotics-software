package us.ihmc.rdx.velocityBasedLocomotion;

import com.badlogic.gdx.controllers.Controller;
import com.badlogic.gdx.controllers.ControllerListener;
import com.badlogic.gdx.controllers.Controllers;
import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.AvatarStepGeneratorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.joystickBasedLocomotion.AbstractJoystickLocomotionPlugin;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.CSGROS2CommunicationHelper;
import us.ihmc.commons.DeadbandTools;
import us.ihmc.ros2.ROS2Node;

/**
 * This is a plugin for an RDX application. It can convert inputs coming from an Xbox One
 * handheld controller into messages that get sent to the {@link AvatarControllerThread} and
 * {@link AvatarStepGeneratorThread} in order to enact and control walking movements.
 * <p>
 * For this plugin to work, ensure the Xbox controller is connected to the computer running
 * the RDX application via cable or wireless receiver dongle. Also make sure that the
 * {@link #update} method is called regularly every tick. It is additionally recommenced to
 * call {@link #shutdown()} upon RDX application termination.
 *
 * @author Stefan Fasano
 */
public class XBoxOneRDXLocomotionPlugin extends AbstractJoystickLocomotionPlugin
{
   public static final double DEFAULT_PARAMETER_INCREMENT = 0.01;

   // Stuff related to the Xbox controller itself
   private Controller currentController;
   private final ControllerListener xboxOneControllerListener;
   private boolean currentControllerConnected = false;
   private boolean controllerListenerHasBeenAdded = false;

   public XBoxOneRDXLocomotionPlugin(DRCRobotModel robotModel, ROS2Node ros2Node)
   {
      this(robotModel, ros2Node, DEFAULT_PARAMETER_INCREMENT);
   }

   public XBoxOneRDXLocomotionPlugin(DRCRobotModel robotModel, ROS2Node ros2Node, double parameterIncrement)
   {
      super(robotModel, ros2Node);

      xboxOneControllerListener = new ControllerListener()
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

//               else if (buttonCode == controller.getMapping().buttonDpadDown)
//                  csgParametersCommand.setSwingHeight(csgStatusMessage.getCurrentSwingHeight() - parameterIncrement);
               else if (buttonCode == controller.getMapping().buttonDpadUp)
                  directionalControlInputMessage.setJump(true);

               csgROS2CommunicationHelper.publish(csgParametersCommand);
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

   @Override
   public void update()
   {
      updateCommandsFromRC();

      if (publisherThrottler.run() && !heartBeat.isExpired(PUBLISHER_HEARTBEAT_DURATION))
      {
         publish();
         directionalControlInputMessage.setJump(false);
      }
   }

   private void updateCommandsFromRC()
   {
      boolean newControllerConnected = currentController != null && currentController != Controllers.getCurrent();

      currentController = Controllers.getCurrent();
      currentControllerConnected = currentController != null;

      if ((!controllerListenerHasBeenAdded || newControllerConnected) && currentControllerConnected)
      {
         currentController.addListener(xboxOneControllerListener);
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
         if (requestWalking)
         {
            forwardJoystickValue = DeadbandTools.applyDeadband(deadband, -currentController.getAxis(currentController.getMapping().axisLeftY));
            lateralJoystickValue = DeadbandTools.applyDeadband(deadband, -currentController.getAxis(currentController.getMapping().axisLeftX));
            turningJoystickValue = DeadbandTools.applyDeadband(deadband, -currentController.getAxis(currentController.getMapping().axisRightX));
         }
      }

      // Pack messages with desired walking commands
      setDesiredWalkingCommands(requestWalking, forwardJoystickValue, lateralJoystickValue, turningJoystickValue, true);
   }

   public CSGROS2CommunicationHelper getCSGROS2CommunicationHelper()
   {
      return csgROS2CommunicationHelper;
   }
}
