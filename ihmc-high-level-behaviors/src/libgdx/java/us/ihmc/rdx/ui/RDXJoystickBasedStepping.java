package us.ihmc.rdx.ui;

import com.badlogic.gdx.controllers.Controller;
import com.badlogic.gdx.controllers.Controllers;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXContinousStepGenerator;
import us.ihmc.rdx.vr.RDXVRController;
import us.ihmc.robotics.math.DeadbandTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXJoystickBasedStepping extends RDXContinousStepGenerator
{
   private Controller currentController;
   private boolean currentControllerConnected;
   private RDXVRController leftVRController;
   private RDXVRController rightVRController;
   private boolean userNotClickingAnImGuiPanel;

   public RDXJoystickBasedStepping(DRCRobotModel robotModel)
   {
      super(robotModel);
   }

   public void create(RDXBaseUI baseUI, ROS2ControllerHelper controllerHelper, ROS2SyncedRobotModel syncedRobot)
   {
      super.create(controllerHelper, syncedRobot);

      leftVRController = baseUI.getVRManager().getContext().getController(RobotSide.LEFT);
      rightVRController = baseUI.getVRManager().getContext().getController(RobotSide.RIGHT);
      baseUI.getVRManager().getContext().addVRInputProcessor(context ->
      {
         userNotClickingAnImGuiPanel = true;
         for (RobotSide side : RobotSide.values)
            userNotClickingAnImGuiPanel =  userNotClickingAnImGuiPanel && context.getController(side).getSelectedPick() == null;
      });
   }

   public void update()
   {
      update(true);
   }

   public void update(boolean enabled)
   {
      currentController = Controllers.getCurrent();
      currentControllerConnected = currentController != null;

      boolean walkingModeEnabled = false;
      double forwardJoystickValue = 0.0;
      double lateralJoystickValue = 0.0;
      double turningJoystickValue = 0.0;

      if (enabled && (currentControllerConnected || (leftVRController.isConnected() && rightVRController.isConnected())))
      {
         if (currentControllerConnected)
         {
            walkingModeEnabled = currentController.getButton(currentController.getMapping().buttonR1);
            forwardJoystickValue = -currentController.getAxis(currentController.getMapping().axisLeftY);
            lateralJoystickValue = -currentController.getAxis(currentController.getMapping().axisLeftX);
            turningJoystickValue = -currentController.getAxis(currentController.getMapping().axisRightX);
         }

         if (rightVRController.isConnected())
         {
            walkingModeEnabled = rightVRController.getClickTriggerActionData().bState() && userNotClickingAnImGuiPanel;
            turningJoystickValue = -rightVRController.getJoystickActionData().x();
         }

         if (leftVRController.isConnected())
         {
            forwardJoystickValue = leftVRController.getJoystickActionData().y();
            lateralJoystickValue = -leftVRController.getJoystickActionData().x();
         }
      }

      double deadband = 0.1;
      setWalkingModeActive(walkingModeEnabled);
      setNormalizedForwardVelocity(DeadbandTools.applyDeadband(deadband, forwardJoystickValue));
      setNormalizedLateralVelocity(DeadbandTools.applyDeadband(deadband, lateralJoystickValue));
      setNormalizedTurningVelocity(DeadbandTools.applyDeadband(deadband, turningJoystickValue));
      super.update();
   }
}
