package us.ihmc.rdx.ui;

import com.badlogic.gdx.controllers.Controller;
import com.badlogic.gdx.controllers.Controllers;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXContinousStepGenerator;
import us.ihmc.rdx.vr.RDXVRController;
import us.ihmc.robotics.math.DeadbandTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXJoystickBasedStepping
{
   private final RDXContinousStepGenerator continousStepGenerator;

   private Controller currentController;
   private boolean currentControllerConnected;
   private RDXVRController leftVRController;
   private RDXVRController rightVRController;
   private boolean userNotClickingAnImGuiPanel;

   public RDXJoystickBasedStepping(DRCRobotModel robotModel)
   {
      continousStepGenerator = new RDXContinousStepGenerator(robotModel);
   }

   public void create(RDXBaseUI baseUI, ROS2ControllerHelper controllerHelper, ROS2SyncedRobotModel syncedRobot)
   {
      continousStepGenerator.create(controllerHelper, syncedRobot);

      leftVRController = baseUI.getVRManager().getContext().getController(RobotSide.LEFT);
      rightVRController = baseUI.getVRManager().getContext().getController(RobotSide.RIGHT);
      baseUI.getVRManager().getContext().addVRInputProcessor(context ->
      {
         userNotClickingAnImGuiPanel = true;
         for (RobotSide side : RobotSide.values)
            userNotClickingAnImGuiPanel =  userNotClickingAnImGuiPanel && context.getController(side).getSelectedPick() == null;
      });
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
      continousStepGenerator.setWalkingModeActive(walkingModeEnabled);
      continousStepGenerator.setNormalizedForwardVelocity(DeadbandTools.applyDeadband(deadband, forwardJoystickValue));
      continousStepGenerator.setNormalizedLateralVelocity(DeadbandTools.applyDeadband(deadband, lateralJoystickValue));
      continousStepGenerator.setNormalizedTurningVelocity(DeadbandTools.applyDeadband(deadband, turningJoystickValue));
      continousStepGenerator.update();
   }

   public void renderImGuiWidgets()
   {
      continousStepGenerator.renderImGuiWidgets();
   }

   public void destroy()
   {

   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }
}
