package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;

public class PelvisOrientationManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public enum PelvisOrientationControlMode
   {
      WALKING_CONTROLLER,
      USER
   }

   private final EnumYoVariable<PelvisOrientationControlMode> pelvisOrientationControlMode = new EnumYoVariable<>("pelvisOrientationControlMode", registry,
                                                                                                                  PelvisOrientationControlMode.class);

   private final ControllerPelvisOrientationManager walkingManager;
   private final UserPelvisOrientationManager userManager;

   private final PelvisOrientationTrajectoryCommand tempPelvisOrientationTrajectoryCommand = new PelvisOrientationTrajectoryCommand();

   public PelvisOrientationManager(YoOrientationPIDGainsInterface gains, HighLevelHumanoidControllerToolbox controllerToolbox,
                                   YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      walkingManager = new ControllerPelvisOrientationManager(gains, controllerToolbox, registry);
      userManager = new UserPelvisOrientationManager(gains, controllerToolbox, registry);

      pelvisOrientationControlMode.set(PelvisOrientationControlMode.WALKING_CONTROLLER);
   }

   public void setWeights(Vector3D weight)
   {
      walkingManager.setWeights(weight);
      userManager.setWeights(weight);
   }

   public void compute()
   {
      walkingManager.compute();
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      // TODO
   }

   public void prepareForLocomotion()
   {
      // TODO
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return walkingManager.getInverseDynamicsCommand();
   }

   public OrientationFeedbackControlCommand getFeedbackControlCommand()
   {
      return walkingManager.getFeedbackControlCommand();
   }

   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      walkingManager.goToHomeFromCurrentDesired(trajectoryTime);
   }

   public void setTrajectoryTime(double transferTime)
   {
      walkingManager.setTrajectoryTime(transferTime);
   }

   public void moveToAverageInSupportFoot(RobotSide supportSide)
   {
      walkingManager.moveToAverageInSupportFoot(supportSide);
   }

   public void resetOrientationOffset()
   {
      walkingManager.resetOrientationOffset();
   }

   public void setToHoldCurrentDesiredInMidFeetZUpFrame()
   {
      walkingManager.setToHoldCurrentDesiredInMidFeetZUpFrame();
   }

   public void setToHoldCurrentDesiredInSupportFoot(RobotSide supportSide)
   {
      walkingManager.setToHoldCurrentDesiredInSupportFoot(supportSide);
   }

   public void setToHoldCurrentInWorldFrame()
   {
      walkingManager.setToHoldCurrentInWorldFrame();
   }

   public void setToZeroInMidFeetZUpFrame()
   {
      walkingManager.setToZeroInMidFeetZUpFrame();
   }

   public void setWithUpcomingFootstep(Footstep upcomingFootstep)
   {
      walkingManager.setWithUpcomingFootstep(upcomingFootstep);
   }

   public void handlePelvisOrientationTrajectoryCommands(PelvisOrientationTrajectoryCommand command)
   {
      userManager.handlePelvisOrientationTrajectoryCommands(command);
   }

   public void handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      tempPelvisOrientationTrajectoryCommand.set(command);
      handlePelvisOrientationTrajectoryCommands(tempPelvisOrientationTrajectoryCommand);
   }
}
