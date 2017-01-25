package us.ihmc.commonWalkingControlModules.controlModules.chest;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.GoHomeCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;

public interface ChestOrientationManagerInterface
{
   public void initialize();

   public void compute();

   public void holdCurrentOrientation();

   public void handleChestTrajectoryCommand(ChestTrajectoryCommand command);

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command);

   public void handleGoHomeCommand(GoHomeCommand command);

   public void goToHomeFromCurrentDesired(double trajectoryTime);

   public void goToHomeFromCurrent(double trajectoryTime);

   public InverseDynamicsCommand<?> getInverseDynamicsCommand();

   public FeedbackControlCommand<?> getFeedbackControlCommand();
}
