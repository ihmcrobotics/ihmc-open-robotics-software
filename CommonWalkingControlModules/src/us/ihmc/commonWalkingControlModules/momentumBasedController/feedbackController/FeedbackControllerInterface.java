package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;

public interface FeedbackControllerInterface
{
   public abstract void initialize();

   public abstract void setEnabled(boolean isEnabled);

   public abstract void compute();

   public abstract void computeAchievedAcceleration();

   public abstract boolean isEnabled();

   public abstract InverseDynamicsCommand<?> getOutput();
}