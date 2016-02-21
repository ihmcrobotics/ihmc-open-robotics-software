package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;

public interface FeedbackControllerInterface
{
   public abstract void initialize();

   public abstract void setEnabled(boolean isEnabled);

   public abstract void compute();

   public abstract boolean isEnabled();

   public abstract InverseDynamicsCommand<?> getOutput();
}