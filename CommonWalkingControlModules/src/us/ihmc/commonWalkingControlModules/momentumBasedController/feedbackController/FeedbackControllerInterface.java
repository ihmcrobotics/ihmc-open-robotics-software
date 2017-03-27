package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;

public interface FeedbackControllerInterface
{
   public abstract void initialize();

   public abstract void setEnabled(boolean isEnabled);

   public abstract void computeInverseDynamics();

   public abstract void computeInverseKinematics();

   public abstract void computeVirtualModelControl();

   public abstract void computeAchievedAcceleration();

   public abstract boolean isEnabled();

   public abstract InverseDynamicsCommand<?> getInverseDynamicsOutput();

   public abstract InverseKinematicsCommand<?> getInverseKinematicsOutput();

   public abstract InverseDynamicsCommand<?> getVirtualModelControlOutput();
}