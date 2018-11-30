package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;

public interface FeedbackControllerInterface
{
   void initialize();

   void setEnabled(boolean isEnabled);

   void computeInverseDynamics();

   void computeInverseKinematics();

   void computeVirtualModelControl();

   void computeAchievedAcceleration();

   boolean isEnabled();

   InverseDynamicsCommand<?> getInverseDynamicsOutput();

   InverseKinematicsCommand<?> getInverseKinematicsOutput();

   VirtualModelControlCommand<?> getVirtualModelControlOutput();
}