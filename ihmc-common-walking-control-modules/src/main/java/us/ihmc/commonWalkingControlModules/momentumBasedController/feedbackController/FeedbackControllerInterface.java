package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;

public interface FeedbackControllerInterface
{
   void initialize();

   void setEnabled(boolean isEnabled);

   default void setImpedanceEnabled(boolean isImpedanceEnabled){
      // Empty default implementation to avoid breaking existing implementations.
   };

   void computeInverseDynamics();

   void computeInverseKinematics();

   void computeVirtualModelControl();

   void computeAchievedAcceleration();

   default void computeAchievedVelocity()
   {
      // Empty default implementation to avoid breaking existing implementations.
   }

   boolean isEnabled();

   default boolean isImpedanceEnabled() {
      return false;
   };

   InverseDynamicsCommand<?> getInverseDynamicsOutput();

   InverseKinematicsCommand<?> getInverseKinematicsOutput();

   VirtualModelControlCommand<?> getVirtualModelControlOutput();
}