package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public interface ControllerCoreCommandInterface
{
   void clear();

   WholeBodyControllerCoreMode getControllerCoreMode();

   boolean isReinitializationRequested();

   InverseDynamicsCommandList getInverseDynamicsCommandList();

   VirtualModelControlCommandList getVirtualModelControlCommandList();

   FeedbackControlCommandList getFeedbackControlCommandList();

   InverseKinematicsCommandList getInverseKinematicsCommandList();

   JointDesiredOutputListReadOnly getLowLevelOneDoFJointDesiredDataHolder();

   default boolean equals(ControllerCoreCommandInterface other)
   {
      if (other == null)
      {
         return false;
      }
      else if (other == this)
      {
         return true;
      }
      else
      {
         if (getControllerCoreMode() != other.getControllerCoreMode())
            return false;
         if (isReinitializationRequested() != other.isReinitializationRequested())
            return false;
         if (!getInverseDynamicsCommandList().equals(other.getInverseDynamicsCommandList()))
            return false;
         if (!getInverseKinematicsCommandList().equals(other.getInverseKinematicsCommandList()))
            return false;
         if (!getVirtualModelControlCommandList().equals(other.getVirtualModelControlCommandList()))
            return false;
         if (!getFeedbackControlCommandList().equals(other.getFeedbackControlCommandList()))
            return false;
         if (!getLowLevelOneDoFJointDesiredDataHolder().equals(other.getLowLevelOneDoFJointDesiredDataHolder()))
            return false;
         return true;
      }
   }
}