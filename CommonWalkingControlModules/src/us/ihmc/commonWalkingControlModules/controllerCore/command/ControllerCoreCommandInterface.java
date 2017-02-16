package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;

public interface ControllerCoreCommandInterface
{
   public abstract InverseDynamicsCommandList getInverseDynamicsCommandList();
   public abstract InverseDynamicsCommandList getVirtualModelControlCommandList();
   public abstract FeedbackControlCommandList getFeedbackControlCommandList();
   public abstract InverseKinematicsCommandList getInverseKinematicsCommandList();
   public abstract LowLevelOneDoFJointDesiredDataHolderReadOnly getLowLevelOneDoFJointDesiredDataHolder();
   public abstract WholeBodyControllerCoreMode getControllerCoreMode();
}