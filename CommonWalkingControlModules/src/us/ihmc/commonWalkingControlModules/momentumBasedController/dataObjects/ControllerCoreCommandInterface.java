package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DesiredOneDoFJointTorqueHolder;

public interface ControllerCoreCommandInterface
{

   InverseDynamicsCommandList getInverseDynamicsCommandList();

   FeedbackControlCommandList getFeedbackControlCommandList();

   DesiredOneDoFJointTorqueHolder geDesiredOneDoFJointTorqueHolder();

   boolean enableControllerCore();

}