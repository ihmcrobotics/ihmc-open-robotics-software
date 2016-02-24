package us.ihmc.wholeBodyController.concurrent.controllerCoreCommands;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.ControllerCoreCommandInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.lowLevelControl.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.lowLevelControl.LowLevelOneDoFJointDesiredDataHolderInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommandList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ControllerCoreCommandDataCopier implements ControllerCoreCommandInterface
{
   private final InverseDynamicsCommandDataCopier inverseDynamicsCommandDataCopier = new InverseDynamicsCommandDataCopier();
   private final FeedbackControlCommandDataCopier feedbackControlCommandDataCopier = new FeedbackControlCommandDataCopier();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private boolean enableControllerCore;

   public ControllerCoreCommandDataCopier()
   {
   }

   public void copyDataFrom(ControllerCoreCommandInterface controllerCoreCommand)
   {
      inverseDynamicsCommandDataCopier.copyFromOther(controllerCoreCommand.getInverseDynamicsCommandList());
      feedbackControlCommandDataCopier.copyFromOther(controllerCoreCommand.getFeedbackControlCommandList());
      lowLevelOneDoFJointDesiredDataHolder.overwriteWith(controllerCoreCommand.getLowLevelOneDoFJointDesiredDataHolder());
   }

   public void retrieveRigidBodiesFromName(Map<String, RigidBody> nameToRigidBodyMap)
   {
      inverseDynamicsCommandDataCopier.retrieveRigidBodiesFromName(nameToRigidBodyMap);
      feedbackControlCommandDataCopier.retrieveRigidBodiesFromName(nameToRigidBodyMap);
   }

   public void retrieveJointsFromName(Map<String, OneDoFJoint> nameToJointMap)
   {
      inverseDynamicsCommandDataCopier.retrieveJointsFromName(nameToJointMap);
      feedbackControlCommandDataCopier.retrieveJointsFromName(nameToJointMap);
      lowLevelOneDoFJointDesiredDataHolder.retrieveJointsFromName(nameToJointMap);
   }

   @Override
   public InverseDynamicsCommandList getInverseDynamicsCommandList()
   {
      return inverseDynamicsCommandDataCopier.getInverseDynamicsCommandList();
   }

   @Override
   public FeedbackControlCommandList getFeedbackControlCommandList()
   {
      return feedbackControlCommandDataCopier.getFeedbackControlCommandList();
   }

   @Override
   public LowLevelOneDoFJointDesiredDataHolderInterface getLowLevelOneDoFJointDesiredDataHolder()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   @Override
   public boolean enableControllerCore()
   {
      return enableControllerCore;
   }
}
