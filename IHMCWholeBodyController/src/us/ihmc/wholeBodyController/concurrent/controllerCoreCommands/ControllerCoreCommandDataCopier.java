package us.ihmc.wholeBodyController.concurrent.controllerCoreCommands;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandInterface;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ControllerCoreCommandDataCopier implements ControllerCoreCommandInterface
{
   private final InverseDynamicsCommandDataCopier inverseDynamicsCommandDataCopier = new InverseDynamicsCommandDataCopier();
   private final InverseDynamicsCommandDataCopier virtualModelControlCommandDataCopier = new InverseDynamicsCommandDataCopier();
   private final FeedbackControlCommandDataCopier feedbackControlCommandDataCopier = new FeedbackControlCommandDataCopier();
   private final InverseKinematicsCommandDataCopier inverseKinematicsCommandDataCopier = new InverseKinematicsCommandDataCopier();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private WholeBodyControllerCoreMode wholeBodyControllerCoreMode;

   public ControllerCoreCommandDataCopier()
   {
   }

   public void copyDataFrom(ControllerCoreCommandInterface controllerCoreCommand)
   {
      wholeBodyControllerCoreMode = controllerCoreCommand.getControllerCoreMode();
      inverseDynamicsCommandDataCopier.copyFromOther(controllerCoreCommand.getInverseDynamicsCommandList());
      virtualModelControlCommandDataCopier.copyFromOther(controllerCoreCommand.getVirtualModelControlCommandList());
      feedbackControlCommandDataCopier.copyFromOther(controllerCoreCommand.getFeedbackControlCommandList());
      inverseKinematicsCommandDataCopier.copyFromOther(controllerCoreCommand.getInverseKinematicsCommandList());
      lowLevelOneDoFJointDesiredDataHolder.overwriteWith(controllerCoreCommand.getLowLevelOneDoFJointDesiredDataHolder());
   }

   public void retrieveRigidBodiesFromName(Map<String, RigidBody> nameToRigidBodyMap)
   {
      inverseDynamicsCommandDataCopier.retrieveRigidBodiesFromName(nameToRigidBodyMap);
      virtualModelControlCommandDataCopier.retrieveRigidBodiesFromName(nameToRigidBodyMap);
      feedbackControlCommandDataCopier.retrieveRigidBodiesFromName(nameToRigidBodyMap);
      inverseKinematicsCommandDataCopier.retrieveRigidBodiesFromName(nameToRigidBodyMap);
   }

   public void retrieveJointsFromName(Map<String, OneDoFJoint> nameToJointMap)
   {
      inverseDynamicsCommandDataCopier.retrieveJointsFromName(nameToJointMap);
      virtualModelControlCommandDataCopier.retrieveJointsFromName(nameToJointMap);
      feedbackControlCommandDataCopier.retrieveJointsFromName(nameToJointMap);
      inverseKinematicsCommandDataCopier.retrieveJointsFromName(nameToJointMap);
      lowLevelOneDoFJointDesiredDataHolder.retrieveJointsFromName(nameToJointMap);
   }

   @Override
   public InverseDynamicsCommandList getInverseDynamicsCommandList()
   {
      return inverseDynamicsCommandDataCopier.getInverseDynamicsCommandList();
   }

   @Override
   public InverseDynamicsCommandList getVirtualModelControlCommandList()
   {
      return virtualModelControlCommandDataCopier.getInverseDynamicsCommandList();
   }

   @Override
   public FeedbackControlCommandList getFeedbackControlCommandList()
   {
      return feedbackControlCommandDataCopier.getFeedbackControlCommandList();
   }

   @Override
   public InverseKinematicsCommandList getInverseKinematicsCommandList()
   {
      return inverseKinematicsCommandDataCopier.getInverseKinematicsCommandList();
   }

   @Override
   public LowLevelOneDoFJointDesiredDataHolderReadOnly getLowLevelOneDoFJointDesiredDataHolder()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   @Override
   public WholeBodyControllerCoreMode getControllerCoreMode()
   {
      return wholeBodyControllerCoreMode;
   }
}
