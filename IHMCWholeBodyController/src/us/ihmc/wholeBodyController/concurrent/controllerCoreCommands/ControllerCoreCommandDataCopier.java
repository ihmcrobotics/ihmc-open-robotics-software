package us.ihmc.wholeBodyController.concurrent.controllerCoreCommands;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.ControllerCoreCommandInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DesiredOneDoFJointTorqueHolder;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ControllerCoreCommandDataCopier implements ControllerCoreCommandInterface
{
   private final InverseDynamicsCommandDataCopier inverseDynamicsCommandDataCopier = new InverseDynamicsCommandDataCopier();
   private final FeedbackControlCommandDataCopier feedbackControlCommandDataCopier = new FeedbackControlCommandDataCopier();
   private final DesiredOneDoFJointTorqueHolder desiredOneDoFJointTorqueHolder = new DesiredOneDoFJointTorqueHolder();

   private boolean enableControllerCore;

   public ControllerCoreCommandDataCopier()
   {
   }

   public void copyDataFrom(ControllerCoreCommandInterface controllerCoreCommand)
   {
      enableControllerCore = controllerCoreCommand.enableControllerCore();
      if (enableControllerCore)
      {
         inverseDynamicsCommandDataCopier.copyFromOther(controllerCoreCommand.getInverseDynamicsCommandList());
         feedbackControlCommandDataCopier.copyFromOther(controllerCoreCommand.getFeedbackControlCommandList());
      }
      else
      {
         desiredOneDoFJointTorqueHolder.set(controllerCoreCommand.geDesiredOneDoFJointTorqueHolder());
      }
   }

   public void retrieveRigidBodiesFromName(Map<String, RigidBody> nameToRigidBodyMap)
   {
      if (enableControllerCore)
      {
         inverseDynamicsCommandDataCopier.retrieveRigidBodiesFromName(nameToRigidBodyMap);
         feedbackControlCommandDataCopier.retrieveRigidBodiesFromName(nameToRigidBodyMap);
      }
   }

   public void retrieveJointsFromName(Map<String, OneDoFJoint> nameToJointMap)
   {
      if (enableControllerCore)
      {
         inverseDynamicsCommandDataCopier.retrieveJointsFromName(nameToJointMap);
         feedbackControlCommandDataCopier.retrieveJointsFromName(nameToJointMap);
      }
      else
      {
         desiredOneDoFJointTorqueHolder.retrieveJointsFromName(nameToJointMap);
      }
   }

   @Override
   public InverseDynamicsCommandList getInverseDynamicsCommandList()
   {
      if (enableControllerCore)
         return inverseDynamicsCommandDataCopier.getInverseDynamicsCommandList();
      else
         return null;
   }

   @Override
   public FeedbackControlCommandList getFeedbackControlCommandList()
   {
      if (enableControllerCore)
         return feedbackControlCommandDataCopier.getFeedbackControlCommandList();
      else
         return null;
   }

   @Override
   public DesiredOneDoFJointTorqueHolder geDesiredOneDoFJointTorqueHolder()
   {
      if (enableControllerCore)
         return null;
      else
         return desiredOneDoFJointTorqueHolder;
   }

   @Override
   public boolean enableControllerCore()
   {
      return enableControllerCore;
   }
}
