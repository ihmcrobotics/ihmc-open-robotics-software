package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;

public class ControllerCoreCommandDataHolder implements ControllerCoreCommandInterface
{

   private boolean hasInverseDynamicsCommandList = false;
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();

   private boolean hasInverseKinematicsCommandList = false;
   private final InverseKinematicsCommandList inverseKinematicsCommandList = new InverseKinematicsCommandList();

   private boolean hasVirtualModelControlCommandList = false;
   private final VirtualModelControlCommandList virtualModelControlCommandList = new VirtualModelControlCommandList();

   private boolean hasFeedbackControlCommandList = false;
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   private boolean hasLowLevelOneDoFJointDesiredDataHolder = false;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private boolean hasControllerCoreMode = false;
   private WholeBodyControllerCoreMode controllerCoreMode;

   private boolean reinitialize = false;

   public ControllerCoreCommandDataHolder()
   {
      clear();
   }

   public void clear()
   {
      hasControllerCoreMode = false;
      hasFeedbackControlCommandList = false;
      hasInverseDynamicsCommandList = false;
      hasVirtualModelControlCommandList = false;
      hasInverseKinematicsCommandList = false;
      hasLowLevelOneDoFJointDesiredDataHolder = false;

      inverseDynamicsCommandList.clear();
      feedbackControlCommandList.clear();
      inverseKinematicsCommandList.clear();
      virtualModelControlCommandList.clear();
      lowLevelOneDoFJointDesiredDataHolder.clear();
      reinitialize = false;
   }

   public void set(ControllerCoreCommandDataHolder other)
   {
      hasInverseDynamicsCommandList = true;
      hasInverseKinematicsCommandList = true;
      hasVirtualModelControlCommandList = true;
      hasControllerCoreMode = true;
      hasFeedbackControlCommandList = true;
      hasLowLevelOneDoFJointDesiredDataHolder = true;

      this.inverseDynamicsCommandList.set(other.getInverseDynamicsCommandList());
      this.inverseKinematicsCommandList.set(other.getInverseKinematicsCommandList());
      this.virtualModelControlCommandList.set(other.getVirtualModelControlCommandList());
      this.controllerCoreMode = other.getControllerCoreMode();
      this.feedbackControlCommandList.set(other.getFeedbackControlCommandList());
      this.lowLevelOneDoFJointDesiredDataHolder.set(other.getLowLevelOneDoFJointDesiredDataHolder());
      reinitialize = other.isReinitializationRequested();
   }

   public void setControllerCoreCommandDataHolder(ControllerCoreCommand controllerCoreCommand)
   {
      setControllerCoreMode(controllerCoreCommand.getControllerCoreMode());

      setInverseDynamicsCommandList(controllerCoreCommand.getInverseDynamicsCommandList());
      setInverseKinematicsCommandList(controllerCoreCommand.getInverseKinematicsCommandList());
      setVirtualModelControlCommandList(controllerCoreCommand.getVirtualModelControlCommandList());
      setFeedbackControlCommandList(controllerCoreCommand.getFeedbackControlCommandList());
      setLowLevelOneDoFJointDesiredDataHolder(controllerCoreCommand.getLowLevelOneDoFJointDesiredDataHolder());
   }

   public void setInverseDynamicsCommandList(InverseDynamicsCommandList inverseDynamicsCommandList)
   {
      hasInverseDynamicsCommandList = true;
      this.inverseDynamicsCommandList.set(inverseDynamicsCommandList);
   }

   public void setInverseKinematicsCommandList(InverseKinematicsCommandList inverseKinematicsCommandList)
   {
      hasInverseKinematicsCommandList = true;
      this.inverseKinematicsCommandList.set(inverseKinematicsCommandList);
   }

   public void setVirtualModelControlCommandList(VirtualModelControlCommandList virtualModelControlCommandList)
   {
      hasVirtualModelControlCommandList = true;
      this.virtualModelControlCommandList.set(virtualModelControlCommandList);
   }

   public void setControllerCoreMode(WholeBodyControllerCoreMode controllerCoreMode)
   {
      hasControllerCoreMode = true;
      this.controllerCoreMode = controllerCoreMode;
   }

   public void setReinitialize(boolean reinitialize)
   {

      this.reinitialize = reinitialize;
   }

   public void requestReinitialization()
   {
      reinitialize = true;
   }

   public void setLowLevelOneDoFJointDesiredDataHolder(LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder)
   {
      hasLowLevelOneDoFJointDesiredDataHolder = true;
      this.lowLevelOneDoFJointDesiredDataHolder.set(lowLevelOneDoFJointDesiredDataHolder);
   }

   public void setFeedbackControlCommandList(FeedbackControlCommandList feedbackControlCommandList)
   {
      hasFeedbackControlCommandList = true;
      this.feedbackControlCommandList.set(feedbackControlCommandList);
   }

   @Override
   public WholeBodyControllerCoreMode getControllerCoreMode()
   {
      return controllerCoreMode;
   }

   @Override
   public boolean isReinitializationRequested()
   {
      return reinitialize;
   }

   @Override
   public InverseDynamicsCommandList getInverseDynamicsCommandList()
   {
      return inverseDynamicsCommandList;
   }

   @Override
   public VirtualModelControlCommandList getVirtualModelControlCommandList()
   {
      return virtualModelControlCommandList;
   }

   @Override
   public FeedbackControlCommandList getFeedbackControlCommandList()
   {
      return feedbackControlCommandList;
   }

   @Override
   public InverseKinematicsCommandList getInverseKinematicsCommandList()
   {
      return inverseKinematicsCommandList;
   }

   @Override
   public LowLevelOneDoFJointDesiredDataHolder getLowLevelOneDoFJointDesiredDataHolder()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   public boolean equals(Object obj)
   {
      if (lowLevelOneDoFJointDesiredDataHolder == null)
         throw new RuntimeException("You used the deprecated constructor equals is not supported in that case");

      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof ControllerCoreCommandDataHolder other)
      {
         if (!inverseKinematicsCommandList.equals(other.inverseKinematicsCommandList))
            return false;
         if (!inverseDynamicsCommandList.equals(other.inverseDynamicsCommandList))
            return false;
         if (!virtualModelControlCommandList.equals(other.virtualModelControlCommandList))
            return false;
         if (!feedbackControlCommandList.equals(other.feedbackControlCommandList))
            return false;
         if (!controllerCoreMode.equals(other.controllerCoreMode))
            return false;
         if(!lowLevelOneDoFJointDesiredDataHolder.equals(other.lowLevelOneDoFJointDesiredDataHolder))
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }
}
