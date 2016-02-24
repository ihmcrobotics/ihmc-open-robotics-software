package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.lowLevelControl.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.lowLevelControl.LowLevelOneDoFJointDesiredDataHolderInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommandList;

public class ControllerCoreCommand implements ControllerCoreCommandInterface
{
   private final InverseDynamicsCommandList solverCommandList;
   private final FeedbackControlCommandList feedbackControlCommandList;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;
   private final boolean enableControllerCore;

   public ControllerCoreCommand(boolean enableControllerCore)
   {
      this.enableControllerCore = enableControllerCore;

      solverCommandList = new InverseDynamicsCommandList();
      feedbackControlCommandList = new FeedbackControlCommandList();
      lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   }

   public void clear()
   {
      solverCommandList.clear();
      feedbackControlCommandList.clear();
      lowLevelOneDoFJointDesiredDataHolder.clear();
   }

   public void addInverseDynamicsCommand(InverseDynamicsCommand<?> inverseDynamicsCommand)
   {
      solverCommandList.addCommand(inverseDynamicsCommand);
   }

   public void addFeedbackControlCommand(FeedbackControlCommand<?> feedbackControlCommand)
   {
      feedbackControlCommandList.addCommand(feedbackControlCommand);
   }

   public void completeLowLevelJointData(LowLevelOneDoFJointDesiredDataHolderInterface lowLevelJointData)
   {
      lowLevelOneDoFJointDesiredDataHolder.completeWith(lowLevelJointData);
   }

   @Override
   public InverseDynamicsCommandList getInverseDynamicsCommandList()
   {
      return solverCommandList;
   }

   @Override
   public FeedbackControlCommandList getFeedbackControlCommandList()
   {
      return feedbackControlCommandList;
   }

   @Override
   public LowLevelOneDoFJointDesiredDataHolder getLowLevelOneDoFJointDesiredDataHolder()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   public void set(ControllerCoreCommand other)
   {
      solverCommandList.set(other.solverCommandList);
      feedbackControlCommandList.set(other.feedbackControlCommandList);
      lowLevelOneDoFJointDesiredDataHolder.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
   }

   @Override
   public boolean enableControllerCore()
   {
      return enableControllerCore;
   }
}
