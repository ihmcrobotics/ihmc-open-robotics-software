package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommandList;

public class ControllerCoreCommandList
{
   private final InverseDynamicsCommandList solverCommandList = new InverseDynamicsCommandList();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   public ControllerCoreCommandList()
   {
   }

   public void clear()
   {
      solverCommandList.clear();
      feedbackControlCommandList.clear();
   }

   public void addInverseDynamicsCommand(InverseDynamicsCommand<?> inverseDynamicsCommand)
   {
      solverCommandList.addCommand(inverseDynamicsCommand);
   }

   public void addFeedbackControlCommand(FeedbackControlCommand<?> feedbackControlCommand)
   {
      feedbackControlCommandList.addCommand(feedbackControlCommand);
   }

   public InverseDynamicsCommandList getInverseDynamicsCommandList()
   {
      return solverCommandList;
   }

   public FeedbackControlCommandList getFeedbackControlCommandList()
   {
      return feedbackControlCommandList;
   }

   public void set(ControllerCoreCommandList other)
   {
      solverCommandList.set(other.solverCommandList);
      feedbackControlCommandList.set(other.feedbackControlCommandList);
   }
}
