package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.DesiredOneDoFJointTorqueHolder;

public class ControllerCoreCommand
{
   private final InverseDynamicsCommandList solverCommandList;
   private final FeedbackControlCommandList feedbackControlCommandList;
   private final DesiredOneDoFJointTorqueHolder desiredOneDoFJointTorqueHolder;
   private final boolean enableControllerCore;

   public ControllerCoreCommand(boolean enableControllerCore)
   {
      this.enableControllerCore = enableControllerCore;

      if (enableControllerCore)
      {
         solverCommandList = new InverseDynamicsCommandList();
         feedbackControlCommandList = new FeedbackControlCommandList();
         desiredOneDoFJointTorqueHolder = null;
      }
      else
      {
         solverCommandList = null;
         feedbackControlCommandList = null;
         desiredOneDoFJointTorqueHolder = new DesiredOneDoFJointTorqueHolder();
      }
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

   public DesiredOneDoFJointTorqueHolder geDesiredOneDoFJointTorqueHolder()
   {
      return desiredOneDoFJointTorqueHolder;
   }

   public void set(ControllerCoreCommand other)
   {
      solverCommandList.set(other.solverCommandList);
      feedbackControlCommandList.set(other.feedbackControlCommandList);
   }

   public boolean enableControllerCore()
   {
      return enableControllerCore;
   }
}
