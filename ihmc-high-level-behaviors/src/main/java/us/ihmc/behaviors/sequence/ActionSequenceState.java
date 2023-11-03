package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionSequenceStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.CRDTInfo;

public class ActionSequenceState extends BehaviorTreeNodeState<ActionSequenceDefinition>
{
   private boolean automaticExecution = false;
   private int executionNextIndex = 0;
   private String nextActionRejectionTooltip = "";
   private boolean manualExecutionRequested = false;

   public ActionSequenceState(long id, CRDTInfo crdtInfo)
   {
      super(id, new ActionSequenceDefinition(), crdtInfo);
   }

   public void toMessage(ActionSequenceStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setAutomaticExecution(automaticExecution);
      message.setExecutionNextIndex(executionNextIndex);
      message.setNextActionRejectionTooltip(nextActionRejectionTooltip);
      message.setManualExecutionRequested(message.getManualExecutionRequested());
   }

   public void fromMessage(ActionSequenceStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      // Only modified by robot
      nextActionRejectionTooltip = message.getNextActionRejectionTooltipAsString();

      // Only modified by operator

      if (isFrozen()) // Can be modified by either
      {
         automaticExecution = message.getAutomaticExecution();
         manualExecutionRequested = message.getManualExecutionRequested();
         executionNextIndex = message.getExecutionNextIndex();
      }
   }

   public void stepBackNextExecutionIndex()
   {
      if (executionNextIndex > 0)
         --executionNextIndex;
      freeze();
   }

   public void stepForwardNextExecutionIndex()
   {
      if (executionNextIndex < getChildren().size())
         ++executionNextIndex;
      freeze();
   }

   public void setExecutionNextIndex(int executionNextIndex)
   {
      this.executionNextIndex = executionNextIndex;
      freeze();
   }

   public int getExecutionNextIndex()
   {
      return executionNextIndex;
   }

   public void setNextActionRejectionTooltip(String nextActionRejectionTooltip)
   {
      this.nextActionRejectionTooltip = nextActionRejectionTooltip;
   }

   public String getNextActionRejectionTooltip()
   {
      return nextActionRejectionTooltip;
   }

   public boolean getAutomaticExecution()
   {
      return automaticExecution;
   }

   public void setAutomaticExecution(boolean automaticExecution)
   {
      this.automaticExecution = automaticExecution;
      freeze();
   }

   public boolean pollManualExecutionRequested()
   {
      boolean previousValue = manualExecutionRequested;
      setManualExecutionRequested(false);
      return previousValue;
   }

   public boolean getManualExecutionRequested()
   {
      return manualExecutionRequested;
   }

   public void setManualExecutionRequested(boolean manualExecutionRequested)
   {
      this.manualExecutionRequested = manualExecutionRequested;
      freeze();
   }
}
