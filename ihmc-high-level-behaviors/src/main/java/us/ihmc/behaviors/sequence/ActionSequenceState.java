package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.ActionSequenceStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class ActionSequenceState extends BehaviorTreeNodeState<ActionSequenceDefinition>
{
   private final CRDTBidirectionalBoolean automaticExecution;
   private final CRDTBidirectionalInteger executionNextIndex;
   private final CRDTBidirectionalNotification manualExecutionRequested;
   private final CRDTUnidirectionalField<String> nextActionRejectionTooltip;

   public ActionSequenceState(long id, CRDTInfo crdtInfo)
   {
      super(id, new ActionSequenceDefinition(crdtInfo), crdtInfo);

      automaticExecution = new CRDTBidirectionalBoolean(this, false);
      executionNextIndex = new CRDTBidirectionalInteger(this, 0);
      manualExecutionRequested = new CRDTBidirectionalNotification(this);
      nextActionRejectionTooltip = new CRDTUnidirectionalField<>(ROS2ActorDesignation.ROBOT, crdtInfo, "");
   }

   public void toMessage(ActionSequenceStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setAutomaticExecution(automaticExecution.toMessage());
      message.setExecutionNextIndex(executionNextIndex.toMessage());
      message.setManualExecutionRequested(manualExecutionRequested.toMessage());
      message.setNextActionRejectionTooltip(nextActionRejectionTooltip.toMessage());
   }

   public void fromMessage(ActionSequenceStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      automaticExecution.fromMessage(message.getAutomaticExecution());
      executionNextIndex.fromMessage(message.getExecutionNextIndex());
      manualExecutionRequested.fromMessage(message.getManualExecutionRequested());
      nextActionRejectionTooltip.fromMessage(message.getNextActionRejectionTooltipAsString());
   }

   public void stepBackNextExecutionIndex()
   {
      if (executionNextIndex.intValue() > 0)
         executionNextIndex.decrement();
   }

   public void stepForwardNextExecutionIndex()
   {
      if (executionNextIndex.intValue() < getChildren().size())
         executionNextIndex.increment();
   }

   public void setExecutionNextIndex(int executionNextIndex)
   {
      this.executionNextIndex.setValue(executionNextIndex);
   }

   public int getExecutionNextIndex()
   {
      return executionNextIndex.intValue();
   }

   public void setNextActionRejectionTooltip(String nextActionRejectionTooltip)
   {
      this.nextActionRejectionTooltip.setValue(nextActionRejectionTooltip);
   }

   public String getNextActionRejectionTooltip()
   {
      return nextActionRejectionTooltip.getValue();
   }

   public boolean getAutomaticExecution()
   {
      return automaticExecution.booleanValue();
   }

   public void setAutomaticExecution(boolean automaticExecution)
   {
      this.automaticExecution.setValue(automaticExecution);
   }

   public boolean pollManualExecutionRequested()
   {
      boolean poll = manualExecutionRequested.poll();
//      LogTools.info("Polled: {}", poll);
      return poll;
   }

   public boolean getManualExecutionRequested()
   {
      return manualExecutionRequested.peek();
   }

   public void setManualExecutionRequested()
   {
      manualExecutionRequested.set();
   }
}
