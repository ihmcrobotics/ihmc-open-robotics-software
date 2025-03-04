package us.ihmc.behaviors.logic.condition;

import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.communication.crdt.CRDTBidirectionalLong;

public class CounterConditionExecutor
{
   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;

   private final CRDTBidirectionalLong count;
   private final CRDTBidirectionalLong countTo;

   public CounterConditionExecutor(ConditionNodeState state)
   {
      this.state = state;

      definition = state.getDefinition();

      count = state.getCounter().getCount();
      countTo = definition.getCounter().getCountTo();
   }

   public void updateCurrentlyExecuting()
   {
      if (count.getValue() < countTo.getValue())
      {
         count.setValue(count.getValue() + 1);
         state.setFailed(true);
      }

      state.setIsExecuting(false); // Completes immediately
   }
}
