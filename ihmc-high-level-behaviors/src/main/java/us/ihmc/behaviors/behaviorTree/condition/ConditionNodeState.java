package us.ihmc.behaviors.behaviorTree.condition;

import behavior_msgs.msg.dds.ConditionNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;

public class ConditionNodeState extends LeafNodeState<ConditionNodeDefinition>
{
   private final CounterConditionState counter;
   private final LLMConditionState llm;
   private final ProximityConditionState proximityCheck;

   public ConditionNodeState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new ConditionNodeDefinition(rootNode.getDefinition()), rootNode);

      counter = new CounterConditionState(definition);
      llm = new LLMConditionState(definition);
      proximityCheck = new ProximityConditionState(definition);
   }

   public void toMessage(ConditionNodeStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());

      switch (definition.getType().getValue())
      {
         case COUNTER -> counter.toMessage(message);
         case LLM -> llm.toMessage(message);
         case PROXIMITY -> proximityCheck.toMessage(message);
      }
   }

   public void fromMessage(ConditionNodeStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());

      switch (definition.getType().getValue())
      {
         case COUNTER -> counter.fromMessage(message);
         case LLM -> llm.fromMessage(message);
         case PROXIMITY -> proximityCheck.fromMessage(message);
      }
   }

   public CounterConditionState getCounter()
   {
      return counter;
   }

   public LLMConditionState getLLM()
   {
      return llm;
   }

   public ProximityConditionState getProximityCheck()
   {
      return proximityCheck;
   }
}