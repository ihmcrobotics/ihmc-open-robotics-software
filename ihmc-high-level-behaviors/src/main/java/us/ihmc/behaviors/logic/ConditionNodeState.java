package us.ihmc.behaviors.logic;

import behavior_msgs.msg.dds.ConditionNodeStateMessage;
import us.ihmc.behaviors.logic.condition.CounterConditionState;
import us.ihmc.behaviors.logic.condition.LLMConditionState;
import us.ihmc.behaviors.sequence.LeafNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ConditionNodeState extends LeafNodeState<ConditionNodeDefinition>
{
   private final CounterConditionState counter;
   private final LLMConditionState llm;

   public ConditionNodeState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new ConditionNodeDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      counter = new CounterConditionState(definition);
      llm = new LLMConditionState(definition);
   }

   public void toMessage(ConditionNodeStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());

      switch (definition.getType().getValue())
      {
         case COUNTER -> counter.toMessage(message);
         case LLM -> llm.toMessage(message);
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
}
