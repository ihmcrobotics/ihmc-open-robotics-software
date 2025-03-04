package us.ihmc.behaviors.logic;

import us.ihmc.behaviors.logic.condition.CounterConditionExecutor;
import us.ihmc.behaviors.logic.condition.LLMConditionExecutor;
import us.ihmc.behaviors.sequence.LeafNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ConditionNodeExecutor extends LeafNodeExecutor<ConditionNodeState, ConditionNodeDefinition>
{
   private final CounterConditionExecutor counter;
   private final LLMConditionExecutor llm;

   public ConditionNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new ConditionNodeState(id, crdtInfo, saveFileDirectory));

      counter = new CounterConditionExecutor(state);
      llm = new LLMConditionExecutor(state);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      switch (definition.getType().getValue())
      {
         case COUNTER -> counter.updateCurrentlyExecuting();
         case LLM -> llm.updateCurrentlyExecuting();
      }
   }

   public CounterConditionExecutor getCounter()
   {
      return counter;
   }

   public LLMConditionExecutor getLLM()
   {
      return llm;
   }
}
