package us.ihmc.behaviors.behaviorTree.condition;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeExecutor;

public class ConditionNodeExecutor extends LeafNodeExecutor<ConditionNodeState, ConditionNodeDefinition>
{
   private final CounterConditionExecutor counter;
   private LLMConditionExecutor llm;
   private final ProximityConditionExecutor proximityCheck;

   public ConditionNodeExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new ConditionNodeState(id, rootNode.getState()), rootNode);

      counter = new CounterConditionExecutor(state);
      //      llm = new LLMConditionExecutor(state, scene);
      proximityCheck = new ProximityConditionExecutor(state, scene);
   }

   @Override
   public void update()
   {
      super.update();

      switch (definition.getType().getValue())
      {
         //         case LLM -> llm.update();
         case PROXIMITY -> proximityCheck.update();
      }
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      switch (definition.getType().getValue())
      {
         case ALWAYS_FAIL:
            state.setFailed(true);
         case ALWAYS_SUCCEED:
            state.setIsExecuting(false);
            break;
         case PROXIMITY:
            proximityCheck.triggerExecution();
            break;
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      switch (definition.getType().getValue())
      {
         case COUNTER -> counter.updateCurrentlyExecuting();
         //         case LLM -> llm.updateCurrentlyExecuting();
         case PROXIMITY -> proximityCheck.updateCurrentlyExecuting();
      }
   }

   @Override
   public void destroy()
   {
      if (llm != null)
         LLMConditionExecutor.destroy();
   }

   public CounterConditionExecutor getCounter()
   {
      return counter;
   }

   public LLMConditionExecutor getLLM()
   {
      return llm;
   }

   public ProximityConditionExecutor getProximityCheck()
   {
      return proximityCheck;
   }
}