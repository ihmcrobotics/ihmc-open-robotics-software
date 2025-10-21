package us.ihmc.behaviors.behaviorTree.condition;

import us.ihmc.behaviors.behaviorTree.action.LeafNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ConditionNodeExecutor extends LeafNodeExecutor<ConditionNodeState, ConditionNodeDefinition>
{
   private final CounterConditionExecutor counter;
   private LLMConditionExecutor llm;
   private final ProximityConditionExecutor proximityCheck;

   public ConditionNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(new ConditionNodeState(id, crdtInfo, saveFileDirectory));

      counter = new CounterConditionExecutor(state);
      //      llm = new LLMConditionExecutor(state, referenceFrameLibrary);
      proximityCheck = new ProximityConditionExecutor(state, referenceFrameLibrary);
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
         llm.destroy();
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