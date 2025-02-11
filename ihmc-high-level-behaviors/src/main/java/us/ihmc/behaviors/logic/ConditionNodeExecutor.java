package us.ihmc.behaviors.logic;

import us.ihmc.behaviors.sequence.LeafNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ConditionNodeExecutor extends LeafNodeExecutor<ConditionNodeState, ConditionNodeDefinition>
{
   public ConditionNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new ConditionNodeState(id, crdtInfo, saveFileDirectory));
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      if (state.getCount().getValue() < definition.getCountTo().getValue())
      {
         state.getCount().setValue(state.getCount().getValue() + 1);
         state.setFailed(true);
      }

      state.setIsExecuting(false); // Completes immediately
   }
}
