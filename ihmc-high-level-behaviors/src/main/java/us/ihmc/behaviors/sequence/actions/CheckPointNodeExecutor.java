package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.LeafNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class CheckPointNodeExecutor extends LeafNodeExecutor<CheckPointNodeState, CheckPointNodeDefinition>
{
   private final CheckPointNodeState state;
   private final CheckPointNodeDefinition definition;

   public CheckPointNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new CheckPointNodeState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(false); // Completes immediately
   }
}
