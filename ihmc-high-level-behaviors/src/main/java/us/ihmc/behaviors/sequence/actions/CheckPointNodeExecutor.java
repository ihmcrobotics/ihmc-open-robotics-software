package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class CheckPointNodeExecutor extends ActionNodeExecutor<CheckPointNodeState, CheckPointNodeDefinition>
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
   public void update()
   {
      super.update();
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(false);

      state.setNominalExecutionDuration(0.0);
      state.setElapsedExecutionTime(0.0);
   }
}
