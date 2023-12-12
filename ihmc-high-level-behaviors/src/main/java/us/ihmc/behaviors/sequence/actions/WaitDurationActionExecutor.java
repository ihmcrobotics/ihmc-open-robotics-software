package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.Timer;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class WaitDurationActionExecutor extends ActionNodeExecutor<WaitDurationActionState, WaitDurationActionDefinition>
{
   private final WaitDurationActionState state;
   private final Timer executionTimer = new Timer();

   public WaitDurationActionExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new WaitDurationActionState(id, crdtInfo, saveFileDirectory));

      state = getState();
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void triggerActionExecution()
   {
      executionTimer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(executionTimer.isRunning(getDefinition().getWaitDuration()));

      state.setNominalExecutionDuration(getDefinition().getWaitDuration());
      state.setElapsedExecutionTime(executionTimer.getElapsedTime());
   }
}
