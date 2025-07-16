package us.ihmc.behaviors.sequence.actions;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.NonWallTimer;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class WaitDurationActionExecutor extends ActionNodeExecutor<WaitDurationActionState, WaitDurationActionDefinition>
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final NonWallTimer executionTimer = new NonWallTimer();

   public WaitDurationActionExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ROS2SyncedRobotModel syncedRobot)
   {
      super(new WaitDurationActionState(id, crdtInfo, saveFileDirectory));

      this.syncedRobot = syncedRobot;
   }

   @Override
   public void update()
   {
      super.update();

      executionTimer.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      state.getLogger().info("Waiting for %.2f s...".formatted(definition.getWaitDuration()));

      executionTimer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(executionTimer.isRunning(definition.getWaitDuration()));

      state.setNominalExecutionDuration(definition.getWaitDuration());
      state.setElapsedExecutionTime(executionTimer.getElapsedTime());
   }
}
