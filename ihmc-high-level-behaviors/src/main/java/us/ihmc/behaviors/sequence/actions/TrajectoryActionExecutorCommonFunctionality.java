package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.TrajectoryTrackingErrorCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;

public class TrajectoryActionExecutorCommonFunctionality
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ActionNodeState<?> state;
   private final TrajectoryTrackingErrorCalculator trackingCalculator = new TrajectoryTrackingErrorCalculator();
   private final transient StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
   private boolean hitTimeLimit;

   public TrajectoryActionExecutorCommonFunctionality(ROS2SyncedRobotModel syncedRobot, ROS2ControllerHelper ros2ControllerHelper, ActionNodeState<?> state)
   {
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.state = state;
   }

   public void update()
   {
      trackingCalculator.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));
   }

   public void triggerActionExecution(double trajectoryDuration)
   {
      trackingCalculator.reset();
      hitTimeLimit = false;
      state.setNominalExecutionDuration(trajectoryDuration);
   }

   public void checkAndHandleTimeLimit()
   {
      trackingCalculator.computeExecutionTimings(state.getNominalExecutionDuration());
      state.setElapsedExecutionTime(trackingCalculator.getElapsedTime());

      if (trackingCalculator.getHitTimeLimit())
      {
         state.setIsExecuting(false);
         state.setFailed(true);
         LogTools.error("Task execution timed out. Publishing stop all trajectories message.");
         ros2ControllerHelper.publishToController(stopAllTrajectoryMessage);
         hitTimeLimit = true;
      }
   }

   public boolean getHitTimeLimit()
   {
      return hitTimeLimit;
   }
}
