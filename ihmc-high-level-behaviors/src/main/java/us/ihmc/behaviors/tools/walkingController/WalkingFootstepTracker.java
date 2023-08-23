package us.ihmc.behaviors.tools.walkingController;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.ArrayList;

import static us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition.getTopic;
import static us.ihmc.tools.string.StringTools.format;

/**
 * The purpose of this class is to check on the robot progress
 * in stepping footsteps that have been tasked to the robot.
 *
 * TODO: Extract ROS 2 logic so the logic could be used with existing data
 */
public class WalkingFootstepTracker
{
   private final ArrayList<FootstepDataMessage> footsteps = new ArrayList<>();
   private volatile int completedIndex = 0;
   private volatile int totalStepsCompleted = 0;
   private volatile int totalIncompleteFootsteps = 0;

   public WalkingFootstepTracker(ROS2NodeInterface ros2Node, String robotName)
   {
      new IHMCROS2Callback<>(ros2Node, getTopic(FootstepDataListMessage.class, robotName), this::interceptFootstepDataListMessage);
      new IHMCROS2Callback<>(ros2Node, getTopic(FootstepStatusMessage.class, robotName), this::acceptFootstepStatusMessage);
      new IHMCROS2Callback<>(ros2Node, getTopic(FootstepQueueStatusMessage.class, robotName), this::acceptFootstepQueueStatusMessage);
   }

   private void acceptFootstepQueueStatusMessage(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      totalIncompleteFootsteps = footstepQueueStatusMessage.getQueuedFootstepList().size();
   }

   private void acceptFootstepStatusMessage(FootstepStatusMessage footstepStatusMessage)
   {
      if (FootstepStatus.fromByte(footstepStatusMessage.getFootstepStatus()) == FootstepStatus.COMPLETED)
      {
         int priorNumerator = completedIndex;
         int priorDenominator = footsteps.size();

         synchronized (this)
         {
            for (int i = 0; i < footsteps.size(); i++)
            {
               if (footsteps.get(i).getSequenceId() == footstepStatusMessage.getSequenceId())
               {
                  completedIndex = i + 1;
                  break;
               }
            }

            totalStepsCompleted++;
         }

         LogTools.info(format("{} footstep completed. Completion: {}/{} -> {}/{}. ID: {} Total steps completed: {}",
                              RobotSide.fromByte(footstepStatusMessage.getRobotSide()),
                              priorNumerator,
                              priorDenominator,
                              completedIndex,
                              footsteps.size(),
                              footstepStatusMessage.getSequenceId(),
                              totalStepsCompleted));
      }
   }

   private void interceptFootstepDataListMessage(FootstepDataListMessage footstepDataListMessage)
   {
      ExecutionMode executionMode = ExecutionMode.fromByte(footstepDataListMessage.getQueueingProperties().getExecutionMode());
      int size = footstepDataListMessage.getFootstepDataList().size();
      int priorNumerator = completedIndex;
      int priorDenominator = footsteps.size();
      long[] ids = new long[size];

      synchronized (this)
      {
         if (executionMode == ExecutionMode.OVERRIDE)
         {
            footsteps.clear();
            completedIndex = 0;
         }

         for (int i = 0; i < size; i++)
         {
            FootstepDataMessage footstep = footstepDataListMessage.getFootstepDataList().get(i);
            ids[i] = footstep.getSequenceId();
            footsteps.add(footstep);
         }
      }

      LogTools.info(format("{}ing {} footstep{}. Completion: {}/{} -> {}/{}. IDs: {}",
                           executionMode.name(),
                           size,
                           size > 1 ? "s" : "",
                           priorNumerator,
                           priorDenominator,
                           completedIndex,
                           footsteps.size(),
                           ids));
   }

   public int getNumberOfIncompleteFootsteps()
   {
      return totalIncompleteFootsteps;
   }

   public void reset()
   {
      synchronized (this)
      {
         footsteps.clear();
         completedIndex = 0;
      }
   }
}
