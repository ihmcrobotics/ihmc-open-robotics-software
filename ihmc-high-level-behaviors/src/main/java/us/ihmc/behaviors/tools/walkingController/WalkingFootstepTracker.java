package us.ihmc.behaviors.tools.walkingController;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
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

   public WalkingFootstepTracker(ROS2NodeInterface ros2Node, String robotName)
   {
      new IHMCROS2Callback<>(ros2Node, getTopic(FootstepDataListMessage.class, robotName), this::interceptFootstepDataListMessage);
      new IHMCROS2Callback<>(ros2Node, getTopic(FootstepStatusMessage.class, robotName), this::acceptFootstepStatusMessage);

      // TODO: Observe when footsteps are cancelled / walking aborted?
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
         }

         LogTools.info(format("{} footstep completed. Completion: {}/{} -> {}/{}. ID: {}",
                              RobotSide.fromByte(footstepStatusMessage.getRobotSide()),
                              priorNumerator,
                              priorDenominator,
                              completedIndex,
                              footsteps.size(),
                              footstepStatusMessage.getSequenceId()));
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
      int numberOfIncompleteFootsteps;
      synchronized (this)
      {
         numberOfIncompleteFootsteps = footsteps.size() - completedIndex;
      }
      return numberOfIncompleteFootsteps;
   }

   public int getNumberOfCompletedFootsteps()
   {
      return completedIndex;
   }

   public void reset()
   {
      synchronized (this)
      {
         footsteps.clear();
         completedIndex = 0;
      }
   }

   public ArrayList<FootstepDataMessage> getFootsteps()
   {
      return footsteps;
   }

   public int getCompletedIndex()
   {
      return completedIndex;
   }
}
