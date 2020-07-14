package us.ihmc.humanoidBehaviors.tools.walking;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.Ros2NodeInterface;

/**
 * The purpose of this class is to check on the robot progress
 * in stepping footsteps that have been tasked to the robot.
 *
 * TODO: Extract ROS 2 logic so the logic could be used with existing data
 */
public class WalkingFootstepTracker
{
   private int stepsCommanded = 0;
   private int stepsCompleted = 0;

   public WalkingFootstepTracker(Ros2NodeInterface ros2Node, String robotName)
   {
      new ROS2Callback<>(ros2Node,
                         FootstepDataListMessage.class,
                         ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotName),
                         this::interceptFootstepDataListMessage);
      new ROS2Callback<>(ros2Node,
                         FootstepStatusMessage.class,
                         ControllerAPIDefinition.getTopic(FootstepStatusMessage.class, robotName),
                         this::acceptFootstepStatusMessage);
   }

   private void acceptFootstepStatusMessage(FootstepStatusMessage footstepStatusMessage)
   {
      if (FootstepStatus.fromByte(footstepStatusMessage.getFootstepStatus()) == FootstepStatus.COMPLETED)
      {
         synchronized (this)
         {
            ++stepsCompleted;

            if (stepsCommanded < stepsCompleted)
            {
               LogTools.warn("Delayed message(s) detected.");
               stepsCompleted = stepsCommanded;
            }
         }

         LogTools.info("Footstep completion: {}/{}", stepsCompleted, stepsCommanded);
      }
   }

   private void interceptFootstepDataListMessage(FootstepDataListMessage footstepDataListMessage)
   {
      ExecutionMode executionMode = ExecutionMode.fromByte(footstepDataListMessage.getQueueingProperties().getExecutionMode());

      synchronized (this)
      {
         if (executionMode == ExecutionMode.OVERRIDE)
         {
            stepsCommanded = 0;
            stepsCompleted = 0;
         }
      }

      stepsCommanded += footstepDataListMessage.getFootstepDataList().size();
      LogTools.info("Footstep completion: {}/{}", stepsCompleted, stepsCommanded);
   }

   public int getNumberOfIncompleteFootsteps()
   {
      int numberOfIncompleteFootsteps;
      synchronized (this)
      {
         numberOfIncompleteFootsteps = stepsCommanded - stepsCompleted;
      }
      return numberOfIncompleteFootsteps;
   }
}
