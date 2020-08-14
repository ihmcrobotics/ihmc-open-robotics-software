package us.ihmc.humanoidBehaviors.tools.walkingController;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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

   private SideDependentList<FootstepDataMessage> lastCommandedFootsteps = new SideDependentList<>();

   public WalkingFootstepTracker(Ros2NodeInterface ros2Node, String robotName)
   {
      new IHMCROS2Callback<>(ros2Node, ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotName), this::interceptFootstepDataListMessage);
      new IHMCROS2Callback<>(ros2Node, ControllerAPIDefinition.getTopic(FootstepStatusMessage.class, robotName), this::acceptFootstepStatusMessage);

      // TODO: Observe when footsteps are cancelled / walking aborted?
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
      int size = footstepDataListMessage.getFootstepDataList().size();

      synchronized (this)
      {
         if (executionMode == ExecutionMode.OVERRIDE)
         {
            stepsCommanded = 0;
            stepsCompleted = 0;
         }

         stepsCommanded += size;
      }

      // handles same foot steps twice in a row
      for (int i = 0; i < size; i++)
      {
         FootstepDataMessage footstep = footstepDataListMessage.getFootstepDataList().get(i);
         lastCommandedFootsteps.set(RobotSide.fromByte(footstep.getRobotSide()), footstep);
      }

      LogTools.info("Footstep completion: {}/{}", stepsCompleted, stepsCommanded);
   }

   public ImmutablePair<FootstepDataMessage, FootstepDataMessage> getLastCommandedFootsteps()
   {
      return ImmutablePair.of(lastCommandedFootsteps.get(RobotSide.LEFT), lastCommandedFootsteps.get(RobotSide.RIGHT));
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

   public void reset()
   {
      synchronized (this)
      {
         stepsCommanded = 0;
         stepsCompleted = 0;
      }
      lastCommandedFootsteps.clear();
   }
}
