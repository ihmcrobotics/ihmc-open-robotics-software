package us.ihmc.behaviors.tools.walkingController;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Subscription;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.communication.HumanoidControllerAPI.getTopic;
import static us.ihmc.tools.string.StringTools.format;

/**
 * The purpose of this class is to check on the robot progress
 * in stepping footsteps that have been tasked to the robot.
 *
 * TODO: Extract ROS 2 logic so the logic could be used with existing data
 */
public class WalkingFootstepTracker
{
   private final ROS2Subscription<FootstepDataListMessage> footstepDataListSubscriber;
   private final ROS2Subscription<FootstepStatusMessage> footstepStatusSubscriber;
   private final ROS2Subscription<FootstepQueueStatusMessage> footstepQueueStatusSubscriber;

   private final ArrayList<FootstepDataMessage> footsteps = new ArrayList<>();
   private List<QueuedFootstepStatusMessage> queuedFootsteps = new ArrayList<>();
   private transient FramePose3D previousFootstepPose;
   private volatile int completedIndex = 0;
   private volatile int totalStepsCompleted = 0;
   private volatile int totalIncompleteFootsteps = 0;

   private final List<TypedNotification<FootstepQueueStatusMessage>> footstepQueueListeners = new ArrayList<>();

   public WalkingFootstepTracker(ROS2NodeInterface ros2Node, String robotName)
   {
      footstepDataListSubscriber = ros2Node.createSubscription2(getTopic(FootstepDataListMessage.class, robotName),
                                                                this::interceptFootstepDataListMessage);
      footstepStatusSubscriber = ros2Node.createSubscription2(getTopic(FootstepStatusMessage.class, robotName),
                                                              this::acceptFootstepStatusMessage);
      footstepQueueStatusSubscriber = ros2Node.createSubscription2(getTopic(FootstepQueueStatusMessage.class, robotName),
                                                                   this::acceptFootstepQueueStatusMessage);
   }

   public void registerFootstepQueuedMessageListener(TypedNotification<FootstepQueueStatusMessage> footstepQueueListener)
   {
      footstepQueueListeners.add(footstepQueueListener);
   }

   private void acceptFootstepQueueStatusMessage(FootstepQueueStatusMessage footstepQueueStatusMessage)
   {
      for (TypedNotification<FootstepQueueStatusMessage> footstepQueueListener : footstepQueueListeners)
      {
         footstepQueueListener.set(footstepQueueStatusMessage);
      }

      totalIncompleteFootsteps = footstepQueueStatusMessage.getQueuedFootstepList().size();
      queuedFootsteps = footstepQueueStatusMessage.getQueuedFootstepList();
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

   /**
    * This method assumes the list is not empty; you need to check outside this method that the list has at least one in it
    */
   public FramePose3DReadOnly getLastFootstepQueuedOnOppositeSide(RobotSide candidateFootstepSide)
   {
      previousFootstepPose = new FramePose3D();

      int i = queuedFootsteps.size() - 1;
      // Moved the index of the list to the last step on the other side
      while (i >= 1 && queuedFootsteps.get(i).getRobotSide() == candidateFootstepSide.toByte())
         --i;

      previousFootstepPose.getPosition().set(queuedFootsteps.get(i).getLocation());
      previousFootstepPose.getRotation().setToYawOrientation(queuedFootsteps.get(i).getOrientation().getYaw());

      return previousFootstepPose;
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

   public void destroy()
   {
      footstepDataListSubscriber.remove();
      footstepStatusSubscriber.remove();
      footstepQueueStatusSubscriber.remove();
   }
}
