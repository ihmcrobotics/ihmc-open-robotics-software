package us.ihmc.behaviors.tools.walkingController;

import static us.ihmc.communication.HumanoidControllerAPI.getLowFrequencyTopic;
import static us.ihmc.communication.HumanoidControllerAPI.getTopic;
import static us.ihmc.tools.string.StringTools.format;

import controller_msgs.FootstepDataListMessage;
import controller_msgs.FootstepDataMessage;
import controller_msgs.FootstepQueueStatusMessage;
import controller_msgs.FootstepStatusMessage;
import controller_msgs.QueuedFootstepStatusMessage;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;

/**
 * The purpose of this class is to check on the robot progress
 * in stepping footsteps that have been tasked to the robot.
 *
 * TODO: Extract ROS 2 logic so the logic could be used with existing data
 */
public class WalkingFootstepTracker
{
   private final ROS2Node ros2Node;
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

   public WalkingFootstepTracker(ROS2Node ros2Node, String robotName)
   {
      this.ros2Node = ros2Node;
      var footstepDataListTopic = getTopic(FootstepDataListMessage.class, robotName);
      footstepDataListSubscriber = ros2Node.createSubscriptionSampler(footstepDataListTopic, this::interceptFootstepDataListMessage);
      var footstepStatusTopic = getTopic(FootstepStatusMessage.class, robotName);
      footstepStatusSubscriber = ros2Node.createSubscriptionSampler(footstepStatusTopic, this::acceptFootstepStatusMessage);
      var footstepQueueStatusTopic = getLowFrequencyTopic(FootstepQueueStatusMessage.class, robotName);
      footstepQueueStatusSubscriber = ros2Node.createSubscriptionSampler(footstepQueueStatusTopic, this::acceptFootstepQueueStatusMessage);
   }

   public void registerFootstepQueuedMessageListener(TypedNotification<FootstepQueueStatusMessage> footstepQueueListener)
   {
      footstepQueueListeners.add(footstepQueueListener);
   }

   private void acceptFootstepQueueStatusMessage(FootstepQueueStatusMessage sample)
   {
      FootstepQueueStatusMessage footstepQueueStatusMessage = new FootstepQueueStatusMessage();
      footstepQueueStatusMessage.set(sample);

      for (TypedNotification<FootstepQueueStatusMessage> footstepQueueListener : footstepQueueListeners)
      {
         FootstepQueueStatusMessage copy = new FootstepQueueStatusMessage();
         copy.set(footstepQueueStatusMessage);
         footstepQueueListener.set(copy);
      }

      totalIncompleteFootsteps = footstepQueueStatusMessage.getQueuedFootstepList().size();
      queuedFootsteps = new ArrayList<>();
      for (int i = 0; i < footstepQueueStatusMessage.getQueuedFootstepList().size(); i++)
         queuedFootsteps.add(footstepQueueStatusMessage.getQueuedFootstepList().get(i));
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

      previousFootstepPose.getPosition().set(queuedFootsteps.get(i).getLocation().getPoint());
      previousFootstepPose.getRotation().setToYawOrientation(queuedFootsteps.get(i).getOrientation().getQuaternion().getYaw());

      return previousFootstepPose;
   }

   public int getNumberOfIncompleteFootsteps()
   {
      return totalIncompleteFootsteps;
   }

   public ArrayList<FootstepDataMessage> getFootsteps()
   {
      return footsteps;
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
      ros2Node.destroySubscription(footstepDataListSubscriber);
      ros2Node.destroySubscription(footstepStatusSubscriber);
      ros2Node.destroySubscription(footstepQueueStatusSubscriber);
   }
}
