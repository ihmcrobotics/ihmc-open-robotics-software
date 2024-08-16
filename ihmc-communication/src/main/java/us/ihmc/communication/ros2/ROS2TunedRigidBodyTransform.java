package us.ihmc.communication.ros2;

import controller_msgs.msg.dds.RigidBodyTransformMessage;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.thread.Throttler;

/**
 * This class is used to hava UI that tunes a transform running remotely on the robot.
 * It allows to make sure to handle the case where two UIs are running, they aren't both
 * sending conflicting updates at the same time, and they both receive the updated
 * transforms as they are tuned.
 */
public class ROS2TunedRigidBodyTransform
{
   private final ROS2PublishSubscribeAPI ros2;
   private final ROS2IOTopicPair<RigidBodyTransformMessage> topicPair;
   private final RigidBodyTransform rigidBodyTransformToSync;
   private final ROS2Input<RigidBodyTransformMessage> frameUpdateSubscription;
   private final Throttler statusThrottler;
   private final RigidBodyTransformMessage statusMessage = new RigidBodyTransformMessage();
   private final boolean isRemoteTuner;
   private boolean acceptingUpdates = true;
   private boolean publishingStatus = true;

   public static ROS2TunedRigidBodyTransform toBeTuned(ROS2PublishSubscribeAPI ros2,
                                                       ROS2IOTopicPair<RigidBodyTransformMessage> topicPair,
                                                       RigidBodyTransform rigidBodyTransformToSync)
   {
      return new ROS2TunedRigidBodyTransform(ros2, topicPair, rigidBodyTransformToSync, false);
   }

   public static ROS2TunedRigidBodyTransform remoteTuner(ROS2PublishSubscribeAPI ros2,
                                                         ROS2IOTopicPair<RigidBodyTransformMessage> topicPair,
                                                         RigidBodyTransform rigidBodyTransformToSync)
   {
      return new ROS2TunedRigidBodyTransform(ros2, topicPair, rigidBodyTransformToSync, true);
   }


   private ROS2TunedRigidBodyTransform(ROS2PublishSubscribeAPI ros2,
                                       ROS2IOTopicPair<RigidBodyTransformMessage> topicPair,
                                       RigidBodyTransform rigidBodyTransformToSync,
                                       boolean isRemoteTuner)
   {
      this.ros2 = ros2;
      this.topicPair = topicPair;
      this.rigidBodyTransformToSync = rigidBodyTransformToSync;
      this.isRemoteTuner = isRemoteTuner;
      // The tuning part is higher frequency to see the updates smoother as you tune. (5 Hz)
      // The status is just and update of the current transform where an observer is not actively tuning. (2.5 Hz)
      statusThrottler = new Throttler().setFrequency(isRemoteTuner ? 5.0 : ROS2Heartbeat.STATUS_FREQUENCY);
      frameUpdateSubscription = ros2.subscribe(isRemoteTuner ? topicPair.getStatusTopic() : topicPair.getCommandTopic());
   }

   public void update()
   {
      if (acceptingUpdates && frameUpdateSubscription.getMessageNotification().poll())
      {
         MessageTools.toEuclid(frameUpdateSubscription.getMessageNotification().read(), rigidBodyTransformToSync);
      }

      if (publishingStatus && statusThrottler.run())
      {
         MessageTools.toMessage(rigidBodyTransformToSync, statusMessage);
         ros2.publish(isRemoteTuner ? topicPair.getCommandTopic() : topicPair.getStatusTopic(), statusMessage);
      }
   }

   public void setAcceptingUpdates(boolean acceptingUpdates)
   {
      this.acceptingUpdates = acceptingUpdates;
   }

   public void setPublishingStatus(boolean publishingStatus)
   {
      this.publishingStatus = publishingStatus;
   }
}
