package us.ihmc.communication.ros2;

import controller_msgs.msg.dds.RigidBodyTransformMessage;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.thread.Throttler;

public class ROS2SyncedRigidBodyTransform
{
   private final ROS2PublishSubscribeAPI ros2;
   private final ROS2IOTopicPair<RigidBodyTransformMessage> topicPair;
   private final RigidBodyTransform rigidBodyTransformToSync;
   private final IHMCROS2Input<RigidBodyTransformMessage> frameUpdateSubscription;
   private final Throttler statusThrottler = new Throttler().setFrequency(ROS2Heartbeat.STATUS_FREQUENCY);
   private final RigidBodyTransformMessage statusMessage = new RigidBodyTransformMessage();

   public ROS2SyncedRigidBodyTransform(ROS2PublishSubscribeAPI ros2,
                                       ROS2IOTopicPair<RigidBodyTransformMessage> topicPair,
                                       RigidBodyTransform rigidBodyTransformToSync)
   {
      this.ros2 = ros2;
      this.topicPair = topicPair;
      this.rigidBodyTransformToSync = rigidBodyTransformToSync;
      frameUpdateSubscription = ros2.subscribe(topicPair.getCommandTopic());
   }

   public void update()
   {
      if (frameUpdateSubscription.getMessageNotification().poll())
      {
         MessageTools.toEuclid(frameUpdateSubscription.getMessageNotification().read(), rigidBodyTransformToSync);
      }

      if (statusThrottler.run())
      {
         MessageTools.toMessage(rigidBodyTransformToSync, statusMessage);
         ros2.publish(topicPair.getStatusTopic(), statusMessage);
      }
   }
}
