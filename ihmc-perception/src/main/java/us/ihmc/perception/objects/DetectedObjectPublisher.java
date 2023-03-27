package us.ihmc.perception.objects;

import perception_msgs.msg.dds.DetectedObjectMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.ros2.ROS2Topic;

public class DetectedObjectPublisher
{
   private final ROS2PublishSubscribeAPI ros2;
   private final ROS2Topic<DetectedObjectMessage> topic;
   private final long ID;
   private final ReferenceFrame objectFrame;
   private final RigidBodyTransform objectTransformToWorld = new RigidBodyTransform();
   private final DetectedObjectMessage detectedObjectMessage = new DetectedObjectMessage();
   private boolean detected = false;

   public DetectedObjectPublisher(ROS2PublishSubscribeAPI ros2, ROS2Topic<DetectedObjectMessage> topic, long ID, ReferenceFrame objectFrame)
   {
      this.ros2 = ros2;
      this.topic = topic;
      this.ID = ID;
      this.objectFrame = objectFrame;
   }

   public void reset()
   {
      detected = false;
   }

   public void markDetected(long detectedID)
   {
      if (detectedID == ID)
      {
         detected = true;
      }
   }

   public void publish()
   {
      detectedObjectMessage.setDetected(detected);

      detectedObjectMessage.setID(ID);

      objectFrame.getTransformToDesiredFrame(objectTransformToWorld, ReferenceFrame.getWorldFrame());
      MessageTools.toMessage(objectTransformToWorld, detectedObjectMessage.getTransformToWorld());

      ros2.publish(topic, detectedObjectMessage);
   }
}