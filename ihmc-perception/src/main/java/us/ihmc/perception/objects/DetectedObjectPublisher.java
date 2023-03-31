package us.ihmc.perception.objects;

import perception_msgs.msg.dds.DetectedObjectMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.ros2.ROS2Topic;

public class DetectedObjectPublisher
{
   private final ROS2PublishSubscribeAPI ros2;
   private final ROS2Topic<DetectedObjectMessage> topic;
   private final String id;
   private final ReferenceFrame objectFrame;
   private final DetectedObjectMessage detectedObjectMessage = new DetectedObjectMessage();
   private boolean detected = false;

   public DetectedObjectPublisher(ROS2PublishSubscribeAPI ros2, ROS2Topic<DetectedObjectMessage> topic, String id, ReferenceFrame objectFrame)
   {
      this.ros2 = ros2;
      this.topic = topic;
      this.id = id;
      this.objectFrame = objectFrame;
   }

   public void reset()
   {
      detected = false;
   }

   public void objectDetected(String detectedID)
   {
      if (detectedID.equals(id))
      {
         detected = true;
      }
   }

   public void publish()
   {
      if (detected)
      {
         detectedObjectMessage.setId(id);

         MessageTools.toMessage(objectFrame.getTransformToRoot(), detectedObjectMessage.getTransformToWorld());

         ros2.publish(topic, detectedObjectMessage);
      }
   }
}