package us.ihmc.perception.scene;

import perception_msgs.msg.dds.DetectedObjectMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.ros2.ROS2Topic;

/**
 * Handles the ROS 2 publication of detected objects.
 */
public class ROS2DetectableSceneObject extends DetectableSceneObject
{
   private ROS2PublishSubscribeAPI ros2;
   private ROS2Topic<DetectedObjectMessage> topic;
   private boolean detected = false;
   private final DetectedObjectMessage detectedObjectMessage = new DetectedObjectMessage();

   public ROS2DetectableSceneObject(String name)
   {
      super(name);
   }

   public void setupForROS2Publishing(ROS2PublishSubscribeAPI ros2, ROS2Topic<DetectedObjectMessage> topic)
   {
      this.ros2 = ros2;
      this.topic = topic;
   }

   public void reset()
   {
      detected = false;
   }

//   public void objectDetected(String detectedID)
//   {
//      if (detectedID.equals(id))
//      {
//         detected = true;
//      }
//   }

   public void publish()
   {
      if (detected)
      {
//         detectedObjectMessage.setId(id);

         MessageTools.toMessage(getReferenceFrame().getTransformToRoot(), detectedObjectMessage.getTransformToWorld());

         ros2.publish(topic, detectedObjectMessage);
      }
   }
}
