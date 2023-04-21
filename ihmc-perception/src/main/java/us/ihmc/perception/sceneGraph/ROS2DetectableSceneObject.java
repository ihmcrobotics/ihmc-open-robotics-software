package us.ihmc.perception.sceneGraph;

import perception_msgs.msg.dds.DetectableSceneObjectMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.ros2.ROS2Topic;

/**
 * Handles the ROS 2 publication of detected objects.
 */
public class ROS2DetectableSceneObject extends DetectableSceneObject
{
   private ROS2PublishSubscribeAPI ros2;
   private ROS2Topic<DetectableSceneObjectMessage> topic;
   private boolean detected = false;
   private final DetectableSceneObjectMessage detectableSceneObjectMessage = new DetectableSceneObjectMessage();

   public ROS2DetectableSceneObject(String name)
   {
      super(name);
   }

   public void setupForROS2Publishing(ROS2PublishSubscribeAPI ros2, ROS2Topic<DetectableSceneObjectMessage> topic)
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
//         detectableSceneObjectMessage.setId(id);

         MessageTools.toMessage(getReferenceFrame().getTransformToRoot(), detectableSceneObjectMessage.getTransformToWorld());

         ros2.publish(topic, detectableSceneObjectMessage);
      }
   }
}
