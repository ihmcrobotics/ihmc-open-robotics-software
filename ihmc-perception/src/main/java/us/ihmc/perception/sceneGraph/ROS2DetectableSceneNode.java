package us.ihmc.perception.sceneGraph;

import perception_msgs.msg.dds.DetectableSceneNodeMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.ros2.ROS2Topic;

/**
 * Handles the ROS 2 publication of detected objects.
 */
public class ROS2DetectableSceneNode extends DetectableSceneNode
{
   private ROS2PublishSubscribeAPI ros2;
   private ROS2Topic<DetectableSceneNodeMessage> topic;
   private boolean detected = false;
   private final DetectableSceneNodeMessage detectableSceneNodeMessage = new DetectableSceneNodeMessage();

   public ROS2DetectableSceneNode(String name)
   {
      super(name);
   }

   public void setupForROS2Publishing(ROS2PublishSubscribeAPI ros2, ROS2Topic<DetectableSceneNodeMessage> topic)
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
//         detectableSceneNodeMessage.setId(id);

         MessageTools.toMessage(getReferenceFrame().getTransformToRoot(), detectableSceneNodeMessage.getTransformToWorld());

         ros2.publish(topic, detectableSceneNodeMessage);
      }
   }
}
