package us.ihmc.rdx.perception;

import perception_msgs.msg.dds.DetectedObjectMessage;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.objects.DetectedObjectsInfo;
import us.ihmc.rdx.simulation.environment.object.objects.door.RDXVirtualGhostObject;
import us.ihmc.ros2.ROS2Topic;

/**
 * Keeps a ghost graphic object updated from a ROS 2 message.
 * Like a door, chair, can of soup, etc.
 */
public class RDXObjectPerceptionUpdater
{
   private final RDXVirtualGhostObject object;
   private final IHMCROS2Input<DetectedObjectMessage> subscription;
   private final String id;

   public RDXObjectPerceptionUpdater(ROS2PublishSubscribeAPI ros2,
                                     ROS2Topic<DetectedObjectMessage> updateTopic,
                                     String id,
                                     DetectedObjectsInfo objectsInfo,
                                     String frameName)
   {
      this.id = id;
      String modelFileName = objectsInfo.getModelFileName(id);
      object = new RDXVirtualGhostObject(modelFileName, frameName);

      subscription = ros2.subscribe(updateTopic);
   }

   public void update()
   {
      if (subscription.getMessageNotification().poll())
      {
         DetectedObjectMessage detectedObjectMessage = subscription.getMessageNotification().read();
         if (id.equals(detectedObjectMessage.getId().toString()))
         {
            object.setShowing(detectedObjectMessage.getDetected());
            MessageTools.toEuclid(detectedObjectMessage.getTransformToWorld(), object.getTransformToParent());
            object.update();
         }
      }
   }

   public RDXVirtualGhostObject getObject()
   {
      return object;
   }
}