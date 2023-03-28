package us.ihmc.rdx.perception;

import com.esotericsoftware.minlog.Log;
import perception_msgs.msg.dds.DetectedObjectMessage;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.log.LogTools;
import us.ihmc.perception.objects.ObjectInfo;
import us.ihmc.rdx.simulation.environment.object.objects.door.RDXVirtualGhostObject;
import us.ihmc.ros2.ROS2Topic;

public class RDXObjectPerceptionUpdater
{
   private final RDXVirtualGhostObject object;
   private final IHMCROS2Input<DetectedObjectMessage> subscription;
   private final String ID;

   public RDXObjectPerceptionUpdater(ROS2PublishSubscribeAPI ros2, ROS2Topic<DetectedObjectMessage> updateTopic, String ID, ObjectInfo objectInfo)
   {
      this.ID = ID;
      String modelFileName = objectInfo.getModelFileName(ID);
      object = new RDXVirtualGhostObject(modelFileName);

      subscription = ros2.subscribe(updateTopic);
   }

   public void update()
   {
      if (subscription.getMessageNotification().poll())
      {
         DetectedObjectMessage detectedObjectMessage = subscription.getMessageNotification().read();
         if(ID.equals(detectedObjectMessage.getId().toString()))
         {
            object.setShowing(detectedObjectMessage.getDetected());
            MessageTools.toEuclid(detectedObjectMessage.getTransformToWorld(), object.getTransformToParent());
            object.update();
            LogTools.info("Receiving object {} {}", ID, detectedObjectMessage.getDetected());
         }
         else
            object.setShowing(false);
      }
   }

   public RDXVirtualGhostObject getObject()
   {
      return object;
   }
}