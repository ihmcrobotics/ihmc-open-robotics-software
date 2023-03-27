package us.ihmc.rdx.perception;

      import perception_msgs.msg.dds.DetectedObjectMessage;
      import us.ihmc.communication.IHMCROS2Input;
      import us.ihmc.communication.packets.MessageTools;
      import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
      import us.ihmc.rdx.simulation.environment.object.objects.door.RDXVirtualGhostObject;
      import us.ihmc.ros2.ROS2Topic;

public class RDXObjectPerceptionUpdater
{
   private final RDXVirtualGhostObject object;
   private final IHMCROS2Input<DetectedObjectMessage> subscription;

   public RDXObjectPerceptionUpdater(ROS2PublishSubscribeAPI ros2, ROS2Topic<DetectedObjectMessage> updateTopic)
   {
      object = new RDXVirtualGhostObject(modelFileName, frameName);
      subscription = ros2.subscribe(updateTopic);
   }

   public void update()
   {
      if (subscription.getMessageNotification().poll())
      {
         DetectedObjectMessage detectedObjectMessage = subscription.getMessageNotification().read();
         object.setShowing(detectedObjectMessage.getDetected());
         MessageTools.toEuclid(detectedObjectMessage.getTransformToWorld(), object.getTransformToParent());
         object.update();
      }
   }

   public RDXVirtualGhostObject getObject()
   {
      return object;
   }
}