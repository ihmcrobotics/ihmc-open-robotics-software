package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import controller_msgs.msg.dds.DoorParameterPacket;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.Ros2Node;

public class DetectedObjectViewer
{
   private final Group root = new Group();

   public DetectedObjectViewer(Ros2Node ros2Node)
   {
      String doorParameterPacketTopicName = ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(DoorParameterPacket.class);
      ROS2Tools.createCallbackSubscription(ros2Node, DoorParameterPacket.class, doorParameterPacketTopicName, this::renderDoor);
   }

   public void renderDoor(Subscriber<DoorParameterPacket> subscriber)
   {
      LogTools.info("DoorParameterPacket received");
      DoorParameterPacket message = subscriber.takeNextData();
      System.out.println("getHingedPointOnGround "+ message.getHingedPointOnGround());
      System.out.println("getEndPointOnGround "+ message.getEndPointOnGround());
      System.out.println("getDoorHeight "+ message.getDoorHeight());
   }
   
   public void render()
   {

   }

   public void clear()
   {

   }
   
   public Node getRoot()
   {
      return root;
   }
}
