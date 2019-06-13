package us.ihmc.ros2.atlasMock;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2Publisher;
import us.ihmc.ros2.Ros2QosProfile;

import java.io.IOException;

public class Ros2IntraProcessExample
{
   public static void publishUsingIntraProcessNode()
   {
      try
      {
         Ros2Node node = new Ros2Node(PubSubImplementation.INTRAPROCESS, "MockAtlasController");
         //      RosPublisher<AtlasRobotConfigurationData> publisher = node.createPublisher(new AtlasRobotConfigurationDataPubSubType(), "/robot_configuration_data");
         Ros2Publisher<RobotConfigurationData> publisher = node.createPublisher(new RobotConfigurationDataPubSubType(), "/robot_configuration_data");

         for (int i = 0; true; i++)
         {
            //         AtlasRobotConfigurationData robotConfigurationData = new AtlasRobotConfigurationData();
            RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

            //            robotConfigurationData.getHeader().getStamp().setNanosec(i);
            robotConfigurationData.setWallTime(i);
            robotConfigurationData.setMonotonicTime(i);
            System.out.println("Publishing: " + i);
            publisher.publish(robotConfigurationData);
            Thread.sleep(1000);
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static void subscribeUsingIntraProcessNode() throws IOException, InterruptedException
   {
      Ros2Node node = new Ros2Node(PubSubImplementation.INTRAPROCESS, "MockNetworkProcessor");
      node.createSubscription(new RobotConfigurationDataPubSubType(), subscriber -> {
         RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

         if (subscriber.takeNextData(robotConfigurationData, null))
         {
            //                  System.out.println(robotConfigurationData.getHeader().getStamp().getNanosec());
            System.out.println(robotConfigurationData.getMonotonicTime());
         }
      }, (subscriber, info) -> {
         System.out.println("Subscription matched!: " + subscriber.getAttributes().getTopic().getTopicName() + " " + info.getStatus().name());
      }, "/robot_configuration_data", Ros2QosProfile.DEFAULT());
      Thread.currentThread().join();
   }

   public static void main(String[] args) throws IOException, InterruptedException
   {
      ThreadTools.startAThread(() -> publishUsingIntraProcessNode(), "publisher");
      subscribeUsingIntraProcessNode();
   }
}
