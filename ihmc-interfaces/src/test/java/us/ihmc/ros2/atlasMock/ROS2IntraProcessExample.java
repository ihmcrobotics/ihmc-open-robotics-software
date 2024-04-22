package us.ihmc.ros2.atlasMock;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2QosProfile;

import java.io.IOException;

public class ROS2IntraProcessExample
{
   public static void publishUsingIntraProcessNode()
   {
      try
      {
         Domain domain = DomainFactory.getDomain(PubSubImplementation.INTRAPROCESS);
         ROS2Node node = new ROS2Node(domain, "MockAtlasController");
         //      RosPublisher<AtlasRobotConfigurationData> publisher = node.createPublisher(new AtlasRobotConfigurationDataPubSubType(), "/robot_configuration_data");
         ROS2PublisherBasics<RobotConfigurationData> publisher = node.createPublisher(new RobotConfigurationDataPubSubType(), "/robot_configuration_data");

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
      Domain domain = DomainFactory.getDomain(PubSubImplementation.INTRAPROCESS);
      ROS2Node node = new ROS2Node(domain, "MockNetworkProcessor");
      node.createSubscription(new RobotConfigurationDataPubSubType(), subscriber -> {
         RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

         if (subscriber.takeNextData(robotConfigurationData, null))
         {
            //                  System.out.println(robotConfigurationData.getHeader().getStamp().getNanosec());
            System.out.println(robotConfigurationData.getMonotonicTime());
         }
      }, (subscriber, info) -> {
         System.out.println("Subscription matched!: " + subscriber.getAttributes().getTopicName() + " " + info.getStatus().name());
      }, "/robot_configuration_data", ROS2QosProfile.RELIABLE());
      Thread.currentThread().join();
   }

   public static void main(String[] args) throws IOException, InterruptedException
   {
      ThreadTools.startAThread(() -> publishUsingIntraProcessNode(), "publisher");
      subscribeUsingIntraProcessNode();
   }
}
