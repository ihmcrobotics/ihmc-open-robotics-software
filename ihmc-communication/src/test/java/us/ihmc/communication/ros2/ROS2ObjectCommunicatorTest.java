package us.ihmc.communication.ros2;

import java.io.IOException;

import org.junit.Test;

import std_msgs.msg.dds.Int64;
import std_msgs.msg.dds.Int64PubSubType;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2ObjectPublisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.interfaces.IHMCInterfaces;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.ros2.RealtimeRos2Subscription;

public class ROS2ObjectCommunicatorTest
{
   @SuppressWarnings("unchecked")
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testROS2ObjectCommunicator() throws IOException
   {
      NetClassList netClassList = new NetClassList();
      netClassList.registerPacketClass(Int64.class);
//      String topicName = NetworkPorts.CONTROLLER_PORT.getName();
      String topicName = "example";

      if (!IHMCInterfaces.contains(Int64.class))
         throw new AssertionError("Add Int64 to IHMCInterfaces!");

      // Create realtime ROS2 node
      PrintTools.info("Creating node: topicName: " + topicName);
      PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;
      RealtimeRos2Node realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, topicName, ROS2Tools.RUNTIME_EXCEPTION);
      
      // Create publisher using tools
      ROS2ObjectPublisher<Int64> publisher = ROS2Tools.createPublisher(pubSubImplementation, realtimeRos2Node, IHMCInterfaces.getPubSubType(Int64.class), topicName,
                                                                       ROS2Tools.RUNTIME_EXCEPTION);
      

      realtimeRos2Node.spin();
      
      Int64 unpackingMessage = (Int64) ROS2Tools.createMessage(Int64.class, ROS2Tools.RUNTIME_EXCEPTION);
      
      // Create subscription using example way
      try
      {
         RealtimeRos2Subscription<Int64> createQueuedSubscription = realtimeRos2Node.createQueuedSubscription(new Int64PubSubType(), topicName);
         final Int64 data = new Int64();
         ThreadTools.startAThread(() -> {
            while (true)
            {
               if (createQueuedSubscription.poll(data))

                  System.out.println("Received: " + data.toString());

               Thread.yield();
            }

         }, "Subscriber");
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }

      
      
      // Create subscription using tools
      ROS2Tools.createCallbackSubscription(pubSubImplementation, realtimeRos2Node, IHMCInterfaces.getPubSubType(Int64.class), topicName, subscriber -> {
         ROS2Tools.popMessage(subscriber, unpackingMessage, new SampleInfo());

         System.out.println("Received2:  " + unpackingMessage);

      }, ROS2Tools.RUNTIME_EXCEPTION);

      
      // Publish 10 messages
      for (int i = 0; i < 10; i++)
      {
         Int64 message = new Int64();
         message.setData(i);
         publisher.publish(message);
         System.out.println("Publishing: " + message.toString());

//         ThreadTools.sleep(500);
      }
      
      RealtimeRos2Subscription<Int64> subscription = realtimeRos2Node.createQueuedSubscription(new Int64PubSubType(), "/example");
      Int64 incomingMessage = new Int64();
      while (!subscription.poll(incomingMessage))
         ; // just waiting for the first message
      System.out.println(incomingMessage); // first message
      int i = 1;
      while (i < 10)
      {
         if (subscription.poll(incomingMessage))
         {
            System.out.println("Receiving: " + incomingMessage);
            i++;
         }
         else
         {
            // no available messages
         }
      }
   }
}
