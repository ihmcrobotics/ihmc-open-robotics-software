package us.ihmc.communication.ros2;

import java.io.IOException;

import org.junit.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationOptions;
import us.ihmc.communication.IHMCInterfacesKryoNetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.idl.RecyclingArrayListPubSub;

public class Ros2PacketCommunicatorTest
{
   @Test(timeout = 60000)
   public void testRealPacketCommunicatorROS2() throws IOException
   {
      CommunicationOptions.USE_ROS2 = true;
      
      performRealTest();
   }

   @Test(timeout = 60000)
   public void testIntraprocessPacketCommunicatorROS2() throws IOException
   {
      CommunicationOptions.USE_ROS2 = true;
      
      performIntraprocessTest();
   }
   
   @Test(timeout = 60000)
   public void testRealPacketCommunicatorKryo() throws IOException
   {
      CommunicationOptions.USE_ROS2 = false;
      
      performRealTest();
   }

   @Test(timeout = 60000)
   public void testIntraprocessPacketCommunicatorKryo() throws IOException
   {
      CommunicationOptions.USE_ROS2 = false;
      
      performIntraprocessTest();
   }

   private void performRealTest() throws IOException
   {
      PacketCommunicator client = PacketCommunicator.createTCPPacketCommunicatorClient("localhost", NetworkPorts.CONTROLLER_PORT,
                                                                                       new IHMCInterfacesKryoNetClassList());
   
      PacketCommunicator server = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, new IHMCInterfacesKryoNetClassList());
   
      performMessageTest(client, server);
   }

   private void performIntraprocessTest() throws IOException
   {
      PacketCommunicator client = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT,
                                                                                       new IHMCInterfacesKryoNetClassList());

      PacketCommunicator server = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, new IHMCInterfacesKryoNetClassList());

      performMessageTest(client, server);
   }

   private void performMessageTest(PacketCommunicator client, PacketCommunicator server) throws IOException
   {
      client.attachGlobalListener(object -> {
         PrintTools.info("Received: " + object.getClass().getSimpleName());
      });

      client.connect();
      server.connect();

      for (int i = 0; i < 20; i++)
      {
         FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
         RecyclingArrayListPubSub<FootstepDataMessage> footstepDataList = footstepDataListMessage.getFootstepDataList();
         
         for (int j = 0; j < 10; j++)
         {
            FootstepDataMessage one = footstepDataList.add();
            one.getLocation().set(0.1 * i * j, 0.2 * i * j, 0.3 * i * j);
         }
         
         server.send(footstepDataListMessage);

         ThreadTools.sleep(10);
      }
   }
}
