package us.ihmc.humanoidRobotics.communication.packets;

import static org.junit.Assert.assertEquals;

import java.io.IOException;

import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.Test;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;

public class RobotConfigurationDataPacketCommunicatorTest
{
   private static final int NUMBER_OF_MESSAGES_TO_SEND = 20;

   @Test(timeout = 60000)
   public void testRealPacketCommunicatorKryo() throws IOException
   {
      performRealTest();
   }

   @Test(timeout = 60000)
   public void testIntraprocessPacketCommunicatorKryo() throws IOException
   {
      performIntraprocessTest();
   }

   private void performRealTest() throws IOException
   {
      PacketCommunicator client = PacketCommunicator.createTCPPacketCommunicatorClient("localhost", NetworkPorts.CONTROLLER_PORT,
                                                                                       new IHMCCommunicationKryoNetClassList());
   
      PacketCommunicator server = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
   
      performMessageTest(client, server, "TCP Kryo: ");
   }

   private void performIntraprocessTest() throws IOException
   {
      PacketCommunicator client = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT,
                                                                                       new IHMCCommunicationKryoNetClassList());

      PacketCommunicator server = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());

      performMessageTest(client, server, "Intraprocess Kryo: ");
   }

   private void performMessageTest(PacketCommunicator client, PacketCommunicator server, String impl) throws IOException
   {
      MutableInt messageReceived = new MutableInt();
      client.attachListener(RobotConfigurationData.class, object -> {
         PrintTools.info(impl + " Received: sequence: " + object.getSequenceId() + " joints: " + object.getJointAngles().size() + " name: "
               + object.getClass().getSimpleName());
         messageReceived.increment();
      });

      client.connect();
      server.connect();
      
      ThreadTools.sleep(10); // these waits ensure all the messages have time to get over, lengthen a little if it fails

      for (int i = 0; i < NUMBER_OF_MESSAGES_TO_SEND; i++)
      {
         RobotConfigurationData robotConfigurationData = new RobotConfigurationData();
         us.ihmc.idl.IDLSequence.Float jointAngles = robotConfigurationData.getJointAngles();
         
         robotConfigurationData.setSequenceId(i);
         
         for (int j = 0; j < 31; j++)
         {
            jointAngles.add((float) (i * j));
         }
         
         server.send(robotConfigurationData);

         ThreadTools.sleep(10); // these waits ensure all the messages have time to get over, lengthen a little if it fails
      }
      
      ThreadTools.sleep(100);
      
      assertEquals("all messages not received.", NUMBER_OF_MESSAGES_TO_SEND, messageReceived.intValue(), 0);
   }
}
