package us.ihmc.communication.net.local;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

import org.junit.After;
import org.junit.Test;

import std_msgs.msg.dds.Float64;
import std_msgs.msg.dds.Int32;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class InterprocessObjectCommunicatorTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOpeningAndClosingALotOfPorts() throws IOException
   {
      TestNetClassList classList = new TestNetClassList();
      ArrayList<IntraprocessObjectCommunicator> communicators = new ArrayList<>();
      for (int i = 0; i < 2560; i += 20)
      {
         IntraprocessObjectCommunicator communicator = new IntraprocessObjectCommunicator(i, classList);
         communicator.connect();
         communicators.add(communicator);
      }
      assertTrue("Hashmap is empty", IntraprocessCommunicationNetwork.hasMap());
      assertEquals("Open ports does not equal number of communicators", communicators.size(), IntraprocessCommunicationNetwork.getOpenPorts());
      for (int i = 0; i < communicators.size(); i++)
      {
         communicators.get(i).disconnect();
      }

      assertEquals("Open ports does not equal zero", 0, IntraprocessCommunicationNetwork.getOpenPorts());
      assertFalse("Hashmap is not cleaned up", IntraprocessCommunicationNetwork.hasMap());

   }

	@ContinuousIntegrationTest(estimatedDuration = 1.4)
   @Test(timeout = 30000)
   public void testSendingObjectsToClients() throws IOException
   {
      IntraprocessObjectCommunicator port128ClientA = new IntraprocessObjectCommunicator(128, new TestNetClassList());
      IntraprocessObjectCommunicator port128ClientB = new IntraprocessObjectCommunicator(128, new TestNetClassList());
      IntraprocessObjectCommunicator port128ClientDisconnected = new IntraprocessObjectCommunicator(128, new TestNetClassList());
      IntraprocessObjectCommunicator port256Client = new IntraprocessObjectCommunicator(256, new TestNetClassList());

      // Check for not getting packets back I sent
      port128ClientA.attachListener(Int32.class, new FailConsumer<Int32>());
      // Check if no packets arrive at a disconnected listener
      port128ClientDisconnected.attachListener(Int32.class, new FailConsumer<Int32>());
      // Check if no packets arrive at a different port
      port256Client.attachListener(Int32.class, new FailConsumer<Int32>());
      // Check if no packets of the wrong type get received
      port128ClientB.attachListener(Float64.class, new FailConsumer<Float64>());

      port128ClientB.attachListener(Int32.class, new ObjectConsumer<Int32>()
      {
         Random random = new Random(1511358L);

         @Override
         public void consumeObject(Int32 object)
         {
            assertEquals(random.nextInt(), object.getData());
         }
      });

      port128ClientA.connect();
      port128ClientB.connect();
      port256Client.connect();

      Random random = new Random(1511358L);
      int iterations = 1200000;
      for (int i = 0; i < iterations; i++)
      {
         Int32 object = new Int32();
         object.setData(random.nextInt());
         port128ClientA.consumeObject(object);
      }

      port128ClientA.disconnect();
      port128ClientB.disconnect();
      port256Client.disconnect();

      assertEquals("Open ports does not equal zero", 0, IntraprocessCommunicationNetwork.getOpenPorts());
      assertFalse("Hashmap is not cleaned up", IntraprocessCommunicationNetwork.hasMap());

   }

   @After
   public void closeNetwork()
   {
      IntraprocessCommunicationNetwork.closeAllConnectionsForMyJUnitTests();
   }

   private final class FailConsumer<T extends Packet<T>> implements ObjectConsumer<T>
   {
      @Override
      public void consumeObject(T object)
      {
         fail();
      }
   }

   private static class TestNetClassList extends NetClassList
   {
      public TestNetClassList()
      {
         registerPacketClass(Int32.class);
         registerPacketClass(Float64.class);
      }
   }
}
