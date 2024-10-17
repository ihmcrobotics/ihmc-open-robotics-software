package us.ihmc.communication.remote;

import static us.ihmc.robotics.Assert.*;

import java.io.IOException;
import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.Assertions;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.commons.thread.ThreadTools;

@Disabled("This is really old code we don't need anymore.")
public class DataObjectTransponderTest
{
   private static final int MAXIMUM_INTER_PACKET_DELAY_MILLIS = 300;
   private static final boolean DEBUG = false;


   private final ArrayList<CommsTester<?>> tests = new ArrayList<CommsTester<?>>();

   public static void waitOnTransponder(DataObjectTransponder transponder)
   {
      synchronized (transponder)
      {
         while (!transponder.isConnected())
         {
            try
            {
               transponder.wait();
            }
            catch (InterruptedException e)
            {
               throw new RuntimeException(e.getMessage());
            }
         }
      }
   }
   
   public static void waitOnTransponderWithTimeout(DataObjectTransponder transponder, long millisecondTimeout)
   {
      long startTime = System.currentTimeMillis();
      long time=0;
      synchronized (transponder)
      {
         while (!transponder.isConnected() && time<=millisecondTimeout)
         {
            try
            {
               time = System.currentTimeMillis()-startTime;
               if (DEBUG)
                  System.out.println("DataObjectTransponderTest.waitOnTransponderWithTimeout Time: " +Long.toString(time));
               transponder.wait(ThreadTools.REASONABLE_WAITING_SLEEP_DURATION_MS);
            }
            catch (InterruptedException e)
            {
               throw new RuntimeException(e.getMessage());
            }
         }
      }
   }

   public void assertAllTestsPassed()
   {
      for (CommsTester<?> test : tests)
      {
         assertTestPassed(test);
      }
   }

	@Test
   public void testBidirectionalCommunication() throws InterruptedException
   {
      int port = 1341;
      int numberOfPackets = 50;
      tests.add(new ServerToClientIntPacketCommunicationTester(numberOfPackets, MAXIMUM_INTER_PACKET_DELAY_MILLIS));
      tests.add(new ClientToServerStringPacketCommunicationTester(numberOfPackets, MAXIMUM_INTER_PACKET_DELAY_MILLIS));
      startTestsAndWaitUntilFinished(port);
      assertAllTestsPassed();
   }

   @Disabled
   @Test
   public void testDoubleBidirectionalCommunication() throws InterruptedException
   {
      int numberOfPackets = 60000;
      int port = 1342;
      tests.add(new ServerToClientIntPacketCommunicationTester(numberOfPackets, MAXIMUM_INTER_PACKET_DELAY_MILLIS));
      tests.add(new ClientToServerStringPacketCommunicationTester(numberOfPackets, MAXIMUM_INTER_PACKET_DELAY_MILLIS));
      tests.add(new ServerToClientStringPacketCommunicationTester(numberOfPackets, MAXIMUM_INTER_PACKET_DELAY_MILLIS));
      tests.add(new ClientToServerIntPacketCommunicationTester(numberOfPackets, MAXIMUM_INTER_PACKET_DELAY_MILLIS));
      startTestsAndWaitUntilFinished(port);
      assertAllTestsPassed();
   }

   @Test
   public void testSerializabilityOfInteger() throws IOException
   {
      Integer integer = Integer.valueOf(2);
      Assertions.assertSerializable(integer);
   }

   @Test
   public void testSerializabilityOfIntPacket() throws IOException
   {
      IntegerPacket intPacket = new IntegerPacket(2);
      Assertions.assertSerializable(intPacket);
   }

   @Test
   public void testSerializabilityOfStringPacket() throws IOException
   {
      StringPacket packet = new StringPacket("Buzzap!");
      Assertions.assertSerializable(packet);
   }

   @Test
   public void testServerClientIntPacketCommunication() throws InterruptedException
   {
      int port = 1337;
      int numberOfPackets = 50;
      tests.add(new ServerToClientIntPacketCommunicationTester(numberOfPackets, MAXIMUM_INTER_PACKET_DELAY_MILLIS));
      startTestsAndWaitUntilFinished(port);
      assertAllTestsPassed();
   }

   @Test
   public void testServerClientStringPacketCommunication() throws InterruptedException
   {
      int port = 1338;
      int numberOfPackets = 50;
      tests.add(new ServerToClientStringPacketCommunicationTester(numberOfPackets, MAXIMUM_INTER_PACKET_DELAY_MILLIS));
      startTestsAndWaitUntilFinished(port);
      assertAllTestsPassed();
   }

   @Test
   public void testTwoPacketTypesAtTheSameTime() throws InterruptedException
   {
      int port = 1339;
      int numberOfPackets = 50;
      tests.add(new ServerToClientIntPacketCommunicationTester(numberOfPackets, MAXIMUM_INTER_PACKET_DELAY_MILLIS));
      tests.add(new ServerToClientStringPacketCommunicationTester(numberOfPackets, MAXIMUM_INTER_PACKET_DELAY_MILLIS));
      startTestsAndWaitUntilFinished(port);
      assertAllTestsPassed();
   }

   @Test
   public void testUnhandledPackets() throws InterruptedException
   {
      int port = 1340;
      int numberOfPackets = 50;
      ServerToClientIntPacketCommunicationTester normalIntPacketTest = new ServerToClientIntPacketCommunicationTester(numberOfPackets, MAXIMUM_INTER_PACKET_DELAY_MILLIS);
      ServerToClientStringPacketCommunicationTester unhandledStringPacketTest = new ServerToClientStringPacketCommunicationTester(numberOfPackets, MAXIMUM_INTER_PACKET_DELAY_MILLIS)
      {
         void setupClientStreamingDataConsumers(DataObjectTransponder transponder)
         {
            transponder.setIsSilent(true); // to get rid of exceptions being thrown
         }
         void setupServerDaemons(DataObjectTransponder transponder)
         {
            CommsTesterRunnable serverConnectionDaemon = new CommsTesterRunnable(StringPacket.getSerialVersionUID(), transponder);
            serverConnectionDaemon.setAssertNoPackagesReceived(true);
            ThreadTools.startAsDaemon(serverConnectionDaemon, "Data Object Server Conn Daemon");
         }
      };
      tests.add(normalIntPacketTest);
//      tests.add(unhandledStringPacketTest);
      startTestsAndWaitUntilFinished(port);
      assertTestPassed(normalIntPacketTest);
      assertTestFailedCompletely(unhandledStringPacketTest);
   }

   private void assertTestFailedCompletely(CommsTester<?> test)
   {
      assertEquals(0, test.getLastPacketReceivedIndex());
   }

   private void assertTestPassed(CommsTester<?> test)
   {
      assertEquals(test.getExpectedNumberOfPackets(), test.getLastPacketReceivedIndex());
   }

   private void startTestsAndWaitUntilFinished(int port) throws InterruptedException
   {
      Thread clientThread = new Thread(new ObjectClientInitiatorThread(port));
      clientThread.start();
      Thread serverThread = new Thread(new ObjectServerInitiatorThread(port));
      serverThread.start();
      waitAroundUntilEndOfAllCommsTests();
   }


   private void waitAroundUntilEndOfAllCommsTests() throws InterruptedException
   {
      for (CommsTester<?> test : tests)
      {
         synchronized (test)
         {
            while (!test.isComplete())
            {
               test.wait();
            }
         }
      }
   }

   private class ObjectClientInitiatorThread extends DataObjectTransponderInitiatorThread implements Runnable
   {

      public ObjectClientInitiatorThread(int port)
      {
         super(port);
      }

      protected DataObjectTransponder getNewTransponder()
      {
         return new DataObjectClient("localhost", port);
      }

      String getName()
      {
         return "ClientTransponder";
      }

      void setupConsumers(CommsTester<?> test, DataObjectTransponder transponder)
      {
         test.setupClientStreamingDataConsumers(transponder);
      }

      void setupProducerDaemons(CommsTester<?> test, DataObjectTransponder transponder)
      {
         test.setupClientDaemons(transponder);
         
      }
   }


   private class ObjectServerInitiatorThread extends DataObjectTransponderInitiatorThread implements Runnable
   {
      public ObjectServerInitiatorThread(int port)
      {
         super(port);
      }

      protected DataObjectTransponder getNewTransponder()
      {
         return new DataObjectServer(port);
      }

      String getName()
      {
         return "ServerTransponder";
      }

      @Override
      void setupConsumers(CommsTester<?> test, DataObjectTransponder transponder)
      {
         test.setupServerStreamingDataConsumers(transponder);      
      }

      @Override
      void setupProducerDaemons(CommsTester<?> test, DataObjectTransponder transponder)
      {
         test.setupServerDaemons(transponder);     
      }
   }
   
   private abstract class DataObjectTransponderInitiatorThread implements Runnable
   {
      protected int port;

      public DataObjectTransponderInitiatorThread(int port)
      {
         this.port = port;
      }
      public void run()
      {
         DataObjectTransponder transponder = getNewTransponder();
         transponder.setName(getName());

         for (CommsTester<?> test : tests)
         {
            setupConsumers(test, transponder);
         }
         waitOnTransponder(transponder);
         for (CommsTester<?> test : tests)
         {
            setupProducerDaemons(test,transponder);
         }
      }
      abstract void setupConsumers(CommsTester<?> test, DataObjectTransponder transponder);
      abstract void setupProducerDaemons(CommsTester<?> test, DataObjectTransponder transponder);
      abstract DataObjectTransponder getNewTransponder();
      abstract String getName();
   }
}
