package us.ihmc.communication.remote;

import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.io.Serializable;

abstract class CommsTester<T extends Serializable>
{
   protected class CommsTesterRunnable implements Runnable
   {
      protected final DataObjectTransponder sender;
      private static final boolean DEBUG = true;
      private final long dataObjectIdentifier;

      private boolean assertNoPackagesReceived = false;

      public CommsTesterRunnable(long dataObjectIdentifier, DataObjectTransponder sender)
      {
         this.dataObjectIdentifier = dataObjectIdentifier;
         this.sender = sender;
      }

      public void run()
      {
         for (int i = 0; i < numberOfPackets; i++)
         {
            T sendPacket = packetGenerator.generatePacket();
            long previousIndex = streamingDataConsumer.getLastPacketReceivedIndex();
            try
            {
               sender.sendData(dataObjectIdentifier, sendPacket);
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }

            tryToWaitUntilPacketIsReceived(previousIndex);
            T receivePacket = streamingDataConsumer.getLastReceivedPacket();

            if (assertNoPackagesReceived)
            {
               assertNull(receivePacket);
            }
            else
            {
               assertTrue(sendPacket.equals(receivePacket));
            }
         }

         isComplete = true;
         notifyTestListeners();
      }

      public void setAssertNoPackagesReceived(boolean assertNoPackagesReceived)
      {
         this.assertNoPackagesReceived = assertNoPackagesReceived;
      }

      private void tryToWaitUntilPacketIsReceived(long previousIndex)
      {
         try
         {
            synchronized (streamingDataConsumer)
            {
               while (streamingDataConsumer.getLastPacketReceivedIndex() == previousIndex)
               {
                  streamingDataConsumer.wait(maxPacketWaitTimeMillis);

                  break;
               }

               if (DEBUG)
               {
                  if (streamingDataConsumer.getLastPacketReceivedIndex() == previousIndex)
                  {
                     System.out.println("CommsTester: CommsTesterRunnable: tryToWaitUntilPacketIsReceived: Packet was not received.");
                  }
               }
            }
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }
   }


   // private static final int MAX_WAIT_CYCLES = DataObjectTransponderTest.MAXIMUM_INTER_PACKET_DELAY / DataObjectTransponderTest.MINIMUM_INTER_PACKET_DELAY;
   private boolean isComplete = false;
   protected final StreamingDataConsumerForTesting<T> streamingDataConsumer;
   protected final PacketGeneratorForTests<T> packetGenerator;
   private final int numberOfPackets;

   private final long maxPacketWaitTimeMillis;

   public CommsTester(StreamingDataConsumerForTesting<T> streamingDataConsumer, PacketGeneratorForTests<T> packetGenerator, int numberOfPackets,
                      long maxPacketWaitTimeMillis)
   {
      this.streamingDataConsumer = streamingDataConsumer;
      this.packetGenerator = packetGenerator;
      this.numberOfPackets = numberOfPackets;
      this.maxPacketWaitTimeMillis = maxPacketWaitTimeMillis;
   }


   public int getExpectedNumberOfPackets()
   {
      return numberOfPackets;
   }

   public long getLastPacketReceivedIndex()
   {
      return streamingDataConsumer.getLastPacketReceivedIndex();
   }

   public boolean isComplete()
   {
      return isComplete;
   }

   private synchronized void notifyTestListeners()
   {
      notifyAll();
   }

   abstract void setupClientDaemons(DataObjectTransponder transponder);

   abstract void setupClientStreamingDataConsumers(DataObjectTransponder transponder);

   abstract void setupServerDaemons(DataObjectTransponder transponder);

   abstract void setupServerStreamingDataConsumers(DataObjectTransponder transponder);
}
