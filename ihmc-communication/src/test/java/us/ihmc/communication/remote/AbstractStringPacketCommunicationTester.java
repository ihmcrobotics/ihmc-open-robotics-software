package us.ihmc.communication.remote;

import java.math.BigInteger;
import java.util.Random;

import us.ihmc.communication.streamingData.StreamingDataConsumer;

abstract class AbstractStringPacketCommunicationTester extends CommsTester<StringPacket>
{
   public AbstractStringPacketCommunicationTester(int numberOfPackets, long maxPacketWaitTimeMillis)
   {
      super(new StringPacketStreamingDataConsumer(), new StringPacketGenerator(), numberOfPackets, maxPacketWaitTimeMillis);
   }

   private static class StringPacketStreamingDataConsumer extends StreamingDataConsumerForTesting<StringPacket> implements StreamingDataConsumer
   {
      public StringPacketStreamingDataConsumer()
      {
         super(StringPacket.getSerialVersionUID(), StringPacket.class);
      }
   }


   private static class StringPacketGenerator implements PacketGeneratorForTests<StringPacket>
   {
      // unseeded random intended here. It shouldn't matter, because any type of packet should be sent over and there are no epsilons involved
      private Random random = new Random();
      private final int numBits = 80;

      public StringPacket generatePacket()
      {
         String contents = new BigInteger(numBits, random).toString();

         return new StringPacket(contents);
      }
   }

}
