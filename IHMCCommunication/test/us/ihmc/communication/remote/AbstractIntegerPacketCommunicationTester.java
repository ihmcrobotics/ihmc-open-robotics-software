package us.ihmc.communication.remote;



abstract class AbstractIntegerPacketCommunicationTester extends CommsTester<IntegerPacket>
{
   public AbstractIntegerPacketCommunicationTester(int numberOfPackets, long maxPacketWaitTimeMillis)
   {
      super(new IntegerPacketStreamingDataConsumer(), new IntPacketGenerator(), numberOfPackets, maxPacketWaitTimeMillis);
   }

   protected static class IntegerPacketStreamingDataConsumer extends StreamingDataConsumerForTesting<IntegerPacket>
   {
      public IntegerPacketStreamingDataConsumer()
      {
         super(IntegerPacket.getSerialVersionUID(), IntegerPacket.class);
      }
   }

   protected static class IntPacketGenerator implements PacketGeneratorForTests<IntegerPacket>
   {
      private int currentIndex = 0;
      
      public IntegerPacket generatePacket()
      {
         return new IntegerPacket(currentIndex++);
      }
   }
}
