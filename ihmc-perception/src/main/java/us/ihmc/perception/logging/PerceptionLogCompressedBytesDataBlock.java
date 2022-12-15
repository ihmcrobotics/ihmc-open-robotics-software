package us.ihmc.perception.logging;

public class PerceptionLogCompressedBytesDataBlock extends PerceptionLogDataBlock
{
   private byte[] byteData;

   public PerceptionLogCompressedBytesDataBlock(byte[] buffer, long timestamp, long timeOfReception)
   {
      setTimeOfReception(timeOfReception);
      this.byteData = buffer;
   }
}
