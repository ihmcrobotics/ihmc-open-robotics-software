package us.ihmc.perception.logging;

public class PerceptionLogFloatSequenceDataBlock extends PerceptionLogDataBlock
{
   private float[] floatData;
   private long[] timestamps;

   public PerceptionLogFloatSequenceDataBlock(float[] buffer, long[] timestamps, long timeOfReception)
   {
      setTimeOfReception(timeOfReception);
      this.floatData = buffer;
      this.timestamps = timestamps;
   }
}
