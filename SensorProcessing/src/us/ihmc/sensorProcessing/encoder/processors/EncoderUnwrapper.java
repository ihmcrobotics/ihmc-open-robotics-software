package us.ihmc.sensorProcessing.encoder.processors;

public class EncoderUnwrapper
{

   private final int tickPerRev;
   private  int tickOffset;
   private boolean isFirstTick;
   private  int lastTick;
   
   public EncoderUnwrapper(int tickPerRev)
   {
      this.tickPerRev = tickPerRev;
      this.tickOffset = 0;
      this.isFirstTick = true;
   }
   
   public int update(int rawTick)
   {
      if(isFirstTick)
      {
              isFirstTick=false;
              lastTick = rawTick;
              return rawTick;
      }

      int diff = rawTick - lastTick;
      if(diff > tickPerRev/2)
      {
                      tickOffset -= tickPerRev;
      }
      else if(diff < -tickPerRev/2)
      {
                      tickOffset += tickPerRev;
      }
      lastTick = rawTick;
      return tickOffset + rawTick;
      
   }
}
