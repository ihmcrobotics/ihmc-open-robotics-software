package us.ihmc.robotDataLogger.dataBuffers;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class LoggerDebugRegistry 
{
   private final YoVariableRegistry registry = new YoVariableRegistry("LoggerDebugRegistry");
   private final YoInteger fullCircularBufferCounter = new YoInteger("FullCircularBuffer", registry);
   private final YoInteger lostTickInCircularBuffer = new YoInteger("lostTickInCircularBuffer", registry);
   
   public LoggerDebugRegistry(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }
   
   public void circularBufferFull()
   {
      fullCircularBufferCounter.increment();
   }

   public void lostTickInCircularBuffer()
   {
      lostTickInCircularBuffer.increment();
   }
   
}
