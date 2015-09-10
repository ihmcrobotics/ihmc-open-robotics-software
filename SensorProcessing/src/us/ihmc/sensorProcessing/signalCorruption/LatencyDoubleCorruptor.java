package us.ihmc.sensorProcessing.signalCorruption;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;


public class LatencyDoubleCorruptor implements SignalCorruptor<MutableDouble>
{
   private final YoVariableRegistry registry;
   private final int latencyTicks;

   private final MutableDouble[] doubles;
   private final IntegerYoVariable index;
   
   public LatencyDoubleCorruptor(String namePrefix, int latencyTicks, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      index = new IntegerYoVariable(namePrefix + "Index", registry);
      this.latencyTicks = latencyTicks;
      parentRegistry.addChild(registry);
      
      doubles = new MutableDouble[latencyTicks+1]; 
      for (int i=0; i<=latencyTicks; i++)
      {
         MutableDouble mutableDouble = new MutableDouble();
         doubles[i] = mutableDouble;
      }
   }

   public void corrupt(MutableDouble signal)
   {
      doubles[index.getIntegerValue()].setValue(signal);
      
      index.increment();
      if (index.getIntegerValue() >= latencyTicks)
      {
         index.set(0);
      }
 
      signal.setValue(doubles[index.getIntegerValue()]);
   }
}
