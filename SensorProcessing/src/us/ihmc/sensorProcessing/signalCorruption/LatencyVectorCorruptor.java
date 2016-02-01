package us.ihmc.sensorProcessing.signalCorruption;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;


public class LatencyVectorCorruptor implements SignalCorruptor<Tuple3d>
{
   private final YoVariableRegistry registry;
   private final int latencyTicks;

   private final Tuple3d[] vectors;
   private final IntegerYoVariable index;
   
   public LatencyVectorCorruptor(String namePrefix, int latencyTicks, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      index = new IntegerYoVariable(namePrefix + "Index", registry);
      this.latencyTicks = latencyTicks;
      parentRegistry.addChild(registry);
      
      vectors = new Tuple3d[latencyTicks+1]; 
      for (int i=0; i<=latencyTicks; i++)
      {
         Vector3d tuple3d = new Vector3d();
         vectors[i] = tuple3d;
      }
   }

   public void corrupt(Tuple3d signal)
   {
      vectors[index.getIntegerValue()].set(signal);
      
      index.increment();
      if (index.getIntegerValue() >= latencyTicks)
      {
         index.set(0);
      }
 
      signal.set(vectors[index.getIntegerValue()]);
   }
}
