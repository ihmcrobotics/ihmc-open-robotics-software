package us.ihmc.sensorProcessing.signalCorruption;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;


public class LatencyVectorCorruptor implements SignalCorruptor<Tuple3DBasics>
{
   private final YoVariableRegistry registry;
   private final int latencyTicks;

   private final Tuple3DBasics[] vectors;
   private final IntegerYoVariable index;
   
   public LatencyVectorCorruptor(String namePrefix, int latencyTicks, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      index = new IntegerYoVariable(namePrefix + "Index", registry);
      this.latencyTicks = latencyTicks;
      parentRegistry.addChild(registry);
      
      vectors = new Tuple3DBasics[latencyTicks+1]; 
      for (int i=0; i<=latencyTicks; i++)
      {
         Vector3D tuple3d = new Vector3D();
         vectors[i] = tuple3d;
      }
   }

   public void corrupt(Tuple3DBasics signal)
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
