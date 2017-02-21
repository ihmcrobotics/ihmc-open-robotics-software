package us.ihmc.sensorProcessing.signalCorruption;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;


public class OrientationLatencyCorruptor implements SignalCorruptor<RotationMatrix>
{
   private final YoVariableRegistry registry;
   private final int latencyTicks;

   private final RotationMatrix[] orientations;
   private final IntegerYoVariable index;
   
   public OrientationLatencyCorruptor(String namePrefix, int latencyTicks, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      index = new IntegerYoVariable(namePrefix + "Index", registry);
      this.latencyTicks = latencyTicks;
      parentRegistry.addChild(registry);
      
      orientations = new RotationMatrix[latencyTicks+1]; 
      for (int i=0; i<=latencyTicks; i++)
      {
         RotationMatrix orientation = new RotationMatrix();
         orientation.setIdentity();
         
         orientations[i] = orientation;
      }
   }

   public void corrupt(RotationMatrix signal)
   {
      orientations[index.getIntegerValue()].set(signal);
      
      index.increment();
      if (index.getIntegerValue() >= latencyTicks)
      {
         index.set(0);
      }
 
      signal.set(orientations[index.getIntegerValue()]);
   }
}
