package us.ihmc.sensorProcessing.signalCorruption;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;


public class OrientationLatencyCorruptor implements SignalCorruptor<Matrix3d>
{
   private final YoVariableRegistry registry;
   private final int latencyTicks;

   private final Matrix3d[] orientations;
   private final IntegerYoVariable index;
   
   public OrientationLatencyCorruptor(String namePrefix, int latencyTicks, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      index = new IntegerYoVariable(namePrefix + "Index", registry);
      this.latencyTicks = latencyTicks;
      parentRegistry.addChild(registry);
      
      orientations = new Matrix3d[latencyTicks+1]; 
      for (int i=0; i<=latencyTicks; i++)
      {
         Matrix3d orientation = new Matrix3d();
         orientation.setIdentity();
         
         orientations[i] = orientation;
      }
   }

   public void corrupt(Matrix3d signal)
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
