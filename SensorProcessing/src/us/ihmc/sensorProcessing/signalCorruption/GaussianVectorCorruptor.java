package us.ihmc.sensorProcessing.signalCorruption;

import java.util.Random;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class GaussianVectorCorruptor implements SignalCorruptor<Tuple3DBasics>
{
   private final YoVariableRegistry registry;
   private final Random random;
   private final Vector3D noise = new Vector3D();
   private final DoubleYoVariable standardDeviation;

   public GaussianVectorCorruptor(long seed, String namePrefix, YoVariableRegistry parentRegistry)
   {
      this.random = new Random(seed);
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.standardDeviation = new DoubleYoVariable(namePrefix + "StdDev", parentRegistry);

      parentRegistry.addChild(registry);
   }

   public void corrupt(Tuple3DBasics signal)
   {
      double std = standardDeviation.getDoubleValue();
      double noiseX = std * random.nextGaussian();
      double noiseY = std * random.nextGaussian();
      double noiseZ = std * random.nextGaussian();
      noise.set(noiseX, noiseY, noiseZ);
      signal.add(noise);
   }

   public void setStandardDeviation(double standardDeviation)
   {
      this.standardDeviation.set(standardDeviation);
   }
}
