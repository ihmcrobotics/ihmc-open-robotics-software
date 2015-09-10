package us.ihmc.sensorProcessing.signalCorruption;

import java.util.Random;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class GaussianVectorCorruptor implements SignalCorruptor<Tuple3d>
{
   private final YoVariableRegistry registry;
   private final Random random;
   private final Vector3d noise = new Vector3d();
   private final DoubleYoVariable standardDeviation;

   public GaussianVectorCorruptor(long seed, String namePrefix, YoVariableRegistry parentRegistry)
   {
      this.random = new Random(seed);
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.standardDeviation = new DoubleYoVariable(namePrefix + "StdDev", parentRegistry);

      parentRegistry.addChild(registry);
   }

   public void corrupt(Tuple3d signal)
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
