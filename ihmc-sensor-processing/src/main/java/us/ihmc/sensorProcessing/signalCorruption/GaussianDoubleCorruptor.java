package us.ihmc.sensorProcessing.signalCorruption;

import java.util.Random;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class GaussianDoubleCorruptor implements SignalCorruptor<MutableDouble>
{
   private final YoVariableRegistry registry;
   private final Random random;
   private final YoDouble standardDeviation;

   public GaussianDoubleCorruptor(long seed, String namePrefix, YoVariableRegistry parentRegistry)
   {
      this.random = new Random(seed);
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.standardDeviation = new YoDouble(namePrefix + "StdDev", parentRegistry);

      parentRegistry.addChild(registry);
   }

   public void corrupt(MutableDouble signal)
   {
      double std = standardDeviation.getDoubleValue();
      double noise = std * random.nextGaussian();
      signal.add(noise);
   }

   public void setStandardDeviation(double standardDeviation)
   {
      this.standardDeviation.set(standardDeviation);
   }
}
