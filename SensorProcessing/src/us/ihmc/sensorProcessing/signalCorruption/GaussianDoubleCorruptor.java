package us.ihmc.sensorProcessing.signalCorruption;

import java.util.Random;

import org.apache.commons.lang.mutable.MutableDouble;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.DoubleYoVariable;

public class GaussianDoubleCorruptor implements SignalCorruptor<MutableDouble>
{
   private final YoVariableRegistry registry;
   private final Random random;
   private final DoubleYoVariable standardDeviation;

   public GaussianDoubleCorruptor(long seed, String namePrefix, YoVariableRegistry parentRegistry)
   {
      this.random = new Random(seed);
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.standardDeviation = new DoubleYoVariable(namePrefix + "StdDev", parentRegistry);

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
