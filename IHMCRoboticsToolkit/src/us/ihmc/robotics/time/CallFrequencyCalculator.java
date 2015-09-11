package us.ihmc.robotics.time;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class CallFrequencyCalculator
{
   private DoubleYoVariable yoCallFrequency;
   private DoubleYoVariable yoRequestDeltaTInMilliseconds;
   private int counter;
   private double deltaTime = 0.0;
   private double lastTimeCalled = 0.0;
   private double currentTime = 0.0;
   private int numberOfSamples = 10;

   public CallFrequencyCalculator(YoVariableRegistry registry, String prefix)
   {
      yoCallFrequency = new DoubleYoVariable((prefix + "_Freq"), registry);
      yoRequestDeltaTInMilliseconds = new DoubleYoVariable((prefix + "_DeltaT"), registry);
   }

   public void setNumberOfSamples(int numberOfSamples)
   {
      this.numberOfSamples = numberOfSamples;
   }

   public double determineCallFrequency()
   {
      currentTime = System.nanoTime();

      yoRequestDeltaTInMilliseconds.set((currentTime - lastTimeCalled) / 1.0E6);

      deltaTime = deltaTime + yoRequestDeltaTInMilliseconds.getDoubleValue();
      lastTimeCalled = currentTime;

      if (counter > numberOfSamples)
      {
         yoCallFrequency.set(1000.0 / ((deltaTime / numberOfSamples)));
         deltaTime = 0;
         counter = 0;
      }
      else
      {
         counter++;
      }

      return yoCallFrequency.getDoubleValue();
   }
}
