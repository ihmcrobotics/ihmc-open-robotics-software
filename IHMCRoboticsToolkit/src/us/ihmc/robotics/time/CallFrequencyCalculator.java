package us.ihmc.robotics.time;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class CallFrequencyCalculator
{
   private DoubleYoVariable yoCallFrequency;
   private DoubleYoVariable yoRequestDeltaTInMilliseconds;
   private int counter;
   private double lastTimeCalled = 0.0;
   private double currentTime = 0.0;
   private int numberOfSamples = 10;
   private double frequencyAddition = 0;

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

      double frequency = 1.0 / ((currentTime - lastTimeCalled) / 1.0E9);
      yoRequestDeltaTInMilliseconds.set((currentTime - lastTimeCalled) / 1.0E6);
      frequencyAddition = frequencyAddition + frequency;
      lastTimeCalled = currentTime;

      if (counter > numberOfSamples)
      {
         yoCallFrequency.set(frequencyAddition / counter);
         counter = 1;
         frequencyAddition = 0;
      }
      else
      {
         counter++;
      }

      return yoCallFrequency.getDoubleValue();
   }
}
