package us.ihmc.robotics.time;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CallFrequencyCalculator
{
   private YoDouble yoCallFrequency;
   private YoDouble yoRequestDeltaTInMilliseconds;
   private int counter;
   private double lastTimeCalled = 0.0;
   private double currentTime = 0.0;
   private int numberOfSamples = 10;
   private double frequencyAddition = 0;

   public CallFrequencyCalculator(YoVariableRegistry registry, String prefix)
   {
      yoCallFrequency = new YoDouble((prefix + "_Freq"), registry);
      yoRequestDeltaTInMilliseconds = new YoDouble((prefix + "_DeltaT"), registry);
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
