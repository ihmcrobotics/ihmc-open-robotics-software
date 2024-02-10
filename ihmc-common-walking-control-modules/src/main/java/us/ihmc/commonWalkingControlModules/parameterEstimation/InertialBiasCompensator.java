package us.ihmc.commonWalkingControlModules.parameterEstimation;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class InertialBiasCompensator
{
   private int counter;
   private final int windowSize;
   private final double[][] measurements;
   private final YoDouble[] bias;

   public InertialBiasCompensator(int nDoFs, int windowSize, String[] rowNames, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry("InertialBiasCompensator");
      parentRegistry.addChild(registry);

      counter = 0;
      this.windowSize = windowSize;
      measurements = new double[nDoFs][windowSize];
      bias = new YoDouble[nDoFs];
      for (int i = 0; i < nDoFs; ++i)
      {
         bias[i] = new YoDouble("bias_" + rowNames[i], registry);
      }
   }

   public void update(int index, double measurement)
   {
      if (counter < windowSize)
         measurements[index][counter] = measurement;
      else  // catches failure cases where the number of measurements would otherwise be overfilled and overwritten
         calculateBias();
   }

   public void calculateBias()
   {
      for (int i = 0; i < measurements.length; ++i)
      {
         double sum = 0;
         for (int j = 0; j < windowSize; ++j)
            sum += measurements[i][j];

         bias[i].set(sum / windowSize);
      }
   }

   public void incrementCounter()
   {
      counter++;
   }

   public void resetCounter()
   {
      counter = 0;
   }

   public void zero()
   {
      for (int i = 0; i < bias.length; ++i)
      {
         bias[i].set(0.0);
      }
   }

   public boolean isWindowFilled()
   {
      return counter == windowSize;
   }

   public double getBias(int index)
   {
      return bias[index].getValue();
   }
}
