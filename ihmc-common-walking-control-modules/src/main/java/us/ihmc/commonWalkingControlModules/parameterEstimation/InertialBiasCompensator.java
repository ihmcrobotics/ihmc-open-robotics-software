package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class InertialBiasCompensator
{
   private int counter;
   private final int windowSize;
   private final double[][] measurements;
   private final YoMatrix bias;

   private final DMatrixRMaj zeroMatrix;

   public InertialBiasCompensator(int nDoFs, int windowSize, String[] rowNames, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry("InertialBiasCompensator");
      parentRegistry.addChild(registry);

      counter = 0;
      this.windowSize = windowSize;
      measurements = new double[nDoFs][windowSize];
      bias = new YoMatrix("bias", nDoFs, 1, rowNames, null, registry);

      zeroMatrix = new DMatrixRMaj(nDoFs, 1);
      zeroMatrix.zero();
   }

   public void update(DMatrix measurement)
   {
      if (counter < windowSize)
         for (int index = 0; index < measurement.getNumRows(); ++index)
            measurements[index][counter] = measurement.get(index, 0);
      else
         LogTools.info("InertialBiasCompensator: window size exceeded, measurement not stored");
   }

   public void calculateBias()
   {
      if (isWindowFilled())
      {
         for (int i = 0; i < measurements.length; ++i)
         {
            double sum = 0;
            for (int j = 0; j < windowSize; ++j)
               sum += measurements[i][j];

            bias.set(i, 0, sum / windowSize);
         }
      }
      else
      {
         LogTools.info("InertialBiasCompensator: window not filled, bias not calculated");
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

   public void reset()
   {
      counter = 0;

      for (int i = 0; i < measurements.length; ++i)
         for (int j = 0; j < windowSize; ++j)
            measurements[i][j] = 0;

      bias.zero();
   }

   public boolean isWindowFilled()
   {
      return counter == windowSize;
   }

   public DMatrix getBias()
   {
      return bias;
   }

   public DMatrixRMaj getZero()
   {
      return zeroMatrix;
   }
}
