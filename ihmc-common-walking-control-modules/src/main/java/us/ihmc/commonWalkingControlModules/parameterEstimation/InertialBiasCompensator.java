package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * This class is used to calculate a constant approximation to the offset seen in {@link  InertialParameterManager} estimation residuals on the real robot.
 * <p>
 * The filters used inside {@link InertialParameterManager} assume that the residuals in the estimation process (see {@code residual} in that class) are
 * zero-mean. This is not the case in practice due to various inertial parameter non-idealities. The bias compensator is used to calculate a constant
 * approximation to the offset by storing samples of the residuals over a window into the past and then averaging them. This is a simple, constant
 * approximation to a bias that is in reality state-dependent and time-varying.
 * </p>
 *
 * @author James Foster
 */
public class InertialBiasCompensator
{
   /** Keeps track of the progress through the measurement window, is updated after each new set of measurements. */
   private int counter;
   /** The number of measurements to be used to calculate the bias. */
   private final int windowSize;
   /**
    * Where the measurements used to calculate the bias ae stored, the first index is the considered degree of freedom, the second index is the tick in the
    * measurement window.
    */
   private final double[][] measurements;
   /** Where the bias for each degree of freedom is stored after it is calculated. */
   private final YoMatrix bias;

   /** If we choose to temporarily exclude the bias, but not overwrite its value,  we need an appropriately sized zero matrix as a placeholder. */
   private static DMatrixRMaj ZERO_MATRIX;

   public InertialBiasCompensator(int nDoFs, int windowSize, String[] rowNames, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry("InertialBiasCompensator");
      parentRegistry.addChild(registry);

      counter = 0;
      this.windowSize = windowSize;
      measurements = new double[nDoFs][windowSize];
      bias = new YoMatrix("bias", nDoFs, 1, rowNames, null, registry);

      ZERO_MATRIX = new DMatrixRMaj(nDoFs, 1);
      ZERO_MATRIX.zero();  // Just to make sure
   }

   /**
    * Updates the measurements with the new measurement, if the window is not full. If the window is full, the measurement is not stored.
    *
    * @param measurement the new measurement to be stored.
    */
   public void update(DMatrix measurement)
   {
      if (counter < windowSize)
         for (int index = 0; index < measurement.getNumRows(); ++index)
            measurements[index][counter] = measurement.get(index, 0);
      else
         LogTools.info("InertialBiasCompensator: window size exceeded, measurement not stored");
   }

   /**
    * Calculates the bias for each degree of freedom by averaging the measurements in the window. If the window is not full, the bias is not calculated.
    */
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

   /**
    * It is the responsibility of external callers to increment the counter, this class only tracks whether the window has been filled or not.
    */
   public void incrementCounter()
   {
      counter++;
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
      return ZERO_MATRIX;
   }
}
