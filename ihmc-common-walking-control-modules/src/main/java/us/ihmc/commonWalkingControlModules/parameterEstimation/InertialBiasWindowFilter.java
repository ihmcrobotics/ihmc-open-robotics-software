package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.robotics.MatrixMissingTools;
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
public class InertialBiasWindowFilter
{
   /** Keeps track of the progress through the measurement window, is updated after each new set of measurements. */
   private int counter;
   /** The number of measurements to be used to calculate the bias. */
   private final int windowSize;
   /**
    * Where the measurements used to calculate the bias ae stored, the first index is the considered degree of freedom, the second index is the tick in the
    * measurement window.
    */
   private final DMatrixRMaj measurements;
   /** Container for the bias to allow EJML operations to be used before passing to YoMatrix. */
   private final DMatrixRMaj biasContainer;
   /** Where the bias for each degree of freedom is stored after it is calculated. */
   private final YoMatrix bias;

   /** If we choose to temporarily exclude the bias, but not overwrite its value, we need an appropriately sized zero matrix as a placeholder. */
   private final DMatrixRMaj ZERO_MATRIX;

   public InertialBiasWindowFilter(int nDoFs, int windowSize, String[] rowNames, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      counter = 0;
      this.windowSize = windowSize;
      measurements = new DMatrixRMaj(nDoFs, windowSize);
      biasContainer = new DMatrixRMaj(nDoFs, 1);
      bias = new YoMatrix("bias_", nDoFs, 1, rowNames, null, registry);

      ZERO_MATRIX = new DMatrixRMaj(nDoFs, 1);
   }

   /**
    * Updates the measurements with the new measurement, if the window is not full. If the window is full, the bias is calculated.
    *
    * @param measurement the new measurement to be stored.
    * @return whether the window is full.
    */
   public boolean update(DMatrix measurement)
   {
      boolean isWindowFilled = counter == windowSize;

      if (isWindowFilled)
      {
         calculateBias();
      }
      else
      {
         MatrixMissingTools.setMatrixColumn(measurements, counter, measurement, 0);
         counter++;
      }

      return isWindowFilled;
   }

   /**
    * Calculates the bias for each degree of freedom by averaging the measurements in the window.
    */
   public void calculateBias()
   {
      CommonOps_DDRM.sumRows(measurements, biasContainer);
      CommonOps_DDRM.scale(1.0 / windowSize, biasContainer);
      bias.set(biasContainer);
   }

   public void reset()
   {
      counter = 0;
      measurements.zero();
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