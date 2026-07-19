package us.ihmc.commonWalkingControlModules.parameterEstimation;

import java.util.Arrays;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.yoVariables.math.YoMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * This class is used to calculate a constant approximation to the offset seen in {@link  InertialParameterManager} estimation residuals on the real robot.
 * <p>
 * The filters used inside {@link InertialParameterManager} assume that the residuals in the estimation process (see {@code residual} in that class) are
 * zero-mean. This is not the case in practice due to various inertial parameter non-idealities. The filter is used to calculate a constant
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

   /**
    * Per-row mask controlling which degrees of freedom {@link #calculateBias()} actually updates. Defaults to all
    * rows. A masked tare (for example the pose-independent floating-base-linear tare) narrows this so that only the
    * chosen rows move on the next window fill, leaving the rest of the bias exactly as it was.
    */
   private final boolean[] rowActive;

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

      rowActive = new boolean[nDoFs];
      Arrays.fill(rowActive, true);
   }

   /** Restores the default: {@link #calculateBias()} updates every degree of freedom. */
   public void setAllRowsActive()
   {
      Arrays.fill(rowActive, true);
   }

   /**
    * Restricts {@link #calculateBias()} to the given measurement rows: on the next window fill only those rows of the
    * bias are updated, and every other row keeps its current value. Used for a masked tare (for example taring only the
    * floating base's linear force rows). Reset with {@link #setAllRowsActive()}.
    */
   public void setActiveRows(int... rows)
   {
      Arrays.fill(rowActive, false);
      for (int row : rows)
         rowActive[row] = true;
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
    * ACCUMULATES the bias: averages the residuals in the window and ADDS that average to the existing bias.
    * <p>
    * The addition is deliberate. The residuals being averaged are produced by a measurement model that already
    * has the current bias subtracted, so their mean is the bias that is still MISSING, not the total. Overwriting
    * would therefore throw the existing bias away and leave only the remainder -- correct only when starting from
    * zero, and wrong on every re-tare.
    * </p>
    */
   public void calculateBias()
   {
      CommonOps_DDRM.sumRows(measurements, biasContainer);
      CommonOps_DDRM.scale(1.0 / windowSize, biasContainer);

      for (int i = 0; i < biasContainer.getNumRows(); i++)
         if (rowActive[i])
            bias.set(i, 0, bias.get(i, 0) + biasContainer.get(i, 0));
   }

   /** Clears the measurement window so a fresh set of residuals can be collected. The bias itself is KEPT. */
   public void rearm()
   {
      counter = 0;
      measurements.zero();
   }

   /** Clears the measurement window AND zeroes the bias, discarding everything previously identified. */
   public void reset()
   {
      rearm();
      biasContainer.zero();
      bias.zero();
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