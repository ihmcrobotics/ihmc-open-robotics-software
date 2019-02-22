package us.ihmc.realtime.helperClasses;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Random;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class MultiplySmallMatricesALotTask extends BarrierSchedulerLoadTestTask
{
   private static final int MATRIX_ROW_DIMENSION = 50;
   private static final int MATRIX_COLUMN_DIMENSION = 50;

   private final Random random = new Random(1976L);
   private final BarrierSchedulerLoadTestData loadTestData = new BarrierSchedulerLoadTestData();
   private final TimingInformation timingInformation;
   private final DenseMatrix64F matrixA;
   private final DenseMatrix64F matrixB;
   private final DenseMatrix64F matrixC;

   private final ExecutionTimer executionTimer;

   public MultiplySmallMatricesALotTask(YoVariableRegistry parentRegistry, long schedulerPeriodNanoseconds, long divisor)
   {
      super(divisor);

      timingInformation = new TimingInformation("MultiplySmallMatricesALotTask", schedulerPeriodNanoseconds * divisor);
      matrixA = new DenseMatrix64F(MATRIX_ROW_DIMENSION, MATRIX_COLUMN_DIMENSION);
      matrixB = new DenseMatrix64F(MATRIX_ROW_DIMENSION, MATRIX_COLUMN_DIMENSION);
      matrixC = new DenseMatrix64F(MATRIX_ROW_DIMENSION, MATRIX_COLUMN_DIMENSION);

      executionTimer = new ExecutionTimer(getClass().getSimpleName() + "ExecutionTimer", parentRegistry);
   }

   @Override
   public BarrierSchedulerLoadTestData getLoadTestData()
   {
      return loadTestData;
   }

   @Override
   public void doTimingReporting()
   {
      System.out.format("MultiplySmallMatricesALotTask Jitter: avg = %.4f us, max = %.4f us%n", timingInformation.getFinalAvgJitterMicroseconds(),
                        timingInformation.getFinalMaxJitterMicroseconds());

      System.out.format("MultiplySmallMatricesALotTask Execution Time: avg = %.4f s, max = %.4f s%n", executionTimer.getAverageTime().getDoubleValue(),
                        executionTimer.getMaxTime().getDoubleValue());
   }

   /**
    * Initializes the internal state of the task.
    * <p>
    * Called once, immediately before the first {@link #execute()} call. This method is executed on
    * the task's thread.
    */
   @Override
   protected boolean initialize()
   {
      RandomMatrices.setRandom(matrixA, random);
      RandomMatrices.setRandom(matrixB, random);
      return true;
   }

   /**
    * Executes a single iteration of this task.
    * <p>
    * This method is executed on the task's thread.
    */
   @Override
   protected void execute()
   {
      BarrierSchedulerLoadTestHelper.doTimingStatistics(timingInformation);

      executionTimer.startMeasurement();

      CommonOps.add(matrixA, loadTestData.getSlowTaskFirstResult());
      CommonOps.add(matrixB, loadTestData.getSlowTaskSecondResult());

      int numberOfMultiplications;

      if (loadTestData.getFastTaskNumberOfOperations() == -1)
      {
         numberOfMultiplications = BarrierSchedulerLoadTestHelper.generateFastTaskNumberOfOperations(random); // always at least 1 iteration
      }
      else
      {
         numberOfMultiplications = loadTestData.getFastTaskNumberOfOperations();
      }

      BarrierSchedulerLoadTestHelper.doMatrixMultiplyOperations(matrixA, matrixB, matrixC, numberOfMultiplications);

      double elementSum = CommonOps.elementSum(matrixC);
      double determinant = CommonOps.det(matrixC);

      loadTestData.setFastTaskFirstResult(elementSum);
      loadTestData.setFastTaskSecondResult(determinant);
      loadTestData.setSlowTaskNumberOfOperations(BarrierSchedulerLoadTestHelper.generateSlowTaskNumberOfOperations(random));

      executionTimer.stopMeasurement();
   }
}
