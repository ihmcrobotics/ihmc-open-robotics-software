package us.ihmc.realtime.barrierScheduler.benchmarks.helperClasses;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Random;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class MultiplyBigMatricesALotTask extends BarrierSchedulerLoadTestTask
{
   private static final int MATRIX_ROW_DIMENSION = 100;
   private static final int MATRIX_COLUMN_DIMENSION = 100;

   private final Random random = new Random(1976L);
   private final BarrierSchedulerLoadTestData loadTestData = new BarrierSchedulerLoadTestData();
   private final TimingInformation timingInformation;
   private final DenseMatrix64F matrixA;
   private final DenseMatrix64F matrixB;
   private final DenseMatrix64F matrixC;

   private final YoDouble actualDTMillis;
   private final boolean useNativeCommonOps;

   private final ExecutionTimer executionTimer;

   public MultiplyBigMatricesALotTask(YoDouble actualDTMillis, YoVariableRegistry parentRegistry, long schedulerPeriodNanoseconds, long divisor, boolean useNativeCommonOps)
   {
      super(divisor);
      this.actualDTMillis = actualDTMillis;

      this.useNativeCommonOps = useNativeCommonOps;

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
      BarrierSchedulerLoadTestHelper.printTimingStatisticsCSV(getClass().getSimpleName(), executionTimer);
      BarrierSchedulerLoadTestHelper.printTimingStatisticsCSV(getClass().getSimpleName(), timingInformation);
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
      timingInformation.initialize(System.nanoTime(), actualDTMillis);
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
      RandomMatrices.setRandom(matrixA, random);
      RandomMatrices.setRandom(matrixB, random);

      executionTimer.startMeasurement();

      CommonOps.add(matrixA, loadTestData.getFastTaskFirstResult());
      CommonOps.add(matrixB, loadTestData.getFastTaskSecondResult());

      int numberOfMultiplications;

      if (loadTestData.getSlowTaskNumberOfOperations() == -1)
      {
         numberOfMultiplications = BarrierSchedulerLoadTestHelper.generateSlowTaskNumberOfOperations(random);
      }
      else
      {
         numberOfMultiplications = loadTestData.getSlowTaskNumberOfOperations();
      }

      if (this.useNativeCommonOps)
      {
         BarrierSchedulerLoadTestHelper.doMatrixMultiplyOperationsNativeCommonOps(matrixA, matrixB, matrixC, numberOfMultiplications);
      }
      else
      {
         BarrierSchedulerLoadTestHelper.doMatrixMultiplyOperationsEJMLCommonOps(matrixA, matrixB, matrixC, numberOfMultiplications);
      }

      double elementSum = CommonOps.elementSum(matrixC);
//      double determinant = CommonOps.det(matrixC);

      loadTestData.setSlowTaskFirstResult(elementSum);
      loadTestData.setSlowTaskSecondResult(elementSum * -1.0);
      loadTestData.setFastTaskNumberOfOperations(BarrierSchedulerLoadTestHelper.generateFastTaskNumberOfOperations(random));

      executionTimer.stopMeasurement();
      BarrierSchedulerLoadTestHelper.doTimingStatistics(timingInformation);
   }

   /**
    * Perform any cleanup before shutting down. Called the next tick after a
    * shutdown request from the barrier scheduler.
    */
   @Override
   protected void cleanup()
   {
   }
}
