package us.ihmc.realtime.barrierScheduler.benchmarks.helperClasses;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

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
   private final DMatrixRMaj matrixA;
   private final DMatrixRMaj matrixB;
   private final DMatrixRMaj matrixC;

   private final YoDouble actualDTMillis;
   boolean useNativeCommonOps;

   private final ExecutionTimer executionTimer;

   public MultiplySmallMatricesALotTask(YoDouble actualDTMillis, YoRegistry parentRegistry, long schedulerPeriodNanoseconds, long divisor, boolean useNativeCommonOps)
   {
      super(divisor);
      this.actualDTMillis = actualDTMillis;

      this.useNativeCommonOps = useNativeCommonOps;

      timingInformation = new TimingInformation("MultiplySmallMatricesALotTask", schedulerPeriodNanoseconds * divisor);
      matrixA = new DMatrixRMaj(MATRIX_ROW_DIMENSION, MATRIX_COLUMN_DIMENSION);
      matrixB = new DMatrixRMaj(MATRIX_ROW_DIMENSION, MATRIX_COLUMN_DIMENSION);
      matrixC = new DMatrixRMaj(MATRIX_ROW_DIMENSION, MATRIX_COLUMN_DIMENSION);

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
      RandomMatrices_DDRM.fillUniform(matrixA, random);
      RandomMatrices_DDRM.fillUniform(matrixB, random);
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
      RandomMatrices_DDRM.fillUniform(matrixA, random);
      RandomMatrices_DDRM.fillUniform(matrixB, random);

      executionTimer.startMeasurement();

      CommonOps_DDRM.add(matrixA, loadTestData.getSlowTaskFirstResult());
      CommonOps_DDRM.add(matrixB, loadTestData.getSlowTaskSecondResult());

      int numberOfMultiplications;

      if (loadTestData.getFastTaskNumberOfOperations() == -1)
      {
         numberOfMultiplications = BarrierSchedulerLoadTestHelper.generateFastTaskNumberOfOperations(random); // always at least 1 iteration
      }
      else
      {
         numberOfMultiplications = loadTestData.getFastTaskNumberOfOperations();
      }

      if (this.useNativeCommonOps)
      {
         BarrierSchedulerLoadTestHelper.doMatrixMultiplyOperationsNativeCommonOps(matrixA, matrixB, matrixC, numberOfMultiplications);
      }
      else
      {
         BarrierSchedulerLoadTestHelper.doMatrixMultiplyOperationsEJMLCommonOps(matrixA, matrixB, matrixC, numberOfMultiplications);
      }

      double elementSum = CommonOps_DDRM.elementSum(matrixC);
      //      double determinant = CommonOps_DDRM.det(matrixC);

      loadTestData.setFastTaskFirstResult(elementSum);
      loadTestData.setFastTaskSecondResult(elementSum * -1.0);
      loadTestData.setSlowTaskNumberOfOperations(BarrierSchedulerLoadTestHelper.generateSlowTaskNumberOfOperations(random));

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
