package us.ihmc.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class ExtendedKalmanFilterTest
{
   private static final double EPSILON = 1e-9;
   private static final int ITERATIONS = 1000;

   @Test
   public void testThrowsExceptionWithInvalidInputNoiseMatrices()
   {
      Random random = new Random(130);

      // Invalid Q
      for (int i = 0; i < ITERATIONS; ++i)
      {
         int stateSize = random.nextInt(10) + 1;
         int measurementSize = random.nextInt(10) + 1;

         DMatrixRMaj Q = RandomMatrices_DDRM.diagonal(stateSize, 1e-3, 1.0, random);
         CommonOps_DDRM.scale(-1.0, Q);  // Make Q negative definite
         DMatrixRMaj R = RandomMatrices_DDRM.diagonal(measurementSize, 1e-3, 1.0, random);
         DMatrixRMaj P0 = RandomMatrices_DDRM.diagonal(stateSize, 1.0, 10.0, random);  // We always keep the initial covariance matrix larger

         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(stateSize, stateSize, -1.0, 1.0, random);
         DMatrixRMaj C = RandomMatrices_DDRM.rectangle(measurementSize, stateSize, -1.0, 1.0, random);

         DMatrixRMaj x0 = RandomMatrices_DDRM.rectangle(stateSize, 1, -1.0, 1.0, random);

         try
         {
            TrivialExtendedKalmanFilter filter = new TrivialExtendedKalmanFilter(x0, A, C, P0, Q, R);
            fail("Should have thrown an exception.");
         }
         catch (IllegalArgumentException e)
         {
            // Good
         }
      }

      // Invalid R
      for (int j = 0; j < ITERATIONS; ++j)
      {
         int stateSize = random.nextInt(10) + 1;
         int measurementSize = random.nextInt(10) + 1;

         DMatrixRMaj Q = RandomMatrices_DDRM.diagonal(stateSize, 1e-3, 1.0, random);
         DMatrixRMaj R = RandomMatrices_DDRM.diagonal(measurementSize, 1e-3, 1.0, random);
         CommonOps_DDRM.scale(-1.0, R);  // Make R negative definite
         DMatrixRMaj P0 = RandomMatrices_DDRM.diagonal(stateSize, 1.0, 10.0, random);  // We always keep the initial covariance matrix larger

         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(stateSize, stateSize, -1.0, 1.0, random);
         DMatrixRMaj C = RandomMatrices_DDRM.rectangle(measurementSize, stateSize, -1.0, 1.0, random);

         DMatrixRMaj x0 = RandomMatrices_DDRM.rectangle(stateSize, 1, -1.0, 1.0, random);

         try
         {
            TrivialExtendedKalmanFilter filter = new TrivialExtendedKalmanFilter(x0, A, C, P0, Q, R);
            fail("Should have thrown an exception.");
         }
         catch (IllegalArgumentException e)
         {
            // Good
         }
      }

      // Invalid P0
      for (int k = 0; k < ITERATIONS; ++k)
      {
         int stateSize = random.nextInt(10) + 1;
         int measurementSize = random.nextInt(10) + 1;

         DMatrixRMaj Q = RandomMatrices_DDRM.diagonal(stateSize, 1e-3, 1.0, random);
         DMatrixRMaj R = RandomMatrices_DDRM.diagonal(measurementSize, 1e-3, 1.0, random);
         DMatrixRMaj P0 = RandomMatrices_DDRM.diagonal(stateSize, 1.0, 10.0, random);  // We always keep the initial covariance matrix larger
         CommonOps_DDRM.scale(-1.0, P0);  // Make P0 negative definite

         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(stateSize, stateSize, -1.0, 1.0, random);
         DMatrixRMaj C = RandomMatrices_DDRM.rectangle(measurementSize, stateSize, -1.0, 1.0, random);

         DMatrixRMaj x0 = RandomMatrices_DDRM.rectangle(stateSize, 1, -1.0, 1.0, random);

         try
         {
            TrivialExtendedKalmanFilter filter = new TrivialExtendedKalmanFilter(x0, A, C, P0, Q, R);
            fail("Should have thrown an exception.");
         }
         catch (IllegalArgumentException e)
         {
            // Good
         }
      }
   }

   @Test
   public void testPredictionStepOnRandomLinearSystems()
   {
      Random random = new Random(23);

      // Test with diagonal positive definite covariance matrices (no off-diagonal terms allowed)
      for (int i = 0; i < ITERATIONS; ++i)
      {
         // Randomly draw the size of the state and measurement vectors
         int stateSize = random.nextInt(10) + 1;
         int measurementSize = random.nextInt(stateSize) + 1;

         // Randomly draw the process noise, measurement noise, and initial covariance matrices to be diagonal (no-off diagonal terms)
         // and positive definite
         SimpleMatrix Q = SimpleMatrix.wrap(RandomMatrices_DDRM.diagonal(stateSize, 1e-3, 1.0, random));
         SimpleMatrix R = SimpleMatrix.wrap(RandomMatrices_DDRM.diagonal(measurementSize, 1e-3, 1.0, random));
         SimpleMatrix P0 = SimpleMatrix.wrap(RandomMatrices_DDRM.diagonal(stateSize,
                                                                          1.0,
                                                                          10.0,
                                                                          random));  // We always keep the initial covariance matrix larger

         // Randomly draw a linear dynamic system, we don't care about its stability or observability properties as we just want the maths to match up
         SimpleMatrix A = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(stateSize, stateSize, -1.0, 1.0, random));
         SimpleMatrix C = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(measurementSize, stateSize, -1.0, 1.0, random));

         // Randomly draw a previous state to use
         SimpleMatrix x0 = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(stateSize, 1, -1.0, 1.0, random));

         // Calculate the prediction step -- we don't care about garbage-free-ness here
         SimpleMatrix xPred = A.mult(x0);
         SimpleMatrix PPred = A.mult(P0).mult(A.transpose()).plus(Q);

         TrivialExtendedKalmanFilter filter = new TrivialExtendedKalmanFilter(x0.getDDRM(), A.getDDRM(), C.getDDRM(), P0.getDDRM(), Q.getDDRM(), R.getDDRM());
         filter.predictionStep();

         // Compare with filter
         SimpleMatrix xPredFilter = SimpleMatrix.wrap(filter.predictedState);
         SimpleMatrix PPredFilter = SimpleMatrix.wrap(filter.predictedCovariance);

         assertArrayEquals(xPred.getDDRM().getData(), xPredFilter.getDDRM().getData(), EPSILON);
         assertArrayEquals(PPred.getDDRM().getData(), PPredFilter.getDDRM().getData(), EPSILON);
      }

      // Test with symmetric positive definite covariance matrices (off-diagonal terms are allowed)
      for (int j = 0; j < ITERATIONS; ++j)
      {
         // Randomly draw the size of the state and measurement vectors
         int stateSize = random.nextInt(10) + 1;
         int measurementSize = random.nextInt(stateSize) + 1;

         // Randomly draw the process noise, measurement noise, and initial covariance matrices to be symmetric and positive definite
         SimpleMatrix Q = SimpleMatrix.wrap(RandomMatrices_DDRM.symmetricPosDef(stateSize, random));
         SimpleMatrix R = SimpleMatrix.wrap(RandomMatrices_DDRM.symmetricPosDef(measurementSize, random));
         SimpleMatrix P0 = SimpleMatrix.wrap(RandomMatrices_DDRM.symmetricPosDef(stateSize, random));  // We always keep the initial covariance matrix larger

         // Randomly draw a linear dynamic system, we don't care about its stability or observability properties as we just want the maths to match up
         SimpleMatrix A = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(stateSize, stateSize, -1.0, 1.0, random));
         SimpleMatrix C = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(measurementSize, stateSize, -1.0, 1.0, random));

         // Randomly draw a previous state to use
         SimpleMatrix x0 = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(stateSize, 1, -1.0, 1.0, random));

         // Calculate the prediction step -- we don't care about garbage-free-ness here
         SimpleMatrix xPred = A.mult(x0);
         SimpleMatrix PPred = A.mult(P0).mult(A.transpose()).plus(Q);

         TrivialExtendedKalmanFilter filter = new TrivialExtendedKalmanFilter(x0.getDDRM(), A.getDDRM(), C.getDDRM(), P0.getDDRM(), Q.getDDRM(), R.getDDRM());
         filter.predictionStep();

         // Compare with filter
         SimpleMatrix xPredFilter = SimpleMatrix.wrap(filter.predictedState);
         SimpleMatrix PPredFilter = SimpleMatrix.wrap(filter.predictedCovariance);

         assertArrayEquals(xPred.getDDRM().getData(), xPredFilter.getDDRM().getData(), EPSILON);
         assertArrayEquals(PPred.getDDRM().getData(), PPredFilter.getDDRM().getData(), EPSILON);
      }
   }

   @Test
   public void testUpdateStepOnRandomLinearSystems()
   {
      Random random = new Random(42);

      // Test with diagonal positive definite covariance matrices (no off-diagonal terms allowed)
      for (int i = 0; i < ITERATIONS; ++i)
      {
         // Randomly draw the size of the state and measurement vectors
         int stateSize = random.nextInt(10) + 1;
         int measurementSize = random.nextInt(stateSize) + 1;

         // Randomly draw the process noise, measurement noise, and initial covariance matrices to be diagonal (no-off diagonal terms)
         // and positive definite
         SimpleMatrix Q = SimpleMatrix.wrap(RandomMatrices_DDRM.diagonal(stateSize, 1e-3, 1.0, random));
         SimpleMatrix R = SimpleMatrix.wrap(RandomMatrices_DDRM.diagonal(measurementSize, 1e-3, 1.0, random));
         SimpleMatrix P0 = SimpleMatrix.wrap(RandomMatrices_DDRM.diagonal(stateSize,
                                                                          1.0,
                                                                          10.0,
                                                                          random));  // We always keep the initial covariance matrix larger

         // Randomly draw a linear dynamic system, we don't care about its stability or observability properties as we just want the maths to match up
         SimpleMatrix A = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(stateSize, stateSize, -1.0, 1.0, random));
         SimpleMatrix C = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(measurementSize, stateSize, -1.0, 1.0, random));

         // Randomly draw a previous state to use
         SimpleMatrix x0 = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(stateSize, 1, -1.0, 1.0, random));

         // We assume the filter can be trusted for the prediction step -- NOTE: if the test for the prediction step is failing, this cannot be trusted!
         TrivialExtendedKalmanFilter filter = new TrivialExtendedKalmanFilter(x0.getDDRM(), A.getDDRM(), C.getDDRM(), P0.getDDRM(), Q.getDDRM(), R.getDDRM());
         filter.predictionStep();
         SimpleMatrix xPred = SimpleMatrix.wrap(filter.predictedState);
         SimpleMatrix PPred = SimpleMatrix.wrap(filter.predictedCovariance);

         // Randomly draw an actual measurement to compute a residual off of
         SimpleMatrix z = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(measurementSize, 1, -1.0, 1.0, random));

         // Calculate the update step -- we don't care about garbage-free-ness here
         SimpleMatrix y = z.minus(C.mult(xPred));
         SimpleMatrix S = C.mult(PPred).mult(C.transpose()).plus(R);
         SimpleMatrix K = PPred.mult(C.transpose()).mult(S.invert());
         SimpleMatrix xUpdate = xPred.plus(K.mult(y));
         SimpleMatrix PUpdate = PPred.minus(K.mult(C).mult(PPred));

         filter.queryInnovationGate();
         filter.calculateMeasurementResidual(z.getDDRM());
         filter.updateStep();

         // Compare with filter
         SimpleMatrix xUpdateFilter = SimpleMatrix.wrap(filter.updatedState);
         SimpleMatrix PUpdateFilter = SimpleMatrix.wrap(filter.updatedCovariance);

         assertArrayEquals(xUpdate.getDDRM().getData(), xUpdateFilter.getDDRM().getData(), EPSILON);
         assertArrayEquals(PUpdate.getDDRM().getData(), PUpdateFilter.getDDRM().getData(), EPSILON);
      }

      // Test with symmetric positive definite covariance matrices (off-diagonal terms are allowed)
      for (int j = 0; j < ITERATIONS; ++j)
      {
         // Randomly draw the size of the state and measurement vectors
         int stateSize = random.nextInt(10) + 1;
         int measurementSize = random.nextInt(stateSize) + 1;

         // Randomly draw the process noise, measurement noise, and initial covariance matrices to be diagonal (no-off diagonal terms)
         // and positive definite
         SimpleMatrix Q = SimpleMatrix.wrap(RandomMatrices_DDRM.symmetricPosDef(stateSize, random));
         SimpleMatrix R = SimpleMatrix.wrap(RandomMatrices_DDRM.symmetricPosDef(measurementSize, random));
         SimpleMatrix P0 = SimpleMatrix.wrap(RandomMatrices_DDRM.symmetricPosDef(stateSize, random));  // We always keep the initial covariance matrix larger

         // Randomly draw a linear dynamic system, we don't care about its stability or observability properties as we just want the maths to match up
         SimpleMatrix A = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(stateSize, stateSize, -1.0, 1.0, random));
         SimpleMatrix C = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(measurementSize, stateSize, -1.0, 1.0, random));

         // Randomly draw a previous state to use
         SimpleMatrix x0 = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(stateSize, 1, -1.0, 1.0, random));

         // We assume the filter can be trusted for the prediction step -- NOTE: if the test for the prediction step is failing, this cannot be trusted!
         TrivialExtendedKalmanFilter filter = new TrivialExtendedKalmanFilter(x0.getDDRM(), A.getDDRM(), C.getDDRM(), P0.getDDRM(), Q.getDDRM(), R.getDDRM());
         filter.predictionStep();
         SimpleMatrix xPred = SimpleMatrix.wrap(filter.predictedState);
         SimpleMatrix PPred = SimpleMatrix.wrap(filter.predictedCovariance);

         // Randomly draw an actual measurement to compute a residual off of
         SimpleMatrix z = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(measurementSize, 1, -1.0, 1.0, random));

         // Calculate the update step -- we don't care about garbage-free-ness here
         SimpleMatrix y = z.minus(C.mult(xPred));
         SimpleMatrix S = C.mult(PPred).mult(C.transpose()).plus(R);
         SimpleMatrix K = PPred.mult(C.transpose()).mult(S.invert());
         SimpleMatrix xUpdate = xPred.plus(K.mult(y));
         SimpleMatrix PUpdate = PPred.minus(K.mult(C).mult(PPred));

         filter.queryInnovationGate();
         filter.calculateMeasurementResidual(z.getDDRM());
         filter.updateStep();

         // Compare with filter
         SimpleMatrix xUpdateFilter = SimpleMatrix.wrap(filter.updatedState);
         SimpleMatrix PUpdateFilter = SimpleMatrix.wrap(filter.updatedCovariance);

         assertArrayEquals(xUpdate.getDDRM().getData(), xUpdateFilter.getDDRM().getData(), EPSILON);
         assertArrayEquals(PUpdate.getDDRM().getData(), PUpdateFilter.getDDRM().getData(), EPSILON);
      }
   }

   private static class TrivialExtendedKalmanFilter extends ExtendedKalmanFilter
   {
      private final DMatrixRMaj A;
      private final DMatrixRMaj C;

      public TrivialExtendedKalmanFilter(DMatrixRMaj x0, DMatrixRMaj A, DMatrixRMaj C, DMatrixRMaj P0, DMatrixRMaj Q, DMatrixRMaj R)
      {
         super(x0, P0, Q, R);
         this.A = new DMatrixRMaj(A);
         this.C = new DMatrixRMaj(C);
      }

      @Override
      public DMatrixRMaj linearizeProcessModel(DMatrixRMaj previousState)
      {
         return A;
      }

      @Override
      public DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedState)
      {
         return C;
      }

      @Override
      public DMatrixRMaj processModel(DMatrixRMaj state)
      {
         DMatrixRMaj result = new DMatrixRMaj(A.numRows, 1);
         CommonOps_DDRM.mult(A, state, result);
         return result;
      }

      @Override
      public DMatrixRMaj measurementModel(DMatrixRMaj state)
      {
         DMatrixRMaj result = new DMatrixRMaj(C.numRows, 1);
         CommonOps_DDRM.mult(C, state, result);
         return result;
      }
   }
}