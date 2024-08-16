package us.ihmc.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.CovarianceRandomDraw_DDRM;

import java.util.Random;

public class ExtendedKalmanFilterTestTools
{
   /**
    * Utility class that can be used to simulate an autonomous (no control input) linear system that is corrupted by process noise and measurement noise.
    */
   public static class LinearSystem
   {
      private final DMatrixRMaj state;
      private final DMatrixRMaj A;
      private final DMatrixRMaj C;

      private CovarianceRandomDraw_DDRM processNoiseGenerator;
      private CovarianceRandomDraw_DDRM measurementNoiseGenerator;

      private final boolean processHasNoise;
      private final boolean measurementHasNoise;

      public LinearSystem(DMatrixRMaj x0, DMatrixRMaj A, DMatrixRMaj C)
      {
         this(x0, A, null, C, null, null);
      }

      public LinearSystem(DMatrixRMaj x0, DMatrixRMaj A, DMatrixRMaj Q, DMatrixRMaj C, DMatrixRMaj R, Random random)
      {
         processHasNoise = (Q != null) && (CommonOps_DDRM.trace(Q) != 0.0);
         measurementHasNoise = (R != null) && (CommonOps_DDRM.trace(R) != 0.0);

         checkDimensions(x0, A, Q, C, R);

         state = new DMatrixRMaj(x0.numRows, 1);
         state.set(x0);

         this.A = new DMatrixRMaj(A.numRows, A.numCols);
         this.A.set(A);

         this.C = new DMatrixRMaj(C.numRows, C.numCols);
         this.C.set(C);

         createNoiseGenerators(Q, R, random);
      }

      private void createNoiseGenerators(DMatrixRMaj Q, DMatrixRMaj R, Random random)
      {
         if (processHasNoise)
            processNoiseGenerator = new CovarianceRandomDraw_DDRM(random, Q);

         if (measurementHasNoise)
            measurementNoiseGenerator = new CovarianceRandomDraw_DDRM(random, R);
      }

      private void checkDimensions(DMatrixRMaj x0, DMatrixRMaj A, DMatrixRMaj Q, DMatrixRMaj C, DMatrixRMaj R)
      {
         // Checks on dimensions
         if (x0.numCols != 1)
            throw new RuntimeException("x0 must be a column vector.");

         if (A.numRows != A.numCols)
            throw new RuntimeException("A must be a square matrix.");

         if (x0.numRows != A.numRows || x0.numRows != C.numCols)
            throw new RuntimeException("Dimensions of x0, A, and C are not consistent.");

         if (processHasNoise)
         {
            if (Q.numRows != Q.numCols || R.numRows != R.numCols)
               throw new RuntimeException("Q or R is not a square matrix.");

            if (x0.numRows != Q.numRows || C.numRows != R.numRows)
               throw new RuntimeException("Dimensions of x0, Q, C, and R are not consistent.");
         }
      }

      public void updateState(DMatrixRMaj state)
      {
         this.state.set(state);
      }

      public DMatrixRMaj calculateNextState()
      {
         // x_{k+1} = A * x_k
         DMatrixRMaj nextState = new DMatrixRMaj(state.numRows, 1);
         CommonOps_DDRM.mult(A, state, nextState);

         // corrupt with process noise if applicable
         if (processHasNoise)
            processNoiseGenerator.next(nextState);

         return nextState;
      }

      public DMatrixRMaj calculateMeasurement()
      {
         // y_k = C * x_k
         DMatrixRMaj measurement = new DMatrixRMaj(state.numRows, 1);
         CommonOps_DDRM.mult(C, state, measurement);

         // corrupt with measurement noise if applicable
         if (measurementHasNoise)
            measurementNoiseGenerator.next(measurement);

         return measurement;
      }
   }

   /**
    * Utility class that can be used to simulate an autonomous (no control input) nonlinear system that is corrupted by process noise and measurement noise.
    */
   public static class NonlinearSystem
   {
      private final DMatrixRMaj state;

      private CovarianceRandomDraw_DDRM processNoiseGenerator;
      private CovarianceRandomDraw_DDRM measurementNoiseGenerator;

      private final boolean processHasNoise;
      private final boolean measurementHasNoise;

      public NonlinearSystem(DMatrixRMaj x0, DMatrixRMaj F, DMatrixRMaj H)
      {
         this(x0, F, null, H, null, null);
      }

      public NonlinearSystem(DMatrixRMaj x0, DMatrixRMaj F, DMatrixRMaj Q, DMatrixRMaj H, DMatrixRMaj R, Random random)
      {
         processHasNoise = (Q != null) && (CommonOps_DDRM.trace(Q) != 0.0);
         measurementHasNoise = (R != null) && (CommonOps_DDRM.trace(R) != 0.0);

         checkDimensions(x0, F, Q, H, R);

         state = new DMatrixRMaj(x0.numRows, 1);
         state.set(x0);

         DMatrixRMaj f = new DMatrixRMaj(F.numRows, F.numCols);
         f.set(F);

         DMatrixRMaj h = new DMatrixRMaj(H.numRows, H.numCols);
         h.set(H);

         createNoiseGenerators(Q, R, random);
      }

      private void createNoiseGenerators(DMatrixRMaj Q, DMatrixRMaj R, Random random)
      {
         if (processHasNoise)
            processNoiseGenerator = new CovarianceRandomDraw_DDRM(random, Q);

         if (measurementHasNoise)
            measurementNoiseGenerator = new CovarianceRandomDraw_DDRM(random, R);
      }

      private void checkDimensions(DMatrixRMaj x0, DMatrixRMaj F, DMatrixRMaj Q, DMatrixRMaj H, DMatrixRMaj R)
      {
         // Checks on dimensions
         if (x0.numCols != 1)
            throw new RuntimeException("x0 must be a column vector.");

         if (F.numRows != F.numCols)
            throw new RuntimeException("A must be a square matrix.");

         if (x0.numRows != F.numRows || x0.numRows != H.numCols)
            throw new RuntimeException("Dimensions of x0, A, and C are not consistent.");

         if (processHasNoise)
         {
            if (Q.numRows != Q.numCols || R.numRows != R.numCols)
               throw new RuntimeException("Q or R is not a square matrix.");

            if (x0.numRows != Q.numRows || H.numRows != R.numRows)
               throw new RuntimeException("Dimensions of x0, Q, C, and R are not consistent.");
         }
      }

      public void updateState(DMatrixRMaj state)
      {
         this.state.set(state);
      }

      public DMatrixRMaj calculateNextState()
      {
         // x_{k+1} = f(x_k)
         DMatrixRMaj nextState = calculateSystemDynamics(state);

         // corrupt with process noise if applicable
         if (processHasNoise)
            processNoiseGenerator.next(nextState);

         return nextState;
      }

      /**
       * The system dynamics must be defined by overriding this method.
       */
      public DMatrixRMaj calculateSystemDynamics(DMatrixRMaj state)
      {
         return null;
      }

      public DMatrixRMaj calculateMeasurement()
      {
         // y_k = h(x_k)
         DMatrixRMaj measurement = calculateMeasurementDynamics(state);

         // corrupt with measurement noise if applicable
         if (measurementHasNoise)
            measurementNoiseGenerator.next(measurement);

         return measurement;
      }

      /**
       * The measurement dynamics must be defined by overriding this method.
       */
      public DMatrixRMaj calculateMeasurementDynamics(DMatrixRMaj state)
      {
         return null;
      }
   }
}
