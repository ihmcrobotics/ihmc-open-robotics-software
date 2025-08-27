package us.ihmc.parameterEstimation.examples;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.parameterEstimation.ExtendedKalmanFilter;

public class ExampleConstantVelocity2DKalmanFilter extends ExtendedKalmanFilter
{
   static final int stateSize = 4;
   static final int measurementSize = 2;

   // Start at (x,y) = (0,0) with no velocity
   static final DMatrixRMaj x0 = new DMatrixRMaj(new double[] {0.0, 0.0, 0.0, 0.0});

   // Position is barely affected by noise, velocity is affected by noise
   static final DMatrixRMaj Q = new DMatrixRMaj(new double[][] {{1e-6, 0.0, 0.0, 0.0},
                                                                        {0.0, 1e-6, 0.0, 0.0},
                                                                        {0.0, 0.0, 1e-6, 0.0},
                                                                        {0.0, 0.0, 0.0, 1e-6}});

   // Both x and y position measurements are corrupted by noise
   static final DMatrixRMaj R = new DMatrixRMaj(new double[][] {{1, 0.0}, {0.0, 1}});

   // Ignorant initial guess on P0, we assume we're more certain about positions than velocities
   static final DMatrixRMaj P0 = new DMatrixRMaj(new double[][] {{0.1, 0.0, 0.0, 0.0},
                                                                         {0.0, 1.0, 0.0, 0.0},
                                                                         {0.0, 0.0, 0.1, 0.0},
                                                                         {0.0, 0.0, 0.0, 1.0}});

   static final double dt = 0.01;

   // Constant velocity model of a 2D planar system
   static final DMatrixRMaj A = new DMatrixRMaj(new double[][] {{1.0, dt, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, dt}, {0.0, 0.0, 0.0, 1.0}});

   // We only measure positions, not velocities
   static final DMatrixRMaj C = new DMatrixRMaj(new double[][] {{1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}});

   public ExampleConstantVelocity2DKalmanFilter()
   {
      super(x0, P0, Q, R);
   }

   // In the case of a linear process model, the linearization is just the A matrix of the process model
   @Override
   public DMatrixRMaj linearizeProcessModel(DMatrixRMaj previousState)
   {
      return A;
   }

   // In the case of a linear measurement model, the linearization is just the C matrix of the measurement model
   @Override
   public DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedState)
   {
      return C;
   }

   // y = Ax
   @Override
   public DMatrixRMaj processModel(DMatrixRMaj state)
   {
      DMatrixRMaj nextState = new DMatrixRMaj(stateSize, 1);
      CommonOps_DDRM.mult(A, state, nextState);
      return nextState;
   }

   // y = Cx
   @Override
   public DMatrixRMaj measurementModel(DMatrixRMaj state)
   {
      DMatrixRMaj measurement = new DMatrixRMaj(measurementSize, 1);
      CommonOps_DDRM.mult(C, state, measurement);
      return measurement;
   }
}
