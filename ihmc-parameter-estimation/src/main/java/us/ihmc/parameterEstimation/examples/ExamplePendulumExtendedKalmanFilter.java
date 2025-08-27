package us.ihmc.parameterEstimation.examples;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.parameterEstimation.ExtendedKalmanFilter;

public class ExamplePendulumExtendedKalmanFilter extends ExtendedKalmanFilter
{
   static final int stateSize = 2;
   static final int measurementSize = 1;

   // Picking an arbitrary nonzero initial condition since 0 is a singularity for the measurement
   private static final DMatrixRMaj x0 = new DMatrixRMaj(new double[] {0.1, 0.1});

   // Process noise
   private static final DMatrixRMaj Q = new DMatrixRMaj(new double[][] {{1e-6, 0.0}, {0.0, 1e-6}});

   // Measurement noise
   private static final DMatrixRMaj R = new DMatrixRMaj(new double[][] {{1e-3}});

   // Ignorant initial guess on P0, we assume we're more certain about positions than velocities
   private static final DMatrixRMaj P0 = new DMatrixRMaj(new double[][] {{0.1, 0.0}, {0.0, 1.0}});

   // Linearized process model dynamics ( Jacobian of f(x) where x[k+1] = f(x[k]) )
   final DMatrixRMaj F = new DMatrixRMaj(stateSize, stateSize);

   // Linearized measurement model dynamics (Jacobian of h(x) where y = h(x))
   final DMatrixRMaj H = new DMatrixRMaj(measurementSize, stateSize);

   private double I = 0.006; // inertia [kg m^2]
   private double m = 0.2;   // mass [kg]
   private double b = 0.02;  // damping coefficient (not pendulum) [N/s]
   private double g = 9.8;   // acceleration due to gravity [m/s^2]
   private double l = 0.3;   // length to center of mass [m]
   private double dt = 0.01; // integration dt [s]

   public ExamplePendulumExtendedKalmanFilter()
   {
      super(x0, P0, Q, R);
   }

   public void setModelProperties(double m, double I, double l, double b, double g, double dt)
   {
      this.m = m;
      this.I = I;
      this.l = l;
      this.b = b;
      this.g = g;
      this.dt = dt;
   }

   public DMatrixRMaj getQ()
   {
      return Q;
   }

   public DMatrixRMaj getR()
   {
      return R;
   }

   public DMatrixRMaj getF()
   {
      return F;
   }

   public DMatrixRMaj getH()
   {
      return H;
   }

   // Linearize f(x) to obtain it's Jacobian matrix, F(x)
   @Override
   public DMatrixRMaj linearizeProcessModel(DMatrixRMaj previousState)
   {
      double q = previousState.get(0, 0);

      F.set(0, 0, 1);
      F.set(0, 1, dt);
      F.set(1, 0, -(dt * m * g * l * Math.cos(q)) / I);
      F.set(1, 1, 1 - dt * b / I);

      return F;
   }

   // Linearize h(x) to obtain it's Jacobian matrix, H(x)
   @Override
   public DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedState)
   {
      double q = predictedState.get(0, 0);
      double qDot = predictedState.get(1, 0);

      H.set(0, 0, m * g * l * Math.sin(q));
      H.set(0, 1, I * qDot);

      return H;
   }

   // x[k+1] = f(x[k])
   @Override
   public DMatrixRMaj processModel(DMatrixRMaj state)
   {
      return getNextState(state);
   }

   // y = h(x)
   @Override
   public DMatrixRMaj measurementModel(DMatrixRMaj state)
   {
      return getMeasurement(state);
   }

   DMatrixRMaj getNextState(DMatrixRMaj state)
   {
      DMatrixRMaj nextState = new DMatrixRMaj(state.numRows, 1);
      double q = state.get(0, 0);
      double qDot = state.get(1, 0);

      // Discretized with Forward Euler
      nextState.set(0, 0, q + dt * qDot);
      nextState.set(1, 0, qDot - dt * (b * qDot + m * g * l * Math.sin(q)) / I);

      return nextState;
   }

   DMatrixRMaj getMeasurement(DMatrixRMaj state)
   {
      DMatrixRMaj measurement = new DMatrixRMaj(ExamplePendulumExtendedKalmanFilter.measurementSize, 1);
      double q = state.get(0, 0);
      double qDot = state.get(1, 0);

      // Measurement is the total energy in the system, which will be dissipated with nonzero damping.
      measurement.set(0, 0, 0.5 * I * qDot * qDot + m * g * l * (1 - Math.cos(q)));

      return measurement;
   }
}
