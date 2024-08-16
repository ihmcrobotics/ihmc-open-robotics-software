package us.ihmc.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.knowm.xchart.QuickChart;
import org.knowm.xchart.SwingWrapper;
import org.knowm.xchart.XYChart;
import us.ihmc.parameterEstimation.ExtendedKalmanFilterTestTools.NonlinearSystem;

import javax.swing.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

/**
 * Demo of an EKF being used with a nonlinear process model and measurement model.
 * <p>
 * The system under consideration is a 2D swinging pendulum. The state is [q, qDot] and the measurement is total energy in the system:
 * [ 0.5 I qDot^2 + m g l (1 - cos(q)) ].
 * </p>
 */
public class PendulumExtendedKalmanFilterDemo
{
   private static final double I = 0.006; // inertia [kg m^2]
   private static final double m = 0.2;   // mass [kg]
   private static final double b = 0.02;  // damping coefficient (not pendulum) [N/s]
   private static final double g = 9.8;   // acceleration due to gravity [m/s^2]
   private static final double l = 0.3;   // length to center of mass [m]

   private static final double dt = 0.01;

   public static class PendulumExtendedKalmanFilter extends ExtendedKalmanFilter
   {
      private static final int stateSize = 2;
      private static final int measurementSize = 1;

      // Picking an arbitrary nonzero initial condition since 0 is a singularity for the measurement
      private static final DMatrixRMaj x0 = new DMatrixRMaj(new double[] {0.1, 0.1});

      // Process noise
      private static final DMatrixRMaj Q = new DMatrixRMaj(new double[][] {{1e-6, 0.0}, {0.0, 1e-6}});

      // Measurement noise
      private static final DMatrixRMaj R = new DMatrixRMaj(new double[][] {{1e-3}});

      // Ignorant initial guess on P0, we assume we're more certain about positions than velocities
      private static final DMatrixRMaj P0 = new DMatrixRMaj(new double[][] {{0.1, 0.0}, {0.0, 1.0}});

      // Linearized process model dynamics ( Jacobian of f(x) where x[k+1] = f(x[k]) )
      private static final DMatrixRMaj F = new DMatrixRMaj(stateSize, stateSize);

      // Linearized measurement model dynamics (Jacobian of h(x) where y = h(x))
      private static final DMatrixRMaj H = new DMatrixRMaj(measurementSize, stateSize);

      public PendulumExtendedKalmanFilter()
      {
         super(x0, P0, Q, R);
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
   }

   private static class PendulumSystem extends NonlinearSystem
   {
      public PendulumSystem(DMatrixRMaj x0, DMatrixRMaj F, DMatrixRMaj H)
      {
         super(x0, F, H);
      }

      public PendulumSystem(DMatrixRMaj x0, DMatrixRMaj F, DMatrixRMaj Q, DMatrixRMaj H, DMatrixRMaj R, Random random)
      {
         super(x0, F, Q, H, R, random);
      }

      @Override
      public DMatrixRMaj calculateSystemDynamics(DMatrixRMaj state)
      {
         return getNextState(state);
      }

      @Override
      public DMatrixRMaj calculateMeasurementDynamics(DMatrixRMaj state)
      {
         return getMeasurement(state);
      }
   }

   private static DMatrixRMaj getNextState(DMatrixRMaj state)
   {
      DMatrixRMaj nextState = new DMatrixRMaj(state.numRows, 1);
      double q = state.get(0, 0);
      double qDot = state.get(1, 0);

      // Discretized with Forward Euler
      nextState.set(0, 0, q + dt * qDot);
      nextState.set(1, 0, qDot - dt * (b * qDot + m * g * l * Math.sin(q)) / I);

      return nextState;
   }

   private static DMatrixRMaj getMeasurement(DMatrixRMaj state)
   {
      DMatrixRMaj measurement = new DMatrixRMaj(PendulumExtendedKalmanFilter.measurementSize, 1);
      double q = state.get(0, 0);
      double qDot = state.get(1, 0);

      // Measurement is the total energy in the system, which will be dissipated with nonzero damping.
      measurement.set(0, 0, 0.5 * I * qDot * qDot + m * g * l * (1 - Math.cos(q)));

      return measurement;
   }

   private static final int ITERATIONS = 500;

   public static void main(String[] args)
   {
      Random random = new Random(45);

      // We'll set things up such that the real system is always less noisy than the filter. Said the other way, our filter design
      // will assume that the system is more noisy than it actually is -- a good thing.
      DMatrixRMaj Q = new DMatrixRMaj(PendulumExtendedKalmanFilter.Q);
      CommonOps_DDRM.scale(0.1, Q);
      // We'll set things up such that the real system is always less noisy than the filter. Said the other way, our filter design
      // will assume that the system is more noisy than it actually is -- a good thing.
      DMatrixRMaj R = new DMatrixRMaj(PendulumExtendedKalmanFilter.R);
      CommonOps_DDRM.scale(0.01, R);

      // In general, initial state of system is different from that of filter.
      DMatrixRMaj initialSystemState = new DMatrixRMaj(new double[] {Math.PI / 4, 0.0});

      // "Real-life" system
      //      NonlinearSystem system = new ConstantVelocity2DNonlinearMeasurementSystem(initialSystemState,
      //                                                                                PendulumExtendedKalmanFilter.F,
      //                                                                                PendulumExtendedKalmanFilter.H);
      NonlinearSystem system = new PendulumSystem(initialSystemState, PendulumExtendedKalmanFilter.F, Q, PendulumExtendedKalmanFilter.H, R, random);

      DMatrixRMaj state = new DMatrixRMaj(PendulumExtendedKalmanFilter.stateSize, 1);
      DMatrixRMaj measurement = new DMatrixRMaj(PendulumExtendedKalmanFilter.measurementSize, 1);

      // Filter
      PendulumExtendedKalmanFilter kalmanFilter = new PendulumExtendedKalmanFilter();
      DMatrixRMaj estimatedState = new DMatrixRMaj(PendulumExtendedKalmanFilter.stateSize, 1);

      // Arrays for recording data that we'll plot
      double[] timestamps = new double[ITERATIONS];
      ArrayList<DMatrixRMaj> trueStates = new ArrayList<>();
      ArrayList<DMatrixRMaj> measurements = new ArrayList<>();
      ArrayList<DMatrixRMaj> estimatedStates = new ArrayList<>();

      // Main loop, simulate system and apply filter
      for (int i = 0; i < ITERATIONS; ++i)
      {
         state.set(system.calculateNextState());
         measurement.set(system.calculateMeasurement());

         estimatedState.set(kalmanFilter.calculateEstimate(measurement));

         timestamps[i] = i * dt;
         trueStates.add(new DMatrixRMaj(state));
         measurements.add(new DMatrixRMaj(measurement));
         estimatedStates.add(new DMatrixRMaj(estimatedState));

         System.out.println("Time: " + i);
         System.out.println("State: " + Arrays.toString(state.getData()));
         System.out.println("Measurement: " + Arrays.toString(measurement.getData()));
         System.out.println("Estimated state: " + Arrays.toString(estimatedState.getData()));
         System.out.println("Normalized innovation: " + kalmanFilter.getNormalizedInnovation());
         System.out.println("======================");

         system.updateState(state);
      }

      // Plotting
      double[] trueXData = trueStates.stream().mapToDouble(stateMatrix -> stateMatrix.get(0)).toArray();
      double[] trueXDotData = trueStates.stream().mapToDouble(stateMatrix -> stateMatrix.get(1)).toArray();
      double[] estimateXData = estimatedStates.stream().mapToDouble(stateMatrix -> stateMatrix.get(0)).toArray();
      double[] estimateXDotData = estimatedStates.stream().mapToDouble(stateMatrix -> stateMatrix.get(1)).toArray();
      double[] measurementData = measurements.stream().mapToDouble(measurementMatrix -> measurementMatrix.get(0)).toArray();

      XYChart chart = QuickChart.getChart("Phase Portrait", "q", "qDot", "true", trueXData, trueXDotData);
      chart.addSeries("estimate", estimateXData, estimateXDotData);

      XYChart xTrackingChart = QuickChart.getChart("Pendulum Angle", "t", "q", "true", timestamps, trueXData);
      xTrackingChart.addSeries("estimate", timestamps, estimateXData);

      XYChart xDotTrackingChart = QuickChart.getChart("Pendulum Angular Velocity", "t", "qDot", "true", timestamps, trueXDotData);
      xDotTrackingChart.addSeries("estimate", timestamps, estimateXDotData);

      XYChart energyTrackingChart = QuickChart.getChart("Pendulum Energy", "t", "E", "estimate", timestamps, measurementData);

      new Thread(() ->
                 {
                    JFrame frame = new SwingWrapper(chart).displayChart();
                    JFrame xFrame = new SwingWrapper(xTrackingChart).displayChart();
                    JFrame xDotFrame = new SwingWrapper(xDotTrackingChart).displayChart();
                    JFrame energyFrame = new SwingWrapper(energyTrackingChart).displayChart();
                    SwingUtilities.invokeLater(() -> frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                    SwingUtilities.invokeLater(() -> xFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                    SwingUtilities.invokeLater(() -> xDotFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                    SwingUtilities.invokeLater(() -> energyFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                 }).start();
   }
}
