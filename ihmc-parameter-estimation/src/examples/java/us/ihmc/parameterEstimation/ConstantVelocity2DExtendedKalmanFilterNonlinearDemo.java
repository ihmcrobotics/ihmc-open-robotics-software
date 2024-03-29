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
 * Demo of an EKF being used with a linear process model (can use A directly) and nonlinear measurement model (must compute h(x)).
 * <p>
 * The system under consideration is a 2D planar system with constant velocity. The state is [x, xDot, y, yDot] and the measurement is [r, theta] (i.e. 2D
 * radius and angle).
 * </p>
 */
public class ConstantVelocity2DExtendedKalmanFilterNonlinearDemo
{
   public static class ConstantVelocity2DKalmanFilter extends ExtendedKalmanFilter
   {
      private static final int stateSize = 4;
      private static final int measurementSize = 2;

      // Starting at 0 velocity. Cannot start at (x,y) = (0,0) because of singularity in polar coordinates, so we start at 0.001.
      private static final DMatrixRMaj x0 = new DMatrixRMaj(new double[] {0.001, 0.0, 0.001, 0.0});

      // Position is barely affected by noise, velocity is affected by noise
      private static final DMatrixRMaj Q = new DMatrixRMaj(new double[][] {{1e-6, 0.0, 0.0, 0.0},
                                                                           {0.0, 1e-6, 0.0, 0.0},
                                                                           {0.0, 0.0, 1e-6, 0.0},
                                                                           {0.0, 0.0, 0.0, 1e-6}});

      // Both distance to target 'r' and azimuth angle 'theta' measurements are corrupted by noise
      private static final DMatrixRMaj R = new DMatrixRMaj(new double[][] {{25, 0.0}, {0.0, 1e-6}});

      // Ignorant initial guess on P0, we assume we're more certain about positions than velocities
      private static final DMatrixRMaj P0 = new DMatrixRMaj(new double[][] {{0.1, 0.0, 0.0, 0.0},
                                                                            {0.0, 1.0, 0.0, 0.0},
                                                                            {0.0, 0.0, 0.1, 0.0},
                                                                            {0.0, 0.0, 0.0, 1.0}});

      static final double dt = 0.01;

      // Constant velocity model of a 2D planar system
      private static final DMatrixRMaj A = new DMatrixRMaj(new double[][] {{1.0, dt, 0.0, 0.0},
                                                                           {0.0, 1.0, 0.0, 0.0},
                                                                           {0.0, 0.0, 1.0, dt},
                                                                           {0.0, 0.0, 0.0, 1.0}});

      // We just need to define the size of H here, the contents should not matter since it will be updated.
      private static final DMatrixRMaj H = new DMatrixRMaj(measurementSize, stateSize);

      public ConstantVelocity2DKalmanFilter()
      {
         super(x0, P0, Q, R);
      }

      // In the case of a linear process model, the linearization is just the A matrix of the process model
      @Override
      public DMatrixRMaj linearizeProcessModel(DMatrixRMaj previousState)
      {
         return A;
      }

      // We need to linearize h(x) to obtain it's Jacobian matrix, H(x)
      @Override
      public DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedState)
      {
         DMatrixRMaj measurement = getPolarMeasurement(predictedState);
         double r = measurement.get(0, 0);
         double theta = measurement.get(1, 0);

         H.zero();
         H.set(0, 0, Math.cos(theta));
         H.set(0, 2, Math.sin(theta));
         H.set(1, 0, -Math.sin(theta) / r);
         H.set(1, 2, Math.cos(theta) / r);

         return H;
      }

      // y = Ax
      @Override
      public DMatrixRMaj processModel(DMatrixRMaj state)
      {
         DMatrixRMaj nextState = new DMatrixRMaj(stateSize, 1);
         CommonOps_DDRM.mult(A, state, nextState);
         return nextState;
      }

      // y = h(x)
      @Override
      public DMatrixRMaj measurementModel(DMatrixRMaj state)
      {
         return getPolarMeasurement(state);
      }
   }

   private static class ConstantVelocity2DNonlinearMeasurementSystem extends NonlinearSystem
   {
      public ConstantVelocity2DNonlinearMeasurementSystem(DMatrixRMaj x0, DMatrixRMaj F, DMatrixRMaj H)
      {
         super(x0, F, H);
      }

      public ConstantVelocity2DNonlinearMeasurementSystem(DMatrixRMaj x0, DMatrixRMaj F, DMatrixRMaj Q, DMatrixRMaj H, DMatrixRMaj R, Random random)
      {
         super(x0, F, Q, H, R, random);
      }

      @Override
      public DMatrixRMaj calculateSystemDynamics(DMatrixRMaj state)
      {
         DMatrixRMaj nextState = new DMatrixRMaj(ConstantVelocity2DKalmanFilter.stateSize, 1);
         CommonOps_DDRM.mult(ConstantVelocity2DKalmanFilter.A, state, nextState);
         return nextState;
      }

      @Override
      public DMatrixRMaj calculateMeasurementDynamics(DMatrixRMaj state)
      {
         return getPolarMeasurement(state);
      }
   }

   private static DMatrixRMaj getPolarMeasurement(DMatrixRMaj state)
   {
      double r = Math.sqrt(state.get(0) * state.get(0) + state.get(2) * state.get(2));
      double theta = Math.atan2(state.get(2), state.get(0));

      DMatrixRMaj measurement = new DMatrixRMaj(ConstantVelocity2DKalmanFilter.measurementSize, 1);
      measurement.set(0, 0, r);
      measurement.set(1, 0, theta);

      return measurement;
   }

   private static final int ITERATIONS = 2000;

   public static void main(String[] args)
   {
      Random random = new Random(45);

      // We'll set things up such that the real system is always less noisy than the filter. Said the other way, our filter design
      // will assume that the system is more noisy than it actually is -- a good thing.
      DMatrixRMaj Q = new DMatrixRMaj(ConstantVelocity2DKalmanFilter.Q);
      CommonOps_DDRM.scale(0.1, Q);
      // We'll set things up such that the real system is always less noisy than the filter. Said the other way, our filter design
      // will assume that the system is more noisy than it actually is -- a good thing.
      DMatrixRMaj R = new DMatrixRMaj(ConstantVelocity2DKalmanFilter.R);
      CommonOps_DDRM.scale(0.01, R);

      // In general, initial state of system is different from that of filter.
      DMatrixRMaj initialSystemState = new DMatrixRMaj(new double[] {1.0, 1.0, 1.0, 1.0});

      // "Real-life" system
      //      NonlinearSystem system = new ConstantVelocity2DNonlinearMeasurementSystem(initialSystemState,
      //                                                   ConstantVelocity2DKalmanFilter.A,
      //                                                   ConstantVelocity2DKalmanFilter.H);
      NonlinearSystem system = new ConstantVelocity2DNonlinearMeasurementSystem(initialSystemState,
                                                                                ConstantVelocity2DKalmanFilter.A,
                                                                                Q,
                                                                                ConstantVelocity2DKalmanFilter.H,
                                                                                R,
                                                                                random);

      DMatrixRMaj state = new DMatrixRMaj(ConstantVelocity2DKalmanFilter.stateSize, 1);
      DMatrixRMaj measurement = new DMatrixRMaj(ConstantVelocity2DKalmanFilter.measurementSize, 1);

      // Filter
      ConstantVelocity2DKalmanFilter kalmanFilter = new ConstantVelocity2DKalmanFilter();
      DMatrixRMaj estimatedState = new DMatrixRMaj(ConstantVelocity2DKalmanFilter.stateSize, 1);

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

         timestamps[i] = i * ConstantVelocity2DKalmanFilter.dt;
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
      double[] trueYData = trueStates.stream().mapToDouble(stateMatrix -> stateMatrix.get(2)).toArray();
      double[] trueYDotData = trueStates.stream().mapToDouble(stateMatrix -> stateMatrix.get(3)).toArray();
      double[] estimateXData = estimatedStates.stream().mapToDouble(stateMatrix -> stateMatrix.get(0)).toArray();
      double[] estimateXDotData = estimatedStates.stream().mapToDouble(stateMatrix -> stateMatrix.get(1)).toArray();
      double[] estimateYData = estimatedStates.stream().mapToDouble(stateMatrix -> stateMatrix.get(2)).toArray();
      double[] estimateYDotData = estimatedStates.stream().mapToDouble(stateMatrix -> stateMatrix.get(3)).toArray();

      XYChart chart = QuickChart.getChart("2D Position", "x", "y", "true", trueXData, trueYData);
      chart.addSeries("estimate", estimateXData, estimateYData);

      XYChart xTrackingChat = QuickChart.getChart("Horizontal Position", "t", "x", "true", timestamps, trueXData);
      xTrackingChat.addSeries("estimate", timestamps, estimateXData);

      XYChart xDotTrackingChat = QuickChart.getChart("Horizontal Velocity", "t", "xDot", "true", timestamps, trueXDotData);
      xDotTrackingChat.addSeries("estimate", timestamps, estimateXDotData);

      XYChart yTrackingChat = QuickChart.getChart("Vertical Position", "t", "y", "true", timestamps, trueYData);
      yTrackingChat.addSeries("estimate", timestamps, estimateYData);

      XYChart yDotTrackingChat = QuickChart.getChart("Vertical Velocity", "t", "yDot", "true", timestamps, trueYDotData);
      yDotTrackingChat.addSeries("estimate", timestamps, estimateYDotData);

      new Thread(() ->
                 {
                    JFrame frame = new SwingWrapper(chart).displayChart();
                    JFrame xFrame = new SwingWrapper(xTrackingChat).displayChart();
                    JFrame xDotFrame = new SwingWrapper(xDotTrackingChat).displayChart();
                    JFrame yFrame = new SwingWrapper(yTrackingChat).displayChart();
                    JFrame yDotFrame = new SwingWrapper(yDotTrackingChat).displayChart();
                    SwingUtilities.invokeLater(() -> frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                    SwingUtilities.invokeLater(() -> xFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                    SwingUtilities.invokeLater(() -> xDotFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                    SwingUtilities.invokeLater(() -> yFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                    SwingUtilities.invokeLater(() -> yDotFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                 }).start();
   }
}
