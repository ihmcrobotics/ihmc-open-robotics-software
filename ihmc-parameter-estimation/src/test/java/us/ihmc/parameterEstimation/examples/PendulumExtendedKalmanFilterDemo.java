package us.ihmc.parameterEstimation.examples;

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
import java.util.function.Function;

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

   private static class PendulumSystem extends NonlinearSystem
   {
      private final Function<DMatrixRMaj, DMatrixRMaj> systemDynamics;
      private final Function<DMatrixRMaj, DMatrixRMaj> measurementDynamics;

      public PendulumSystem(DMatrixRMaj x0,
                            DMatrixRMaj F,
                            DMatrixRMaj H,
                            Function<DMatrixRMaj, DMatrixRMaj> systemDynamics,
                            Function<DMatrixRMaj, DMatrixRMaj> measurementDynamics)
      {
         super(x0, F, H);

         this.systemDynamics = systemDynamics;
         this.measurementDynamics = measurementDynamics;
      }

      public PendulumSystem(DMatrixRMaj x0, DMatrixRMaj F, DMatrixRMaj Q, DMatrixRMaj H, DMatrixRMaj R, Random random,
                            Function<DMatrixRMaj, DMatrixRMaj> systemDynamics,
                            Function<DMatrixRMaj, DMatrixRMaj> measurementDynamics)
      {
         super(x0, F, Q, H, R, random);

         this.systemDynamics = systemDynamics;
         this.measurementDynamics = measurementDynamics;

      }

      @Override
      public DMatrixRMaj calculateSystemDynamics(DMatrixRMaj state)
      {
         return systemDynamics.apply(state);
      }

      @Override
      public DMatrixRMaj calculateMeasurementDynamics(DMatrixRMaj state)
      {
         return measurementDynamics.apply(state);
      }
   }

   private static final int ITERATIONS = 500;

   public static void main(String[] args)
   {
      Random random = new Random(45);

      // Filter
      ExamplePendulumExtendedKalmanFilter kalmanFilter = new ExamplePendulumExtendedKalmanFilter();
      kalmanFilter.setModelProperties(m, I, l, b, g, dt);
      DMatrixRMaj estimatedState = new DMatrixRMaj(ExamplePendulumExtendedKalmanFilter.stateSize, 1);

      // We'll set things up such that the real system is always less noisy than the filter. Said the other way, our filter design
      // will assume that the system is more noisy than it actually is -- a good thing.
      DMatrixRMaj Q = new DMatrixRMaj(kalmanFilter.getQ());
      CommonOps_DDRM.scale(0.1, Q);
      // We'll set things up such that the real system is always less noisy than the filter. Said the other way, our filter design
      // will assume that the system is more noisy than it actually is -- a good thing.
      DMatrixRMaj R = new DMatrixRMaj(kalmanFilter.getR());
      CommonOps_DDRM.scale(0.01, R);

      // In general, initial state of system is different from that of filter.
      DMatrixRMaj initialSystemState = new DMatrixRMaj(new double[] {Math.PI / 4, 0.0});

      // "Real-life" system
      //      NonlinearSystem system = new ConstantVelocity2DNonlinearMeasurementSystem(initialSystemState,
      //                                                                                PendulumExtendedKalmanFilter.F,
      //                                                                                PendulumExtendedKalmanFilter.H);
      NonlinearSystem system = new PendulumSystem(initialSystemState, kalmanFilter.getF(), Q, kalmanFilter.getH(), R, random,
                                                  kalmanFilter::getNextState, kalmanFilter::getMeasurement);

      DMatrixRMaj state = new DMatrixRMaj(ExamplePendulumExtendedKalmanFilter.stateSize, 1);
      DMatrixRMaj measurement = new DMatrixRMaj(ExamplePendulumExtendedKalmanFilter.measurementSize, 1);

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
