package us.ihmc.parameterEstimation.examples;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.knowm.xchart.QuickChart;
import org.knowm.xchart.SwingWrapper;
import org.knowm.xchart.XYChart;
import us.ihmc.parameterEstimation.ExtendedKalmanFilterTestTools.LinearSystem;

import javax.swing.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

/**
 * Demo of an EKF being used as a regular KF (i.e. linear process and measurement models are linear, so their Jacobians are just the A and C matrices).
 * <p>
 * The system under consideration is a 2D planar system with constant velocity. The state is [x, xDot, y, yDot] and the measurement is [x, y] (i.e. we measure
 * position but not velocity).
 * </p>
 */
public class ConstantVelocity2DExtendedKalmanFilterDemo
{

   private static final int ITERATIONS = 1000;

   public static void main(String[] args)
   {
      Random random = new Random(45);

      // We'll set things up such that the real system is always less noisy than the filter. Said the other way, our filter design
      // will assume that the system is more noisy than it actually is -- a good thing.
      DMatrixRMaj Q = new DMatrixRMaj(ExampleConstantVelocity2DKalmanFilter.Q);
      CommonOps_DDRM.scale(0.1, Q);
      // We'll set things up such that the real system is always less noisy than the filter. Said the other way, our filter design
      // will assume that the system is more noisy than it actually is -- a good thing.
      DMatrixRMaj R = new DMatrixRMaj(ExampleConstantVelocity2DKalmanFilter.R);
      CommonOps_DDRM.scale(0.01, R);

      // In general, initial state of system is different from that of filter.
      DMatrixRMaj initialSystemState = new DMatrixRMaj(new double[] {1.0, 1.0, 1.0, 1.0});

      // "Real-life" system
      //      LinearSystem system = new LinearSystem(initialSystemState,
      //                                             ConstantVelocity2DKalmanFilter.A,
      //                                             ConstantVelocity2DKalmanFilter.C);
      LinearSystem system = new LinearSystem(initialSystemState, ExampleConstantVelocity2DKalmanFilter.A, Q, ExampleConstantVelocity2DKalmanFilter.C, R, random);

      DMatrixRMaj state = new DMatrixRMaj(ExampleConstantVelocity2DKalmanFilter.stateSize, 1);
      DMatrixRMaj measurement = new DMatrixRMaj(ExampleConstantVelocity2DKalmanFilter.measurementSize, 1);

      // Filter
      ExampleConstantVelocity2DKalmanFilter kalmanFilter = new ExampleConstantVelocity2DKalmanFilter();
      DMatrixRMaj estimatedState = new DMatrixRMaj(ExampleConstantVelocity2DKalmanFilter.stateSize, 1);

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

         timestamps[i] = i * ExampleConstantVelocity2DKalmanFilter.dt;
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

      XYChart chart = QuickChart.getChart("2D Position", "X", "Y", "True", trueXData, trueYData);
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
                    javax.swing.SwingUtilities.invokeLater(() -> frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                    javax.swing.SwingUtilities.invokeLater(() -> xFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                    javax.swing.SwingUtilities.invokeLater(() -> xDotFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                    javax.swing.SwingUtilities.invokeLater(() -> yFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                    javax.swing.SwingUtilities.invokeLater(() -> yDotFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE));
                 }).start();
   }
}
