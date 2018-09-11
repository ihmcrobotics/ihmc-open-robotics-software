package us.ihmc.stateEstimation.ekf;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class StateEstimator
{
   private final RobotState robotState;

   private final ComposedState state;
   private final ComposedSensor sensor;

   private final ExecutionTimer predictionTimer;
   private final ExecutionTimer correctionTimer;

   private final FilterMatrixOps filterMatrixOps = new FilterMatrixOps();

   public StateEstimator(List<Sensor> sensors, RobotState robotState, YoVariableRegistry registry)
   {
      this.robotState = robotState;
      this.state = new ComposedState();
      this.sensor = new ComposedSensor(sensors, robotState.getSize());

      state.addState(robotState);
      state.addState(sensor.getSensorState());

      filterMatrixOps.setIdentity(Pposterior, state.getSize());
      CommonOps.scale(1.0, Pposterior);

      predictionTimer = new ExecutionTimer(getClass().getSimpleName() + "Prediction", registry);
      correctionTimer = new ExecutionTimer(getClass().getSimpleName() + "Correction", registry);
   }

   private final DenseMatrix64F F = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Q = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F H = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F R = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F K = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F residual = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F Xprior = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Pprior = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Xposterior = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Pposterior = new DenseMatrix64F(0, 0);

   public void predict()
   {
      predictionTimer.startMeasurement();

      // State prediction.
      state.predict();

      // Get linearized plant model and predict error covariance.
      state.getFMatrix(F);
      state.getQMatrix(Q);
      filterMatrixOps.predictErrorCovariance(Pprior, F, Pposterior, Q);

      predictionTimer.stopMeasurement();
   }

   public void correct()
   {
      correctionTimer.startMeasurement();

      // From the sensor get the linearized measurement model and the measurement residual
      sensor.assembleFullJacobian(H, residual, robotState);

      // Compute the kalman gain and correct the state
      sensor.getRMatrix(R);
      if (!filterMatrixOps.computeKalmanGain(K, Pprior, H, R))
      {
         PrintTools.info("Inversion failed integrating only.");
         correctionTimer.stopMeasurement();
         return;
      }
      state.getStateVector(Xprior);
      filterMatrixOps.updateState(Xposterior, K, residual, Xprior);

      // Update the error covariance.
      filterMatrixOps.updateErrorCovariance(Pposterior, K, H, R, Pprior);

      // Update the state data structure after the correction step.
      state.setStateVector(Xposterior);

      correctionTimer.stopMeasurement();
   }

   public void getCovariance(DenseMatrix64F covarianceToPack)
   {
      covarianceToPack.set(Pposterior);
   }

   public void resetCovariance(double variance)
   {
      filterMatrixOps.setIdentity(Pposterior, state.getSize());
      CommonOps.scale(variance, Pposterior);
   }
}
