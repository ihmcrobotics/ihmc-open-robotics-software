package us.ihmc.parameterEstimation;

import us.ihmc.matrixlib.NativeKalmanFilter;
import us.ihmc.matrixlib.NativeMatrix;

/**
 * This class provides an implementation of an Extended Kalman Filter (EKF). The EKF is a recursive filter that uses a linearization of the process and
 * measurement models to provide an estimate of the state of a system.
 * <p>
 * This implementation is for state-only systems, no input -- or if you prefer: x_dot = f(x), not x_dot = f(x, u).
 * </p>
 *
 * @author James Foster
 */
public abstract class ExtendedNativeKalmanFilter
{
   /**
    * Covariance 'Q' of the additive white noise in the process model.
    */
   private final NativeMatrix processCovariance;
   /**
    * Covariance 'R' of the additive white noise in the measurement model.
    */
   private final NativeMatrix measurementCovariance;

   /**
    * Container used for storing the linearization 'F' of the process model.
    */
   private final NativeMatrix processJacobian;
   /**
    * Container used for storing the linearization 'H' of the measurement model.
    */
   private final NativeMatrix measurementJacobian;

   private final NativeMatrix state;
   /**
    * Covariance matrix 'P' of the state estimate of the filter.
    */
   private final NativeMatrix covariance;

   /**
    * The predicted estimate of the state according to the process model of the filter.
    */
   final NativeMatrix predictedState;
   /**
    * Covariance matrix of the linearized process model, used in the prediction step of the filter.
    */
   final NativeMatrix predictedCovariance;

   /**
    * The residual 'y' between the actual measurement (i.e. direct from the sensors), and the predicted measurement from the prediction step of the filter.
    */
   private final NativeMatrix measurementResidual;
   /**
    * The Kalman gain of the filter chooses the relative weighting of the prediction (process) and update (measurement) according to the noise profiles of the
    * two processes.
    */
   private final NativeMatrix kalmanGain;

   /**
    * Overall result of the filter -- the estimated state (here called {@code updatedState}) because it is the result of the update step.
    */
   final NativeMatrix updatedState;
   /**
    * The estimate of the error covariance after running the update step.
    */
   final NativeMatrix updatedCovariance;

   public ExtendedNativeKalmanFilter(int stateSize, int measurementSize)
   {
      processCovariance = new NativeMatrix(stateSize, stateSize);
      measurementCovariance = new NativeMatrix(measurementSize, measurementSize);

      processJacobian = new NativeMatrix(stateSize, stateSize);
      measurementJacobian = new NativeMatrix(measurementSize, stateSize);

      state = new NativeMatrix(stateSize, 1);
      predictedState = new NativeMatrix(stateSize, 1);
      covariance = new NativeMatrix(stateSize, stateSize);
      predictedCovariance = new NativeMatrix(stateSize, stateSize);

      measurementResidual = new NativeMatrix(measurementSize, 1);
      kalmanGain = new NativeMatrix(stateSize, measurementSize);
      updatedState = new NativeMatrix(stateSize, 1);
      updatedCovariance = new NativeMatrix(stateSize, stateSize);
   }

   public ExtendedNativeKalmanFilter(NativeMatrix initialState,
                                     NativeMatrix initialCovariance,
                                     NativeMatrix processCovariance,
                                     NativeMatrix measurementCovariance)
   {
      this(initialState.getNumRows(), measurementCovariance.getNumRows());

      this.processCovariance.set(processCovariance);
      this.measurementCovariance.set(measurementCovariance);

      state.set(initialState);
      covariance.set(initialCovariance);
   }

   /**
    * First stage of the filter, using the process model and its noise profile to make a model-based prediction of the state.
    */
   void predictionStep()
   {
      // x_(k|k-1) = f(x_(k-1|k-1))
      predictedState.set(processModel(state));

      processJacobian.set(linearizeProcessModel(state));  // NOTE: process model linearization occurs at x_(k-1|k-1)

      // P_(k|k-1) = F_k * P_(k-1|k-1) * F_k^T + Q_k
      predictedCovariance.zero();
      NativeKalmanFilter.predictErrorCovariance(predictedCovariance, processJacobian, covariance, processCovariance);
   }

   /**
    * Second, corrective stage of the filter. The actual measurement, as well as the measurement model and its noise profile, are used to correct the
    * prediction made in the {@link #predictionStep()}.
    */
   void updateStep(NativeMatrix actual)
   {
      // y_k = z_k - h(x_(k|k-1))
      measurementResidual.set(actual);
      measurementResidual.subtract(measurementResidual, measurementModel(predictedState));

      // NOTE: measurement model linearization occurs at x_(k|k-1)
      measurementJacobian.set(linearizeMeasurementModel(predictedState));

      NativeKalmanFilter.computeKalmanGain(kalmanGain, predictedCovariance, measurementJacobian, measurementCovariance);

      // x_(k|k) = x_(k|k-1) + K_k * y_k
      NativeKalmanFilter.updateState(updatedState, predictedState, kalmanGain, measurementResidual);
      // P_(k|k) = (I - K_k * H_k) * P_(k|k-1)
      NativeKalmanFilter.updateErrorCovariance(updatedCovariance, kalmanGain, measurementJacobian, predictedCovariance);
   }

   /**
    * Due to the recursive nature of the filter, the state and covariance must be updated at the end of the filter.
    */
   private void cleanupStep()
   {
      state.set(updatedState);
      covariance.set(updatedCovariance);
   }

   /**
    * Runs the filter for a single time step.
    *
    * @param actualMeasurement the measurement at the current time step.
    * @return the estimated state at the current time step.
    */
   public NativeMatrix calculateEstimate(NativeMatrix actualMeasurement)
   {
      predictionStep();
      updateStep(actualMeasurement);
      cleanupStep();

      return updatedState;
   }

   /**
    * Must be implemented by the user to provide the linearization of the process model.
    * <p>
    * NOTE: the process model linearization must be taken at the previous state, or x_(k-1|k-1) in filter parlance.
    * </p>
    *
    * @param previousState the state at the previous time step.
    * @return the Jacobian of the process model at the previous state.
    */
   protected abstract NativeMatrix linearizeProcessModel(NativeMatrix previousState);

   /**
    * Must be implemented by the user to provide the linearization of the measurement model.
    * <p>
    * NOTE: the measurement model linearization must be taken at the predicted state, or x_(k|k-1) in filter parlance. See {@link #predictionStep()}.
    * </p>
    *
    * @param predictedState the predicted state after running the prediction step of the filter.
    * @return the Jacobian of the measurement model at the predicted state.
    */
   protected abstract NativeMatrix linearizeMeasurementModel(NativeMatrix predictedState);

   /**
    * Must be implemented by the user to provide the process model.
    *
    * @param state the state at the previous time step.
    * @return the state at the current time step after applying the process model.
    */
   protected abstract NativeMatrix processModel(NativeMatrix state);

   /**
    * Must be implemented by the user to provide the measurement model.
    *
    * @param state the state at the current time step.
    * @return the measurement at the current time step.
    */
   protected abstract NativeMatrix measurementModel(NativeMatrix state);
}
