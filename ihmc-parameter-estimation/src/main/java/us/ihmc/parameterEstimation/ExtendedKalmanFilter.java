package us.ihmc.parameterEstimation;

import org.ejml.LinearSolverSafe;
import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;

/**
 * This class provides an implementation of an Extended Kalman Filter (EKF). The EKF is a recursive filter that uses a linearization of the process and
 * measurement models to provide an estimate of the state of a system.
 * <p>
 * This implementation is for state-only systems, no input -- or if you prefer: x_dot = f(x), not x_dot = f(x, u).
 * </p>
 *
 * @author James Foster
 */
public abstract class ExtendedKalmanFilter
{
   /**
    * Whether to use the Joseph form of the covariance update. It is slightly less efficient than the default covariance update, but more numerically stable.
    */
   private static final boolean JOSEPH_COVARIANCE_UPDATE = true;

   /**
    * Covariance 'Q' of the additive white noise in the process model.
    */
   protected final DMatrixRMaj processCovariance;
   /**
    * Covariance 'R' of the additive white noise in the measurement model.
    */
   protected final DMatrixRMaj measurementCovariance;

   /**
    * Container used for storing the linearization 'F' of the process model.
    */
   protected final DMatrixRMaj processJacobian;
   /**
    * Container used for storing the linearization 'H' of the measurement model.
    */
   protected final DMatrixRMaj measurementJacobian;

   /**
    * The state estimate of the filter.
    */
   protected final DMatrixRMaj state;
   /**
    * Covariance matrix 'P' of the state estimate of the filter.
    */
   protected final DMatrixRMaj covariance;

   /**
    * The predicted estimate of the state according to the process model of the filter.
    */
   protected final DMatrixRMaj predictedState;
   /**
    * Covariance matrix of the linearized process model, used in the prediction step of the filter.
    */
   protected final DMatrixRMaj predictedCovariance;
   /**
    * Container variable used for garbage free operations.
    */
   private final DMatrixRMaj predictedCovarianceContainer;

   /**
    * The residual 'y' between the actual observation (i.e. direct from the sensors), and the predicted measurement from the prediction step of the filter.
    */
   private final DMatrixRMaj measurementResidual;
   /**
    * Container variable used for garbage free operations.
    */
   private final DMatrixRMaj residualCovarianceContainer;
   /**
    * Covariance matrix 'S' of the linearized measurement model, used in the update step of the filter.
    */
   private final DMatrixRMaj residualCovariance;
   /**
    * Container variable used to hold the inverse of the residual covariance matrix.
    */
   private final DMatrixRMaj inverseResidualCovariance;
   /**
    * Dedicated solver to invert the residual covariance matrix (specialized to symmetric positive definite matrices).
    */
   private final LinearSolverSafe<DMatrixRMaj> solver;
   /**
    * The Kalman gain of the filter chooses the relative weighting of the prediction (process) and update (measurement) according to the noise profiles of the
    * two processes.
    */
   protected final DMatrixRMaj kalmanGain;
   /**
    * Container variable used for garbage free operations.
    */
   private final DMatrixRMaj kalmanGainContainer;

   /**
    * The predicted estimate of the state according to the measurement model of the filter.
    */
   protected final DMatrixRMaj updatedState;
   /**
    * The estimate of the error covariance after running the update step.
    */
   protected final DMatrixRMaj updatedCovariance;
   /**
    * Container variable used for garbage free operations.
    */
   private final DMatrixRMaj updatedCovarianceContainer;

   /**
    * The normalized innovation can be helpful in assessing the convergence of the filter.
    */
   private final DMatrixRMaj normalizedInnovation;
   /**
    * The threshold for gating new measurements based on the normalized innovation. If the normalized innovation is greater than this threshold, the
    * measurement is likely an outlier and is rejected.
    */
   private double normalizedInnovationThreshold = Double.MAX_VALUE;  // Let everything through by default

   /**
    * Container variable used for garbage free operations.
    */
   private final DMatrixRMaj normalizedInnovationContainer;

   /** Terms used for calculating the Joseph form of the covariance update. */
   private final DMatrixRMaj josephTransposeTerm;
   private final DMatrixRMaj josephTransposeTermContainer;
   private final DMatrixRMaj josephIdentity;
   private final DMatrixRMaj josephGainTerm;
   private final DMatrixRMaj josephGainTermContainer;

   public ExtendedKalmanFilter(int stateSize, int measurementSize)
   {
      processCovariance = new DMatrixRMaj(stateSize, stateSize);
      measurementCovariance = new DMatrixRMaj(measurementSize, measurementSize);

      processJacobian = new DMatrixRMaj(stateSize, stateSize);
      measurementJacobian = new DMatrixRMaj(measurementSize, stateSize);

      state = new DMatrixRMaj(stateSize, 1);
      predictedState = new DMatrixRMaj(stateSize, 1);
      covariance = new DMatrixRMaj(stateSize, stateSize);
      predictedCovariance = new DMatrixRMaj(stateSize, stateSize);
      predictedCovarianceContainer = new DMatrixRMaj(stateSize, stateSize);

      measurementResidual = new DMatrixRMaj(measurementSize, 1);
      residualCovarianceContainer = new DMatrixRMaj(measurementSize, measurementSize);
      residualCovariance = new DMatrixRMaj(measurementSize, measurementSize);
      inverseResidualCovariance = new DMatrixRMaj(measurementSize, measurementSize);
      solver = new LinearSolverSafe<>(LinearSolverFactory_DDRM.symmPosDef(measurementSize));
      kalmanGain = new DMatrixRMaj(stateSize, measurementSize);
      kalmanGainContainer = new DMatrixRMaj(stateSize, measurementSize);
      updatedState = new DMatrixRMaj(stateSize, 1);
      updatedCovariance = new DMatrixRMaj(stateSize, stateSize);
      updatedCovarianceContainer = new DMatrixRMaj(stateSize, stateSize);

      normalizedInnovation = new DMatrixRMaj(measurementSize, 1);
      normalizedInnovationContainer = new DMatrixRMaj(measurementSize, 1);

      josephTransposeTerm = new DMatrixRMaj(stateSize, stateSize);
      josephTransposeTermContainer = new DMatrixRMaj(stateSize, stateSize);
      josephIdentity = CommonOps_DDRM.identity(stateSize);
      josephGainTerm = new DMatrixRMaj(stateSize, stateSize);
      josephGainTermContainer = new DMatrixRMaj(stateSize, stateSize);
   }

   public ExtendedKalmanFilter(DMatrixRMaj initialState, DMatrixRMaj initialCovariance, DMatrixRMaj processCovariance, DMatrixRMaj measurementCovariance)
   {
      this(initialState.getNumRows(), measurementCovariance.getNumRows());

      // State must be a column vector
      if (initialState.numCols != 1)
         throw new IllegalArgumentException("Initial state must be a column vector.");

      // Covariance matrices must be square
      if (initialCovariance.numRows != initialCovariance.numCols || processCovariance.numRows != processCovariance.numCols
          || measurementCovariance.numRows != measurementCovariance.numCols)
         throw new IllegalArgumentException("The input initial, process, and measurement covariance matrices must be square.");

      // Covariance matrices must be positive definite
      if (!MatrixFeatures_DDRM.isPositiveDefinite(initialCovariance) || !MatrixFeatures_DDRM.isPositiveDefinite(processCovariance)
          || !MatrixFeatures_DDRM.isPositiveDefinite(measurementCovariance))
         throw new IllegalArgumentException("The input initial, process, and measurement covariance matrices must be positive definite.");

      this.processCovariance.set(processCovariance);
      this.measurementCovariance.set(measurementCovariance);

      state.set(initialState);
      covariance.set(initialCovariance);
   }

   /**
    * Runs the filter for a single time step.
    *
    * @param observation the observation at the current time step.
    * @return the estimated state at the current time step.
    */
   public final DMatrixRMaj calculateEstimate(DMatrix observation)
   {
      preSolveHook();
      predictionStep();
      calculateMeasurementResidual(observation);
      preUpdateHook();
      boolean validMeasurement = queryInnovationGate();
      if (validMeasurement)
         updateStep();
      else
         applyInnovationGate();
      postSolveHook();

      return state;
   }

   /**
    * A hook that the user can override to run code before the prediction step of the filter.
    */
   protected void preSolveHook()
   {
      // Default empty
   }

   /**
    * A hook that the user can override to run code before the update step of the filter.
    */
   protected void preUpdateHook()
   {
      // Default empty
   }

   /**
    * A hook that the user can override to run code after the filter solve is complete.
    */
   protected void postSolveHook()
   {
      // Default empty
   }

   /**
    * First stage of the filter, using the process model and its noise profile to make a model-based prediction of the state.
    */
   protected void predictionStep()
   {
      // x_(k|k-1) = f(x_(k-1|k-1))
      predictedState.set(processModel(state));

      processJacobian.set(linearizeProcessModel(state));  // NOTE: process model linearization occurs at x_(k-1|k-1)

      // P_(k|k-1) = F_k * P_(k-1|k-1) * F_k^T + Q_k
      predictedCovariance.zero();
      CommonOps_DDRM.mult(processJacobian, covariance, predictedCovarianceContainer);
      CommonOps_DDRM.multTransB(predictedCovarianceContainer, processJacobian, predictedCovariance);
      CommonOps_DDRM.addEquals(predictedCovariance, processCovariance);
   }

   /** Technically part of the update step of the filter, we keep it separate so that users can hook in between the residual calculation and the update step. */
   void calculateMeasurementResidual(DMatrix actual)
   {
      // y_k = z_k - h(x_(k|k-1))
      measurementResidual.set(actual);
      CommonOps_DDRM.subtractEquals(measurementResidual, measurementModel(predictedState));
   }

   /**
    * With the residual calculated in {@link #calculateMeasurementResidual(DMatrix)}, we can now check if the observation is valid. This is done by comparing
    * the normalized innovation to a threshold.
    */
   boolean queryInnovationGate()
   {
      measurementJacobian.set(linearizeMeasurementModel(predictedState));  // NOTE: measurement model linearization occurs at x_(k|k-1)
      calculateResidualCovarianceAndInverse();

      return calculateNormalizedInnovation() < normalizedInnovationThreshold;
   }

   /**
    * If the observation is invalid, it is rejected and the state and covariance are simply updated as the result of the prediction step.
    */
   private void applyInnovationGate()
   {
      state.set(predictedState);
      covariance.set(predictedCovariance);
   }

   /**
    * If the observation is valid, the actual observation, as well as the measurement model and its noise profile, are used to correct the
    * prediction made in the {@link #predictionStep()}.
    */
   protected void updateStep()
   {
      calculateKalmanGain();

      // x_(k|k) = x_(k|k-1) + K_k * y_k
      updatedState.set(predictedState);
      CommonOps_DDRM.multAdd(kalmanGain, measurementResidual, updatedState);
      calculateUpdatedCovariance();

      state.set(updatedState);
      covariance.set(updatedCovariance);
   }

   private void calculateResidualCovarianceAndInverse()
   {
      // S_k = H_k * P_(k|k-1) * H_k^T + R_k
      residualCovariance.zero();
      CommonOps_DDRM.mult(measurementJacobian, predictedCovariance, residualCovarianceContainer);
      CommonOps_DDRM.multTransB(residualCovarianceContainer, measurementJacobian, residualCovariance);
      CommonOps_DDRM.addEquals(residualCovariance, measurementCovariance);

      solver.setA(residualCovariance);
      solver.invert(inverseResidualCovariance);
   }

   protected void calculateKalmanGain()
   {
      // K_k = P_(k|k-1) * H_k^T * S_k^-1
      kalmanGain.zero();
      CommonOps_DDRM.multTransB(predictedCovariance, measurementJacobian, kalmanGainContainer);
      CommonOps_DDRM.mult(kalmanGainContainer, inverseResidualCovariance, kalmanGain);
   }

   protected void calculateUpdatedCovariance()
   {
      // Search for "Joseph form": https://en.wikipedia.org/wiki/Kalman_filter#Derivations
      if (JOSEPH_COVARIANCE_UPDATE)
      {
         // Joseph form of covariance update is apparently more numerically stable
         // P_(k|k) = (I - K_k * H_k) * P_(k|k-1) * (I - K_k * H_k)^T + K_k * R_k * K_k^T
         josephGainTerm.set(kalmanGain);
         CommonOps_DDRM.mult(josephGainTerm, measurementCovariance, josephGainTermContainer);
         CommonOps_DDRM.multTransB(josephGainTermContainer, kalmanGain, josephGainTerm);

         josephTransposeTermContainer.set(kalmanGain);
         CommonOps_DDRM.mult(josephTransposeTermContainer, measurementJacobian, josephTransposeTerm);
         CommonOps_DDRM.scale(-1.0, josephTransposeTerm);
         CommonOps_DDRM.addEquals(josephTransposeTerm, josephIdentity);

         // Now put these terms into the covariance update variables
         CommonOps_DDRM.multTransB(predictedCovariance, josephTransposeTerm, updatedCovarianceContainer);
         CommonOps_DDRM.mult(josephTransposeTerm, updatedCovarianceContainer, updatedCovariance);
         CommonOps_DDRM.addEquals(updatedCovariance, josephGainTerm);
      }
      else  // DEFAULT
      {
         // P_(k|k) = (I - K_k * H_k) * P_(k|k-1)
         updatedCovariance.set(kalmanGain);
         CommonOps_DDRM.mult(updatedCovariance, measurementJacobian, updatedCovarianceContainer);
         CommonOps_DDRM.mult(updatedCovarianceContainer, predictedCovariance, updatedCovariance);
         CommonOps_DDRM.scale(-1.0, updatedCovariance);
         CommonOps_DDRM.addEquals(updatedCovariance, predictedCovariance);
      }
   }

   /**
    * Calculates the normalized innovation of the filter. This can be helpful in assessing the convergence of the filter.
    * <p>
    * The normalized innovation is defined as: y_k^T * S_k^-1 * y_k, where y_k is the measurement residual and S_k is the residual covariance. The intuition
    * is,
    * as the filter converges, the measurement residual should become a white noise process around zero (meaning low estimation error) with a small variance.
    * This calculation is a way of quantifying that intuition. See: <a
    * href="https://stackoverflow.com/questions/19498683/ekf-how-to-detect-if-filter-is-converged">...</a>
    * </p>
    */
   private double calculateNormalizedInnovation()
   {
      CommonOps_DDRM.mult(inverseResidualCovariance, measurementResidual, normalizedInnovationContainer);
      CommonOps_DDRM.multTransA(measurementResidual, normalizedInnovationContainer, normalizedInnovation);
      return normalizedInnovation.getData()[0];
   }

   /**
    * Must be implemented by the user to provide the process model.
    *
    * @param state the state at the previous time step.
    * @return the state at the current time step after applying the process model.
    */
   protected abstract DMatrixRMaj processModel(DMatrixRMaj state);

   /**
    * Must be implemented by the user to provide the measurement model.
    *
    * @param state the state at the current time step.
    * @return the measurement at the current time step.
    */
   protected abstract DMatrixRMaj measurementModel(DMatrixRMaj state);

   /**
    * Must be implemented by the user to provide the linearization of the process model.
    * <p>
    * NOTE: the process model linearization must be taken at the previous state, or x_(k-1|k-1) in filter parlance.
    * </p>
    *
    * @param previousState the state at the previous time step.
    * @return the Jacobian of the process model at the previous state.
    */
   protected abstract DMatrixRMaj linearizeProcessModel(DMatrixRMaj previousState);

   /**
    * Must be implemented by the user to provide the linearization of the measurement model.
    * <p>
    * NOTE: the measurement model linearization must be taken at the predicted state, or x_(k|k-1) in filter parlance. See {@link #predictionStep()}.
    * </p>
    *
    * @param predictedState the predicted state after running the prediction step of the filter.
    * @return the Jacobian of the measurement model at the predicted state.
    */
   protected abstract DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedState);

   public void setProcessCovariance(DMatrix processCovariance)
   {
      this.processCovariance.set(processCovariance);
   }

   public void setMeasurementCovariance(DMatrix measurementCovariance)
   {
      this.measurementCovariance.set(measurementCovariance);
   }

   public DMatrixRMaj getMeasurementResidual()
   {
      return measurementResidual;
   }

   public double getNormalizedInnovation()
   {
      return normalizedInnovation.getData()[0];
   }

   public void setNormalizedInnovationThreshold(double normalizedInnovationThreshold)
   {
      this.normalizedInnovationThreshold = normalizedInnovationThreshold;
   }
}
