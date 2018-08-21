package us.ihmc.stateEstimation.ekf;

import org.ejml.data.DenseMatrix64F;

/**
 * A state interface for the Extended Kalman Filter implementation.
 * <p>
 * I can be extended to represent any state such as a sensor bias, a joint state, or a pose state for a robot.
 * </p>
 *
 * @author Georg
 *
 */
public abstract class State
{
   /**
    * Sets the state vector of this state. This method is called by the filter when a correction of the state
    * based on a measurement was made. Depending on the implementation of the state this can be an error (in
    * case an error state is used as in the {@link PoseState} for the orientation) or simply the new state
    * vector.
    *
    * @param newState the new state that should be stored inside this class.
    */
   public abstract void setStateVector(DenseMatrix64F newState);

   /**
    * Packs the state that is stored in this class. This will be called after the prediction to obtain the
    * most up to date state. In the case of an error state this should return a vector with zeroes. See the
    * {@link PoseState} for an example of this.
    *
    * @param stateVectorToPack will be modified to contain the state stored in this class.
    */
   public abstract void getStateVector(DenseMatrix64F vectorToPack);

   /**
    * Provides the size of the state. E.g. a {@link JointState} has size 3 (position, velocity, and acceleration
    * of the joint) while a {@link PoseState} has size 18.
    *
    * @return the size of the state.
    */
   public abstract int getSize();

   /**
    * Should perform a single prediction step on the state. This method will be called once per estimation tick.
    * In case of a linear state this would be equivalent to {@code state = F * state}. However, this method can
    * be overwritten to perform a non-linear prediction according to {@code state = f(state)}.
    */
   public abstract void predict();

   /**
    * This method will pack the linearized state {@code F} matrix describing the state evolution. In the general
    * case the state is updated according to {@code state(t+dt) = f(state(t)) + w}. This matrix packs the linearized
    * matrix {@code F} such that {@code state(t+dt) ~= F * state(t)}.
    *
    * @param matrixToPack packs the matrix describing the linearized state evolution.
    */
   public abstract void getFMatrix(DenseMatrix64F matrixToPack);

   /**
    * This method packs the covariance of the process noise {@code w}. As this value might not be constant this
    * method is called in every estimation tick.
    *
    * @param noiseCovarianceToPack the covariance of the process noise.
    */
   public abstract void getQMatrix(DenseMatrix64F noiseCovarianceToPack);
}
