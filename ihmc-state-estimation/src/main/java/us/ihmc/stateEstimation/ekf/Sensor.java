package us.ihmc.stateEstimation.ekf;

import org.ejml.data.DenseMatrix64F;

/**
 * A sensor interface for the Extended Kalman Filter implementation.
 * <p>
 * It provides the filter with all the information needed to correctly use a measurement from a sensor on a robot.
 * Each sensor can define it's own state as some sensors will need to add biases to the filter that need to be
 * estimated.
 * </p>
 *
 * @author Georg
 *
 */
public abstract class Sensor
{
   /** A default sensor state. */
   private static final State EMPTY_STATE = new EmptyState();

   /**
    * Returns the sensor specific state that is added to the filter to be estimated. Usually this will be a
    * {@link BiasState} that is used with the {@link AngularVelocitySensor} for example.
    *
    * @return the sensor state to be estimated.
    */
   public State getSensorState()
   {
      return EMPTY_STATE;
   }

   /**
    * If a sensor state other then the {@link #EMPTY_STATE} is returned by {@link Sensor#getSensorState()} this
    * method must be overwritten also. It will provide the estimator with information on how the sensor state
    * is affecting the measurement.
    * <p>
    * The linearized measurement equation of the filter is {@code z = H * x + v}. The {@code x} vector is the full
    * estimator state and contains the sensor state of this sensor. This method needs to pack the part of the
    * {@code H} matrix that corresponds to the sensor state only. In the simple case of a bias this would be an
    * indentity matrix.
    * </p>
    *
    * @param jacobianToPack the part of the {@code H} matrix corresponding to the sensor state.
    */
   public void getSensorJacobian(DenseMatrix64F jacobianToPack)
   {
      jacobianToPack.reshape(0, 0);
   }

   /**
    * Provides the size of the measurement. E.g. a {@link JointPositionSensor} has size 1 while a body
    * {@link AngularVelocitySensor} has size 3.
    *
    * @return the size of the measurement vector.
    */
   public abstract int getMeasurementSize();

   /**
    * This method provides the estimator with the current measurement residual as well as the linearized
    * measurement matrix.
    * <p>
    * The linearized measurement equation of the filter is {@code z = H * x + v}. The {@code x} vector is the full
    * estimator state containing the robot state. This method needs to pack the part of the {@code H} matrix that
    * corresponds to the robot state. In addition the measurement residual must be computed and packed by this
    * method. A up to date robot model used by this sensor, the provided {@code robotState}, as well as any sensor
    * state (see {@link #getSensorState()}) can be used for this purpose. The measurement residual is computed as
    * {@code r = z - h(x)} where {@code h(x)} is the (possibly nonlinear) function to compute the expected measurement
    * from the state {@code x}.
    * </p>
    * @param jacobianToPack the part of the {@code H} matrix corresponding to the robot state.
    * @param residualToPack the measurement residual.
    * @param robotState is the up to date state of the robot.
    */
   public abstract void getRobotJacobianAndResidual(DenseMatrix64F jacobianToPack, DenseMatrix64F residualToPack, RobotState robotState);

   /**
    * This method packs the covariance of the observation noise {@code v}. As this value might not be constant
    * (e.g. for a body velocity sensor) this method is called in every estimation tick.
    *
    * @param noiseCovarianceToPack the covariance of the measurement noise.
    */
   public abstract void getRMatrix(DenseMatrix64F noiseCovarianceToPack);
}
