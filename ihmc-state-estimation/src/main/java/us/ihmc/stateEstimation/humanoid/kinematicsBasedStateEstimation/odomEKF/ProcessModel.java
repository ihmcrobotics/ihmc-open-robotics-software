package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryKalmanFilter.StateVariables;
import us.ihmc.yoVariables.providers.BooleanProvider;

class ProcessModel
{
   private static final DMatrixRMaj eye3x3 = CommonOps_DDRM.identity(3);

   // State providers
   private final StateVariables currentState;
   private final StateVariables predictedState;
   private final BooleanProvider cancelGravityFromAccelerationMeasurement;
   private final FrameVector3DReadOnly gravityVector;
   private final double estimatorDt;

   // Temp variables
   private final Vector3D integratedVelocity = new Vector3D();
   private final Vector3D unbiasedAcceleration = new Vector3D();
   private final Quaternion integratedRotation = new Quaternion();

   public ProcessModel(StateVariables processState,
                       StateVariables predictedState,
                       BooleanProvider cancelGravityFromAccelerationMeasurement,
                       FrameVector3DReadOnly gravityVector,
                       double estimatorDt)
   {
      this.currentState = processState;
      this.predictedState = predictedState;
      this.cancelGravityFromAccelerationMeasurement = cancelGravityFromAccelerationMeasurement;
      this.gravityVector = gravityVector;
      this.estimatorDt = estimatorDt;
   }

   public void update()
   {
      // transform the acceleration to world
      unbiasedAcceleration.set(currentState.unbiasedAccel);
      currentState.orientation.transform(unbiasedAcceleration);
      if (cancelGravityFromAccelerationMeasurement.getValue())
      {
         // remove gravity from acceleration
         unbiasedAcceleration.sub(gravityVector);
      }

      // First order integration of the position state
      predictedState.translation.scaleAdd(estimatorDt, currentState.linearVelocity, currentState.translation);

      // Integration of the velocity state
      predictedState.linearVelocity.scaleAdd(estimatorDt, unbiasedAcceleration, currentState.linearVelocity);

      // First order integration of the angular state along the SO(3) manifold
      integratedVelocity.setAndScale(estimatorDt, currentState.unbiasedGyro);
      integratedRotation.setRotationVector(integratedVelocity);
      QuaternionTools.multiply(currentState.orientation, integratedRotation, predictedState.orientation);

      // Propagate the bias
      predictedState.accelBias.set(currentState.accelBias);
      predictedState.gyroBias.set(currentState.gyroBias);
   }

   /**
    * This is equation 39
    */
   public static void computeProcessJacobian(StateVariables stateVariables, double estimateDT, int offset, DMatrixRMaj jacobianToPack)
   {
      // Do the partial derivative with respect to the base position state
      int rowStart = offset + OdometryIndexHelper.getStatePositionIndex();
      CommonOps_DDRM.insert(eye3x3, jacobianToPack, rowStart, offset + OdometryIndexHelper.getStatePositionIndex());
      MatrixTools.setMatrixBlock(jacobianToPack, rowStart, offset + OdometryIndexHelper.getStateVelocityIndex(), eye3x3, 0, 0, 3, 3, estimateDT);

      // Do the partial derivative with respect to the base velocity state FIXME lots of garbage
      rowStart = offset + OdometryIndexHelper.getStateVelocityIndex();
      CommonOps_DDRM.insert(eye3x3, jacobianToPack, rowStart, offset + OdometryIndexHelper.getStateVelocityIndex());

      DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
      DMatrixRMaj skewMatrix = new DMatrixRMaj(3, 3);
      OdometryTools.toRotationMatrix(stateVariables.orientation, rotationMatrix);
      OdometryTools.toSkewSymmetricMatrix(stateVariables.unbiasedAccel, skewMatrix);
      MatrixTools.multAddBlock(-estimateDT, rotationMatrix, skewMatrix, jacobianToPack, rowStart, offset + OdometryIndexHelper.getStateOrientationIndex());

      if (OdometryKalmanFilter.includeBias)
         MatrixTools.setMatrixBlock(jacobianToPack, rowStart, offset + OdometryIndexHelper.getStateGyroBiasIndex(), rotationMatrix, 0, 0, 3, 3, -estimateDT);

      // Do the partial derivative with respect to the orientation state
      rowStart = offset + OdometryIndexHelper.getStateOrientationIndex();
      OdometryTools.toSkewSymmetricMatrix(stateVariables.unbiasedGyro, skewMatrix);
      MatrixTools.setMatrixBlock(jacobianToPack, rowStart, offset + OdometryIndexHelper.getStateOrientationIndex(), rotationMatrix, 0, 0, 3, 3, -estimateDT);
      if (OdometryKalmanFilter.includeBias)
      {
         MatrixTools.addMatrixBlock(jacobianToPack, rowStart, offset + OdometryIndexHelper.getStateOrientationIndex(), eye3x3, 0, 0, 3, 3, 1.0);
         MatrixTools.setMatrixBlock(jacobianToPack, rowStart, offset + OdometryIndexHelper.getStateAccelerationBiasIndex(), eye3x3, 0, 0, 3, 3, -estimateDT);
      }

      // Do the partial derivative with respect to the acceleration bias state
      MatrixTools.setMatrixBlock(jacobianToPack,
                                 offset + OdometryIndexHelper.getStateAccelerationBiasIndex(),
                                 offset + OdometryIndexHelper.getStateAccelerationBiasIndex(),
                                 eye3x3,
                                 0,
                                 0,
                                 3,
                                 3,
                                 1.0);
      // Do the partial derivative with respect to the gyro bias state
      MatrixTools.setMatrixBlock(jacobianToPack,
                                 offset + OdometryIndexHelper.getStateGyroBiasIndex(),
                                 offset + OdometryIndexHelper.getStateGyroBiasIndex(),
                                 eye3x3,
                                 0,
                                 0,
                                 3,
                                 3,
                                 1.0);
   }
}
