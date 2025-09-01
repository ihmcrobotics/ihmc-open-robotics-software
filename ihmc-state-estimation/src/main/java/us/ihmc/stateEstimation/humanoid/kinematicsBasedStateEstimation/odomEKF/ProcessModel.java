package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.yoVariables.providers.BooleanProvider;

import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryIndexHelper.*;

class ProcessModel
{
   private static final DMatrixRMaj eye3x3 = CommonOps_DDRM.identity(3);
   private final DMatrixRMaj deltaT;

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
   private final DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj skewMatrix = new DMatrixRMaj(3, 3);


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

      deltaT = new DMatrixRMaj(eye3x3);
      CommonOps_DDRM.scale(estimatorDt, deltaT);
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
   public void computeProcessJacobian(StateVariables stateVariables, double estimateDT, int offset, DMatrixRMaj jacobianToPack)
   {
      // First row. Partial derivative with respect to the base position state
      // p_k+1 = p_k + v_k deltaT
      int rowStart = offset + errorTranslationIndex;
      CommonOps_DDRM.insert(eye3x3, jacobianToPack, rowStart, offset + errorTranslationIndex);
      CommonOps_DDRM.insert(deltaT, jacobianToPack, rowStart, offset + errorLinearVelocityIndex);

      // Second row. Partial derivative with respect to the base velocity state
      // v_k+1 = v_k + deltaT * (R * a - g)
      rowStart = offset + errorLinearVelocityIndex;
      CommonOps_DDRM.insert(eye3x3, jacobianToPack, rowStart, offset + errorLinearVelocityIndex);

      OdometryTools.toRotationMatrix(stateVariables.orientation, rotationMatrix);
      OdometryTools.toSkewSymmetricMatrix(stateVariables.unbiasedAccel, skewMatrix);
      MatrixTools.multAddBlock(-estimateDT, rotationMatrix, skewMatrix, jacobianToPack, rowStart, offset + errorOrientationIndex);

      if (OdometryKalmanFilter.includeBias)
         MatrixTools.setMatrixBlock(jacobianToPack, rowStart, offset + errorAccelBiasIndex, rotationMatrix, 0, 0, 3, 3, -estimateDT);

      // Third row. Partial derivative with respect to the orientation state
      rowStart = offset + errorOrientationIndex;
      OdometryTools.toSkewSymmetricMatrix(stateVariables.unbiasedGyro, skewMatrix);
      CommonOps_DDRM.insert(eye3x3, jacobianToPack, rowStart, offset + errorOrientationIndex);
      MatrixTools.addMatrixBlock(jacobianToPack, rowStart, offset + errorOrientationIndex, skewMatrix, 0, 0, 3, 3, -estimateDT);
      if (OdometryKalmanFilter.includeBias)
      {
         MatrixTools.setMatrixBlock(jacobianToPack, rowStart, offset + errorGyroBiasIndex, deltaT, 0, 0, 3, 3, -1.0);
      }

      // Fourth row. Partial derivative with respect to the acceleration bias state
      CommonOps_DDRM.insert(eye3x3, jacobianToPack, offset + errorAccelBiasIndex, offset + errorAccelBiasIndex);
      // Fifth row. Partial derivative with respect to the gyro bias state
      CommonOps_DDRM.insert(eye3x3, jacobianToPack, offset + errorGyroBiasIndex, offset + errorGyroBiasIndex);
   }
}
