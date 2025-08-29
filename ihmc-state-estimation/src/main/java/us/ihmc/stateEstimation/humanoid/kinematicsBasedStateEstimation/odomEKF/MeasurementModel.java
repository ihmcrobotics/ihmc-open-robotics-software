package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryKalmanFilter.MeasuredVariables;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryKalmanFilter.StateVariables;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryIndexHelper.*;

class MeasurementModel
{
   private static final DMatrixRMaj eye3x3 = CommonOps_DDRM.identity(3);

   private final StateVariables baseState;
   private final StateVariables footState;
   private final MeasuredVariables footMeasurements;
   private final FrameVector3DReadOnly gravityVector;

   // State variables
   public final YoVector3D footPredictedRelativePosition;
   public final YoVector3D footPredictedRelativeOrientationError;
   public final YoVector3D footPredictedLinearVelocityError;
   public final YoVector3D footPredictedLinearVelocity;
   public final YoVector3D footPredictedGravityVector;
   public final YoQuaternion footOrientationInBaseFrame;

   public final YoBoolean isInContact;
   private final DoubleProvider contactThreshold;

   // Temp variables
   private final Quaternion footOrientationError = new Quaternion();
   private final Vector3D velocityError = new Vector3D();

   public MeasurementModel(String prefix,
                           StateVariables baseState,
                           StateVariables footState,
                           MeasuredVariables footMeasurements,
                           FrameVector3DReadOnly gravityVector,
                           DoubleProvider contactThreshold,
                           YoRegistry registry)
   {
      this.baseState = baseState;
      this.footState = footState;
      this.footMeasurements = footMeasurements;
      this.gravityVector = gravityVector;
      this.contactThreshold = contactThreshold;

      footPredictedRelativePosition = new YoVector3D(prefix + "PredictedRelativePosition", registry);
      footPredictedRelativeOrientationError = new YoVector3D(prefix + "PredictedRelativeOrientationError", registry);
      footPredictedLinearVelocityError = new YoVector3D(prefix + "PredictedLinearVelocityError", registry);
      footPredictedLinearVelocity = new YoVector3D(prefix + "PredictedLinearVelocity", registry);
      footPredictedGravityVector = new YoVector3D(prefix + "PredictedAccelMeasure", registry);
      footOrientationInBaseFrame = new YoQuaternion(prefix + "OrientationInBaseFrame", registry);
      isInContact = new YoBoolean(prefix + "IsInContact", registry);
   }

   /**
    * This is equation 35
    */
   public void update(DMatrixRMaj footCovariance)
   {
      isInContact.set(isFootInContact(footCovariance));

      // First row. Update the predicted foot position of the foot relative to the base in the base frame
      footPredictedRelativePosition.sub(footState.translation, baseState.translation);
      baseState.orientation.transform(footPredictedRelativePosition); // FIXME is this right? transforms do more operations than I think is expected

      // Update the predicted foot orientation of the foot relative to the base in the base frame
      QuaternionTools.multiplyConjugateLeft(baseState.orientation, footState.orientation, footOrientationInBaseFrame);

      // Compute the error between the predicted and measured foot orientation
      QuaternionTools.multiplyConjugateLeft(footMeasurements.orientationMeasurement, footOrientationInBaseFrame, footOrientationError);
      footOrientationError.getRotationVector(footPredictedRelativeOrientationError);

      // Third row. Compute the linear velocity error in the base frame
      velocityError.sub(baseState.linearVelocity, footState.linearVelocity);
      baseState.orientation.inverseTransform(velocityError);

      footPredictedLinearVelocityError.cross(baseState.unbiasedGyro, footMeasurements.positionMeasurement);
      footPredictedLinearVelocityError.scale(-1.0);
      footPredictedLinearVelocityError.sub(velocityError);
      footPredictedLinearVelocityError.add(footMeasurements.linearVelocity);

      if (!isInContact.getBooleanValue())
         return;

      // Fourth row. Set the predicted linear velocity
      footPredictedLinearVelocity.set(footState.linearVelocity);

      // Fifth row. Set the predicted gravity vector
      footState.orientation.transform(gravityVector, footPredictedGravityVector);
      if (OdometryKalmanFilter.includeBias)
         footPredictedGravityVector.add(footState.accelBias);
   }

   private final DMatrixRMaj vector = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj tempVector = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj tempScalar = new DMatrixRMaj(3, 1);

   private boolean isFootInContact(DMatrixRMaj footCovariance)
   {
      footState.translation.get(vector);

      CommonOps_DDRM.mult(footCovariance, vector, tempVector);
      CommonOps_DDRM.multTransA(vector, tempVector, tempScalar);

      // tests the mahalonobis distance of the foot velocity being below a certain threshold.
      return tempScalar.get(0, 0) < contactThreshold.getValue();
   }

   public void get(int start, DMatrixRMaj measurementToPack)
   {
      footPredictedRelativePosition.get(start + measurementRelativeTranslationIndex, measurementToPack);
      footPredictedRelativeOrientationError.get(start + measurementRelativeOrientationErrorIndex, measurementToPack);
      footPredictedLinearVelocityError.get(start + measurementRelativeVelocityIndex, measurementToPack);

      if (isInContact.getBooleanValue())
         return;

      footPredictedLinearVelocity.get(start + measurementContactVelocityIndex, measurementToPack);
      footPredictedGravityVector.get(start + measurementAccelIndex, measurementToPack);
   }

   /**
    * This is equation 41
    */
   public void computeBaseMeasurementJacobian(StateVariables baseState,
                                               StateVariables footState,
                                               MeasuredVariables footMeasurements,
                                               int rowOffset,
                                               DMatrixRMaj jacobianToPack)
   {
      // FIXME garbage
      DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
      DMatrixRMaj rotationMatrixTranspose = new DMatrixRMaj(3, 3);
      OdometryTools.toRotationMatrix(baseState.orientation, rotationMatrix);
      CommonOps_DDRM.transpose(rotationMatrix, rotationMatrixTranspose);

      Vector3D footPositionError = new Vector3D();
      Vector3D footVelocityError = new Vector3D();
      footPositionError.sub(footState.translation, baseState.translation);
      footVelocityError.sub(footState.linearVelocity, baseState.linearVelocity);
      baseState.orientation.inverseTransform(footPositionError);
      baseState.orientation.inverseTransform(footVelocityError);

      DMatrixRMaj skewMatrix = new DMatrixRMaj(3, 3);
      OdometryTools.toSkewSymmetricMatrix(footPositionError, skewMatrix);

      // First row
      int row = rowOffset + measurementRelativeTranslationIndex;
      MatrixTools.setMatrixBlock(jacobianToPack, row, stateTranslationIndex, rotationMatrixTranspose, 0, 0, 3, 3, -1.0);
      CommonOps_DDRM.insert(skewMatrix, jacobianToPack, row, stateOrientationIndex);

      // Second row
      row = rowOffset + measurementRelativeOrientationErrorIndex;
      DMatrixRMaj left = new DMatrixRMaj(3, 3);
      DMatrixRMaj right = new DMatrixRMaj(3, 3);
      Quaternion orientation = new Quaternion(footState.orientation);
      orientation.multiplyConjugateThis(baseState.orientation);
      OdometryKalmanFilter.l3Operator(orientation, left);
      OdometryKalmanFilter.r3Operator(footMeasurements.orientationMeasurement, right);
      MatrixTools.multAddBlock(-1.0, left, right, jacobianToPack, row, stateOrientationIndex);

      // Third row
      row = rowOffset + measurementRelativeVelocityIndex;
      OdometryTools.toSkewSymmetricMatrix(footVelocityError, skewMatrix);

      MatrixTools.setMatrixBlock(jacobianToPack, row, stateLinearVelocityIndex, rotationMatrixTranspose, 0, 0, 3, 3, -1.0);
      CommonOps_DDRM.insert(skewMatrix, jacobianToPack, row, stateOrientationIndex);

      if (OdometryKalmanFilter.includeBias)
      {
         OdometryTools.toSkewSymmetricMatrix(footMeasurements.positionMeasurement, skewMatrix);
         MatrixTools.setMatrixBlock(jacobianToPack, row, stateGyroBiasIndex, skewMatrix, 0, 0, 3, 3, -1.0);
      }
   }

   /**
    * This is equation 42
    */
   public void computeFootMeasurementJacobian(StateVariables baseState,
                                               StateVariables footState,
                                               MeasuredVariables footMeasurements,
                                               FrameVector3DReadOnly gravityVector,
                                               int rowOffset,
                                               int colOffset,
                                               DMatrixRMaj jacobianToPack)
   {
      // FIXME garbage
      DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
      OdometryTools.toRotationMatrix(baseState.orientation, rotationMatrix);
      CommonOps_DDRM.transpose(rotationMatrix);

      Vector3D footPositionError = new Vector3D();
      Vector3D footVelocityError = new Vector3D();
      footPositionError.sub(footState.translation, baseState.translation);
      footVelocityError.sub(footState.linearVelocity, baseState.linearVelocity);
      baseState.orientation.inverseTransform(footPositionError);
      baseState.orientation.inverseTransform(footVelocityError);

      DMatrixRMaj skewMatrix = new DMatrixRMaj(3, 3);
      OdometryTools.toSkewSymmetricMatrix(footPositionError, skewMatrix);

      // First row
      CommonOps_DDRM.insert(rotationMatrix, jacobianToPack, rowOffset, colOffset);

      // Second row
      rowOffset += 3;
      DMatrixRMaj left = new DMatrixRMaj(3, 3);
      Quaternion orientation = new Quaternion(footMeasurements.orientationMeasurement);
      orientation.multiplyConjugateOther(baseState.orientation);
      orientation.multiply(footState.orientation);
      OdometryKalmanFilter.l3Operator(orientation, left);
      CommonOps_DDRM.insert(left, jacobianToPack, rowOffset, colOffset + 6);

      // Third row
      rowOffset += 3;
      CommonOps_DDRM.insert(rotationMatrix, jacobianToPack, rowOffset, colOffset + 3);

      if (isInContact.getBooleanValue())
         return;

      // Fourth row
      rowOffset += 3;
      CommonOps_DDRM.insert(eye3x3, jacobianToPack, rowOffset, colOffset + 3);

      // Fifth row
      rowOffset += 3;
      Vector3D gravity = new Vector3D(gravityVector);
      footState.orientation.inverseTransform(gravity);
      OdometryTools.toSkewSymmetricMatrix(gravityVector, skewMatrix);
      CommonOps_DDRM.insert(skewMatrix, jacobianToPack, rowOffset, colOffset + 6);

      if (OdometryKalmanFilter.includeBias)
         CommonOps_DDRM.insert(eye3x3, jacobianToPack, rowOffset, colOffset + 9);
   }
}
