package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryIndexHelper.*;

class MeasurementModel
{
   private static final DMatrixRMaj eye3x3 = CommonOps_DDRM.identity(3);

   private final StateVariables baseState;
   private final StateVariables footState;
   private final SensedVariables footSensing;
   private final MeasurementVariables predictedMeasurements;
   private final FrameVector3DReadOnly gravityVector;

   // State variables
   public final YoQuaternion footOrientationInBaseFrame;

   public final YoBoolean isInContact;
   private final DoubleProvider contactThreshold;

   // Temp variables
   private final Quaternion footOrientationError = new Quaternion();
   private final Vector3D positionError = new Vector3D();
   private final Vector3D velocityError = new Vector3D();
   private final Vector3D tempVector = new Vector3D();
   private final DMatrixRMaj rotationMatrixTranspose = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj skewMatrix = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj left3x3 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj left = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj right = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj lr = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj lr3x3 = new DMatrixRMaj(3, 3);
   private final Quaternion tempRotation = new Quaternion();

   public MeasurementModel(String prefix,
                           StateVariables baseState,
                           StateVariables footState,
                           SensedVariables footSensing,
                           MeasurementVariables predictedMeasurement,
                           FrameVector3DReadOnly gravityVector,
                           DoubleProvider contactThreshold,
                           YoRegistry registry)
   {
      this.baseState = baseState;
      this.footState = footState;
      this.footSensing = footSensing;
      this.predictedMeasurements = predictedMeasurement;
      this.gravityVector = gravityVector;
      this.contactThreshold = contactThreshold;

      footOrientationInBaseFrame = new YoQuaternion(prefix + "OrientationInBaseFrame", registry);
      isInContact = new YoBoolean(prefix + "IsInContact", registry);
   }

   /**
    * This is equation 35
    */
   public void update(DMatrixRMaj footCovariance)
   {
      isInContact.set(computeIsFootInContact(footCovariance));

      // First row. Update the predicted foot position of the foot relative to the base in the base frame
      predictedMeasurements.relativePosition.sub(footState.translation, baseState.translation);
      baseState.orientation.transform(predictedMeasurements.relativePosition); // FIXME is this right? transforms do more operations than I think is expected

      // Update the predicted foot orientation of the foot relative to the base in the base frame
      QuaternionTools.multiplyConjugateLeft(baseState.orientation, footState.orientation, footOrientationInBaseFrame);

      // Compute the error between the predicted and measured foot orientation
      QuaternionTools.multiplyConjugateLeft(footSensing.orientationMeasurement, footOrientationInBaseFrame, footOrientationError);
      footOrientationError.getRotationVector(predictedMeasurements.relativeOrientationError);

      // Third row. Compute the linear velocity error in the base frame
      velocityError.sub(baseState.linearVelocity, footState.linearVelocity);
      baseState.orientation.inverseTransform(velocityError);

      predictedMeasurements.relativeLinearVelocityError.cross(baseState.unbiasedGyro, footSensing.positionMeasurement);
      predictedMeasurements.relativeLinearVelocityError.scale(-1.0);
      predictedMeasurements.relativeLinearVelocityError.sub(velocityError);
      predictedMeasurements.relativeLinearVelocityError.add(footSensing.linearVelocity);

      if (!isInContact.getBooleanValue())
         return;

      // Fourth row. Set the predicted linear velocity
      predictedMeasurements.contactVelocity.set(footState.linearVelocity);

      // Fifth row. Set the predicted gravity vector
      footState.orientation.transform(gravityVector, predictedMeasurements.accelMeasure);
      if (OdometryKalmanFilter.includeBias)
         predictedMeasurements.accelMeasure.add(footState.accelBias);
   }

   private final DMatrixRMaj vector = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj tempDVector = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj tempScalar = new DMatrixRMaj(3, 1);

   private boolean computeIsFootInContact(DMatrixRMaj footCovariance)
   {
      footState.translation.get(vector);

      CommonOps_DDRM.mult(footCovariance, vector, tempDVector);
      CommonOps_DDRM.multTransA(vector, tempDVector, tempScalar);

      // tests the mahalonobis distance of the foot velocity being below a certain threshold.
      return tempScalar.get(0, 0) < contactThreshold.getValue();
   }

   public boolean getIsFootInContact()
   {
      return isInContact.getBooleanValue();
   }

   /**
    * This is equation 41
    */
   public void computeBaseMeasurementJacobian(int rowOffset, DMatrixRMaj jacobianToPack)
   {
      OdometryTools.toRotationMatrix(baseState.orientation, rotationMatrixTranspose);
      CommonOps_DDRM.transpose(rotationMatrixTranspose);


      positionError.sub(footState.translation, baseState.translation);
      velocityError.sub(footState.linearVelocity, baseState.linearVelocity);
      baseState.orientation.inverseTransform(positionError);
      baseState.orientation.inverseTransform(velocityError);

      OdometryTools.toSkewSymmetricMatrix(positionError, skewMatrix);

      // First row. Partial of relative foot position w.r.t. the base state
      int row = rowOffset + measurementRelativeTranslationIndex;
      MatrixTools.setMatrixBlock(jacobianToPack, row, errorTranslationIndex, rotationMatrixTranspose, 0, 0, 3, 3, -1.0);
      CommonOps_DDRM.insert(skewMatrix, jacobianToPack, row, errorOrientationIndex);

      // Second row. Partial of relative foot orientation error w.r.t. the base state
      row = rowOffset + measurementRelativeOrientationErrorIndex;
      QuaternionTools.multiplyConjugateLeft(footState.orientation, baseState.orientation, tempRotation);
      OdometryTools.lOperator(tempRotation, left);
      OdometryTools.rOperator(footSensing.orientationMeasurement, right);
      CommonOps_DDRM.mult(left, right, lr);
      MatrixTools.setMatrixBlock(jacobianToPack, row, errorOrientationIndex, lr, 1, 1, 3, 3, -1.0);

      // Third row. Partial of relative foot velocity error w.r.t. the base state
      row = rowOffset + measurementRelativeVelocityIndex;
      OdometryTools.toSkewSymmetricMatrix(velocityError, skewMatrix);

      MatrixTools.setMatrixBlock(jacobianToPack, row, errorLinearVelocityIndex, rotationMatrixTranspose, 0, 0, 3, 3, -1.0);
      CommonOps_DDRM.insert(skewMatrix, jacobianToPack, row, errorOrientationIndex);

      if (OdometryKalmanFilter.includeBias)
      {
         OdometryTools.toSkewSymmetricMatrix(footSensing.positionMeasurement, skewMatrix);
         MatrixTools.setMatrixBlock(jacobianToPack, row, errorGyroBiasIndex, skewMatrix, 0, 0, 3, 3, -1.0);
      }
   }

   /**
    * This is equation 42
    */
   public void computeFootMeasurementJacobian(int rowOffset, int colOffset, DMatrixRMaj jacobianToPack)
   {
      OdometryTools.toRotationMatrix(baseState.orientation, rotationMatrixTranspose);
      CommonOps_DDRM.transpose(rotationMatrixTranspose);

      positionError.sub(footState.translation, baseState.translation);
      velocityError.sub(footState.linearVelocity, baseState.linearVelocity);
      baseState.orientation.inverseTransform(positionError);
      baseState.orientation.inverseTransform(velocityError);

      OdometryTools.toSkewSymmetricMatrix(positionError, skewMatrix);

      // First row. Partial of relative contact position w.r.t. foot state
      int row = rowOffset + measurementRelativeTranslationIndex;
      CommonOps_DDRM.insert(rotationMatrixTranspose, jacobianToPack, row, colOffset + errorTranslationIndex);

      // Second row. Partial of relative orientation error w.r.t. foot state
      row = rowOffset + measurementRelativeOrientationErrorIndex;
      tempRotation.set(footSensing.orientationMeasurement);
      tempRotation.multiplyConjugateOther(baseState.orientation);
      tempRotation.multiply(footState.orientation);
      OdometryTools.l3Operator(tempRotation, left3x3);
      CommonOps_DDRM.insert(left3x3, jacobianToPack, row, colOffset + errorOrientationIndex);

      // Third row. Partial of relative velocity error w.r.t. foot state
      row = rowOffset + measurementRelativeVelocityIndex;
      CommonOps_DDRM.insert(rotationMatrixTranspose, jacobianToPack, row, colOffset + errorLinearVelocityIndex);

      if (!isInContact.getBooleanValue())
         return;

      // Fourth row. Partial of contact velocity w.r.t. foot state
      // The contact velocity is directly the foot velocity when it's in contact.
      row = rowOffset + measurementContactVelocityIndex;
      CommonOps_DDRM.insert(eye3x3, jacobianToPack, row, colOffset + errorLinearVelocityIndex);

      // Fifth row. Partial of contact acceleration w.r.t. foot state
      row = rowOffset + measurementAccelIndex;
      tempVector.set(gravityVector);
      footState.orientation.inverseTransform(tempVector);
      OdometryTools.toSkewSymmetricMatrix(tempVector, skewMatrix);
      CommonOps_DDRM.insert(skewMatrix, jacobianToPack, row, colOffset + errorOrientationIndex);

      if (OdometryKalmanFilter.includeBias)
         CommonOps_DDRM.insert(eye3x3, jacobianToPack, row, colOffset + errorAccelBiasIndex);
   }
}
