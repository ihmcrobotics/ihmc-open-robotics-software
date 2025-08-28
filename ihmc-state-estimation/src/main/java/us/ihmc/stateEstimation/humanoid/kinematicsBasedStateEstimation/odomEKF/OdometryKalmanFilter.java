package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.parameterEstimation.ExtendedKalmanFilter;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

/**
 * The process model in this class is defined as
 */
public class OdometryKalmanFilter extends ExtendedKalmanFilter
{
   // Constants and providers
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final IMUSensorReadOnly baseIMU;
   private final List<? extends IMUSensorReadOnly> feetIMUs;
   private final IMUBiasProvider imuBiasProvider;

   private final double estimatorDT;
   private final FrameVector3DReadOnly gravityVector;

   private final YoDouble translationCovariance = new YoDouble("translationCovariance", registry);
   private final YoDouble velocityCovariance = new YoDouble("velocityCovariance", registry);
   private final YoDouble orientationCovariance = new YoDouble("orientationCovariance", registry);
   private final YoDouble biasCovariance = new YoDouble("biasCovariance", registry);

   private final YoDouble measuredTranslationCovariance = new YoDouble("measuredTranslationCovariance", registry);
   private final YoDouble measuredOrientationCovariance = new YoDouble("measuredOrientationCovariance", registry);
   private final YoDouble velocityErrorCovariance = new YoDouble("velocityErrorCovariance", registry);
   private final YoDouble contactVelocityCovariance = new YoDouble("contactVelocityCovariance", registry);
   private final YoDouble contactOrientationCovariance = new YoDouble("contactOrientationCovariance", registry);

   // Internal variables for state holding
   private final YoFrameVector3D yoBaseAngularVelocityMeasurement;

   private final YoFrameVector3D yoRootJointAngularVelocityMeasurementInWorld;
   private final YoFrameVector3D yoRootJointAngularVelocityMeasurement;

   // Outputs
   private final DMatrixRMaj predictedState;
   private final DMatrixRMaj predictedMeasurement;
   private final DMatrixRMaj observationVector;

   private final DMatrixRMaj AMatrix;
   private final DMatrixRMaj CMatrix;

   private final MeasuredVariables baseMeasurement;
   private final List<MeasuredVariables> feetMeasurements = new ArrayList<>();

   private final StateVariables baseProcessState;
   private final StateVariables basePredictedState;
   private final List<StateVariables> feetProcessState = new ArrayList<>();
   private final List<StateVariables> feetPredictedState = new ArrayList<>();

   private final List<ProcessModel> processModels = new ArrayList<>();
   private final List<MeasurementModel> measurementModels = new ArrayList<>();
   private final List<Observation> observations = new ArrayList<>();

   private final YoFrameQuaternion filteredBaseOrientation;
   private final YoFramePoint3D filteredBaseTranslation;
   private final YoFrameVector3D filteredBaseLinearVelocity;

   private final YoFramePose3D estimatedRootJointPose;
   private final YoFrameVector3D estimatedRootJointLinearVelocity;
   private final YoFrameVector3D estimatedRootJointAngularVelocity;

   private final RigidBodyTransformReadOnly transformToRootJoint;

   // Temporary variables
   private final DMatrixRMaj footCovariance = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj processCovariance;
   private final DMatrixRMaj measurementCovariance;


   private static final DMatrixRMaj eye3x3 = CommonOps_DDRM.identity(3);

   public OdometryKalmanFilter(IMUSensorReadOnly baseIMU,
                               List<? extends IMUSensorReadOnly> feetIMUs,
                               IMUBiasProvider baseImuBiasProvider,
                               BooleanProvider cancelGravityFromAccelerationMeasurement,
                               double estimatorDT,
                               double gravitationalAcceleration,
                               YoRegistry parentRegistry)
   {
      super(OdometryIndexHelper.getStateSizePerLink() * (1 + feetIMUs.size()), OdometryIndexHelper.getObservationSizePerLink() * feetIMUs.size());

      // INitialize covariance
      processCovariance = new DMatrixRMaj(getStateSize(), getStateSize());
      measurementCovariance = new DMatrixRMaj(getMeasurementSize(), getMeasurementSize());
      MatrixTools.setDiagonal(processCovariance, 1e-2);
      MatrixTools.setDiagonal(measurementCovariance, 1e-3);
      setProcessCovariance(processCovariance);
      setMeasurementCovariance(measurementCovariance);

      translationCovariance.set(1e-4);
      velocityCovariance.set(1e-3);
      orientationCovariance.set(1e-4);
      biasCovariance.set(1e-6);

      measuredTranslationCovariance.set(1e-5);
      measuredOrientationCovariance.set(1e-5);
      velocityErrorCovariance.set(1e-4);
      contactVelocityCovariance.set(1e-5);
      contactOrientationCovariance.set(1e-6);

      this.baseIMU = baseIMU;
      this.feetIMUs = feetIMUs;
      this.imuBiasProvider = baseImuBiasProvider;
      this.estimatorDT = estimatorDT;

      predictedState = new DMatrixRMaj(getStateSize(), 1);
      predictedMeasurement = new DMatrixRMaj(getMeasurementSize(), 1);
      AMatrix = new DMatrixRMaj(getStateSize(), getStateSize());
      CMatrix = new DMatrixRMaj(getMeasurementSize(), getStateSize());
      observationVector = new DMatrixRMaj(OdometryIndexHelper.getObservationSizePerLink() * feetIMUs.size(), 1);

      gravityVector = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(() -> worldFrame, new Vector3D(0, 0, -Math.abs(gravitationalAcceleration)));

      baseMeasurement = new MeasuredVariables("base", baseIMU, imuBiasProvider, registry);
      baseProcessState = new StateVariables("baseProcessState");
      basePredictedState = new StateVariables("basePredicted");
      processModels.add(new ProcessModel(baseMeasurement,
                                         baseProcessState,
                                         basePredictedState,
                                         cancelGravityFromAccelerationMeasurement,
                                         gravityVector,
                                         estimatorDT));
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         MeasuredVariables footMeasurements = new MeasuredVariables("foot" + i, feetIMUs.get(i), imuBiasProvider, registry);
         StateVariables footProcessState = new StateVariables("foot" + i + "ProcessState");
         StateVariables footPredictedState = new StateVariables("foot" + i + "PredictedState");
         feetMeasurements.add(footMeasurements);
         feetProcessState.add(footProcessState);
         feetPredictedState.add(footPredictedState);

         observations.add(new Observation("foot" + i + "Observation", baseIMU, feetIMUs.get(i), footMeasurements, registry));

         processModels.add(new ProcessModel(footMeasurements,
                                            footProcessState,
                                            footPredictedState,
                                            cancelGravityFromAccelerationMeasurement,
                                            gravityVector,
                                            estimatorDT));
         measurementModels.add(new MeasurementModel("foot" + i,
                                                    basePredictedState,
                                                    baseMeasurement,
                                                    footPredictedState,
                                                    baseIMU,
                                                    feetIMUs.get(i),
                                                    gravityVector,
                                                    registry));
      }

      filteredBaseOrientation = new YoFrameQuaternion("filteredBaseOrientation", worldFrame, registry);
      filteredBaseTranslation = new YoFramePoint3D("filteredBaseTranslation", worldFrame, registry);
      filteredBaseLinearVelocity = new YoFrameVector3D("filteredBaseLinearVelocity", worldFrame, registry);

      estimatedRootJointPose = new YoFramePose3D("estimatedRootJointPose", worldFrame, registry);
      estimatedRootJointLinearVelocity = new YoFrameVector3D("estimatedRootJointLinearVelocity", worldFrame, registry);
      estimatedRootJointAngularVelocity = new YoFrameVector3D("estimatedRootJointAngularVelocity", worldFrame, registry);

      transformToRootJoint = baseIMU.getMeasurementFrame().getTransformToDesiredFrame(baseIMU.getMeasurementLink().getParentJoint().getFrameAfterJoint());

      yoBaseAngularVelocityMeasurement = new YoFrameVector3D("baseImuLinearAccelerationInWorld", baseIMU.getMeasurementFrame(), registry);
      yoRootJointAngularVelocityMeasurement = new YoFrameVector3D("rootJointImuLinearAcceleration", baseIMU.getMeasurementFrame(), registry);
      yoRootJointAngularVelocityMeasurementInWorld = new YoFrameVector3D("rootJointImuLinearAccelerationInWorld", worldFrame, registry);

      parentRegistry.addChild(registry);
   }

   private static class ProcessModel
   {
      // State providers
      private final MeasuredVariables measuredVariables;
      private final StateVariables currentState;
      private final StateVariables predictedState;
      private final BooleanProvider cancelGravityFromAccelerationMeasurement;
      private final FrameVector3DReadOnly gravityVector;
      private final double estimatorDt;

      // Temp variables
      private final Vector3D unbiasedAcceleration = new Vector3D();
      private final Vector3D unbiasedAngularVelocity = new Vector3D();
      private final Vector3D integratedVelocity = new Vector3D();
      private final Quaternion integratedRotation = new Quaternion();

      public ProcessModel(MeasuredVariables measuredVariables,
                          StateVariables processState,
                          StateVariables predictedState,
                          BooleanProvider cancelGravityFromAccelerationMeasurement,
                          FrameVector3DReadOnly gravityVector,
                          double estimatorDt)
      {
         this.measuredVariables = measuredVariables;
         this.currentState = processState;
         this.predictedState = predictedState;
         this.cancelGravityFromAccelerationMeasurement = cancelGravityFromAccelerationMeasurement;
         this.gravityVector = gravityVector;
         this.estimatorDt = estimatorDt;
      }

      public void update()
      {
         unbiasedAcceleration.sub(measuredVariables.linearAccelerationMeasurement, currentState.accelBias);
         unbiasedAngularVelocity.sub(measuredVariables.gyroMeasurement, currentState.gyroBias);
         // transform the acceleration to world
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
         integratedVelocity.setAndScale(estimatorDt, unbiasedAngularVelocity);
         OdometryIndexHelper.toQuaternionFromRotationVector(integratedVelocity, integratedRotation);
         predictedState.orientation.multiply(currentState.orientation, integratedRotation);

         // Propagate the bias
         predictedState.accelBias.set(currentState.accelBias);
         predictedState.gyroBias.set(currentState.gyroBias);
      }
   }

   private static class Observation
   {
      // State providers
      private final IMUSensorReadOnly baseIMU;
      private final IMUSensorReadOnly footIMU;
      private final MeasuredVariables footMeasurements;

      // State variables
      private final YoVector3D footPositionFromKinematics;

      // Temp variables
      private final FramePoint3D footPosition = new FramePoint3D();

      public Observation(String prefix, IMUSensorReadOnly baseIMU, IMUSensorReadOnly footIMU, MeasuredVariables footMeasurements, YoRegistry registry)
      {
         this.baseIMU = baseIMU;
         this.footIMU = footIMU;
         this.footMeasurements = footMeasurements;

         footPositionFromKinematics = new YoVector3D(prefix + "PositionFromKinematics", registry);
      }

      public void update()
      {
         footPosition.setToZero(footIMU.getMeasurementFrame());
         footPosition.changeFrame(baseIMU.getMeasurementFrame());

         footPositionFromKinematics.set(footPosition);
      }

      public void get(int startRow, DMatrixRMaj observation)
      {
         footPositionFromKinematics.get(startRow, observation);
         footMeasurements.linearAccelerationMeasurement.get(startRow + 6, observation);
      }
   }

   private static class MeasurementModel
   {
      private final StateVariables baseState;
      private final MeasuredVariables baseMeasurement;
      private final StateVariables footState;
      private final IMUSensorReadOnly baseIMU;
      private final IMUSensorReadOnly footIMU;
      private final FrameVector3DReadOnly gravityVector;

      // State variables
      public final YoVector3D footPredictedRelativePosition;
      public final YoVector3D footPredictedRelativeOrientation;
      public final YoVector3D footPredictedLinearVelocityError;
      public final YoVector3D footPredictedLinearVelocity;
      public final YoVector3D footPredictedGravityVector;

      public final YoBoolean isInContact;

      // Temp variables
      private final FramePose3D footPose = new FramePose3D();
      private final Quaternion footOrientationError = new Quaternion();
      private final Vector3D velocityError = new Vector3D();
      private final Vector3D unbiasedAngularVelocity = new Vector3D();

      public MeasurementModel(String prefix,
                              StateVariables baseState,
                              MeasuredVariables baseMeasurement,
                              StateVariables footState,
                              IMUSensorReadOnly baseIMU,
                              IMUSensorReadOnly footIMU,
                              FrameVector3DReadOnly gravityVector,
                              YoRegistry registry)
      {
         this.baseState = baseState;
         this.baseMeasurement = baseMeasurement;
         this.footState = footState;
         this.baseIMU = baseIMU;
         this.footIMU = footIMU;
         this.gravityVector = gravityVector;

         footPredictedRelativePosition = new YoVector3D(prefix + "PredictedRelativePosition", registry);
         footPredictedRelativeOrientation = new YoVector3D(prefix + "PredictedRelativeOrientation", registry);
         footPredictedLinearVelocityError = new YoVector3D(prefix + "PredictedLinearVelocityError", registry);
         footPredictedLinearVelocity = new YoVector3D(prefix + "PredictedLinearVelocity", registry);
         footPredictedGravityVector = new YoVector3D(prefix + "PredictedGravityVector", registry);
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
         baseState.orientation.transform(footPredictedRelativePosition);

         // Second row. get the foot pose relative to the base link, which should be entirely based on kinematics
         footPose.setToZero(footIMU.getMeasurementFrame());
         footPose.changeFrame(baseIMU.getMeasurementFrame());

         // Update the predicted foot orientation of the foot relative to the base in the base frame
         footOrientationError.set(footPose.getOrientation());
         footOrientationError.multiplyConjugateBoth(baseState.orientation);
         footOrientationError.multiply(footState.orientation);
         OdometryIndexHelper.logMap(footOrientationError, footPredictedRelativeOrientation);

         // Compute the linear velocity error in the base frame
         unbiasedAngularVelocity.sub(baseMeasurement.gyroMeasurement, baseState.gyroBias);
         velocityError.sub(baseState.linearVelocity, footState.linearVelocity);
         baseState.orientation.inverseTransform(velocityError);

         // FIXME this likely changes based on the contact conditions
         footPredictedLinearVelocityError.cross(unbiasedAngularVelocity, footPose.getPosition());
         footPredictedLinearVelocityError.scale(-1.0);
         footPredictedLinearVelocityError.sub(velocityError);

         // FIXME get the estimated foot linear velocity from the kinematics, and add it to here. That is, get the one using the MovingReferenceFrame, relative to the base IMU.

         if (isInContact.getBooleanValue())
            return;

         // Fourth row. Set the predicted linear velocity
         footPredictedLinearVelocity.set(footState.linearVelocity);

         // Fifth row. Set the predicted gravity vector
         footState.orientation.transform(gravityVector, footPredictedGravityVector);
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
         return tempScalar.get(0, 0) < 0.2; // FIXME extract magic number
      }

      public void get(int start, DMatrixRMaj measurementToPack)
      {
         footPredictedRelativePosition.get(start, measurementToPack);
         footPredictedRelativeOrientation.get(start + 3, measurementToPack);
         footPredictedLinearVelocityError.get(start + 6, measurementToPack);
         if (isInContact.getBooleanValue())
            return;

         footPredictedLinearVelocity.get(start + 9, measurementToPack);
         footPredictedGravityVector.get(start + 12, measurementToPack);
      }
   }

   private static class MeasuredVariables
   {
      // State providers
      private final IMUSensorReadOnly imu;
      private final IMUBiasProvider imuBiasProvider;

      // State recorders
      public final YoFrameVector3D gyroMeasurementInWorld;
      public final YoFrameVector3D gyroMeasurement;
      public final YoFrameVector3D linearAccelerationMeasurementInWorld;
      public final YoFrameVector3D linearAccelerationMeasurement;

      // Temp variables
      private final FrameVector3D linearAcceleration = new FrameVector3D();
      private final FrameVector3D angularVelocity = new FrameVector3D();

      public MeasuredVariables(String prefix, IMUSensorReadOnly imu, IMUBiasProvider imuBiasProvider, YoRegistry registry)
      {
         this.imu = imu;
         this.imuBiasProvider = imuBiasProvider;

         gyroMeasurementInWorld = new YoFrameVector3D(prefix + "GyroMeasurementInWorld", worldFrame, registry);
         gyroMeasurement = new YoFrameVector3D(prefix + "GyroMeasurement", imu.getMeasurementFrame(), registry);
         linearAccelerationMeasurementInWorld = new YoFrameVector3D(prefix + "LinearAccelerationMeasurementInWorld", worldFrame, registry);
         linearAccelerationMeasurement = new YoFrameVector3D(prefix + "LinearAccelerationMeasurement", imu.getMeasurementFrame(), registry);
      }

      public void update()
      {
         // Update gyro measure
         FrameVector3DReadOnly gyroBiasInput = imuBiasProvider.getAngularVelocityBiasInIMUFrame(imu);
         Vector3DReadOnly gyroRawInput = imu.getAngularVelocityMeasurement();

         angularVelocity.setReferenceFrame(imu.getMeasurementFrame());
         angularVelocity.sub(gyroRawInput, gyroBiasInput);

         gyroMeasurementInWorld.setMatchingFrame(angularVelocity);
         gyroMeasurement.setMatchingFrame(angularVelocity);

         // Update the accelerometer measure
         FrameVector3DReadOnly accelBiasInput = imuBiasProvider.getLinearAccelerationBiasInIMUFrame(imu);
         Vector3DReadOnly accelRawInput = imu.getLinearAccelerationMeasurement();

         linearAcceleration.setReferenceFrame(imu.getMeasurementFrame());
         linearAcceleration.sub(accelRawInput, accelBiasInput);

         linearAccelerationMeasurementInWorld.setMatchingFrame(linearAcceleration);
         linearAccelerationMeasurement.setMatchingFrame(linearAcceleration);
      }
   }

   private class StateVariables
   {
      public final YoFramePoint3D translation;
      public final YoFrameVector3D linearVelocity;
      public final YoFrameQuaternion orientation;
      public final YoFrameVector3D accelBias;
      public final YoFrameVector3D gyroBias;

      public StateVariables(String prefix)
      {
         translation = new YoFramePoint3D(prefix + "Translation", worldFrame, registry);
         linearVelocity = new YoFrameVector3D(prefix + "LinearVelocity", worldFrame, registry);
         orientation = new YoFrameQuaternion(prefix + "Orientation", worldFrame, registry);
         accelBias = new YoFrameVector3D(prefix + "AccelBias", worldFrame, registry);
         gyroBias = new YoFrameVector3D(prefix + "GyroBias", worldFrame, registry);
      }

      public void set(int start, DMatrixRMaj state)
      {
         translation.set(start + OdometryIndexHelper.getStatePositionIndex(), state);
         linearVelocity.set(start + OdometryIndexHelper.getStateVelocityIndex(), state);
         orientation.set(start + OdometryIndexHelper.getStateOrientationIndex(), state);
         accelBias.set(start + OdometryIndexHelper.getStateAccelerationBiasIndex(), state);
         gyroBias.set(start + OdometryIndexHelper.getStateGyroBiasIndex(), state);
      }

      public void get(int start, DMatrixRMaj stateToPack)
      {
         translation.get(start + OdometryIndexHelper.getStatePositionIndex(), stateToPack);
         linearVelocity.get(start + OdometryIndexHelper.getStateVelocityIndex(), stateToPack);
         orientation.get(start + OdometryIndexHelper.getStateOrientationIndex(), stateToPack);
         accelBias.get(start + OdometryIndexHelper.getStateAccelerationBiasIndex(), stateToPack);
         gyroBias.get(start + OdometryIndexHelper.getStateGyroBiasIndex(), stateToPack);
      }
   }

   public void compute()
   {
      updateCovariance();

      baseMeasurement.update();
      for (int i = 0; i < feetMeasurements.size(); i++)
         feetMeasurements.get(i).update();

      for (int i = 0; i < observations.size(); i++)
      {
         observations.get(i).update();
         observations.get(i).get(i * OdometryIndexHelper.getObservationSizePerLink(), observationVector);
      }
      updateRootJointTwistAngularPart(); // TODO maybe do this after we update everything based on the estimate?

      DMatrixRMaj stateEstimate = calculateEstimate(observationVector);

      filteredBaseTranslation.set(stateEstimate);
      filteredBaseLinearVelocity.set(3, stateEstimate);
      filteredBaseOrientation.set(6, stateEstimate);

      // Transform the pose of the IMU in the world frame to the pose of the root joint in the world frame
      tempPose.set(filteredBaseOrientation, filteredBaseTranslation);
      tempPose.appendTransform(transformToRootJoint);
      estimatedRootJointPose.set(tempPose);
      estimatedRootJointAngularVelocity.set(yoRootJointAngularVelocityMeasurementInWorld);

      updateRootJointTwistLinearPart(filteredBaseLinearVelocity, estimatedRootJointAngularVelocity, estimatedRootJointLinearVelocity);
   }

   private void updateCovariance()
   {
      for (int i = 0; i < feetIMUs.size() + 1; i++)
      {
         int index = i * OdometryIndexHelper.getStateSizePerLink();
         processCovariance.set(index, index++, translationCovariance.getValue());
         processCovariance.set(index, index++, translationCovariance.getValue());
         processCovariance.set(index, index++, translationCovariance.getValue());
         processCovariance.set(index, index++, velocityCovariance.getValue());
         processCovariance.set(index, index++, velocityCovariance.getValue());
         processCovariance.set(index, index++, velocityCovariance.getValue());
         processCovariance.set(index, index++, orientationCovariance.getValue());
         processCovariance.set(index, index++, orientationCovariance.getValue());
         processCovariance.set(index, index++, orientationCovariance.getValue());
         processCovariance.set(index, index++, orientationCovariance.getValue());
         processCovariance.set(index, index++, biasCovariance.getValue());
         processCovariance.set(index, index++, biasCovariance.getValue());
         processCovariance.set(index, index++, biasCovariance.getValue());
         processCovariance.set(index, index++, biasCovariance.getValue());
         processCovariance.set(index, index++, biasCovariance.getValue());
         processCovariance.set(index, index, biasCovariance.getValue());
      }

      for (int i = 0; i < feetIMUs.size(); i++)
      {
         int index = i * OdometryIndexHelper.getObservationSizePerLink();
         measurementCovariance.set(index, index++, measuredTranslationCovariance.getValue());
         measurementCovariance.set(index, index++, measuredTranslationCovariance.getValue());
         measurementCovariance.set(index, index++, measuredTranslationCovariance.getValue());
         measurementCovariance.set(index, index++, measuredOrientationCovariance.getValue());
         measurementCovariance.set(index, index++, measuredOrientationCovariance.getValue());
         measurementCovariance.set(index, index++, measuredOrientationCovariance.getValue());
         measurementCovariance.set(index, index++, velocityErrorCovariance.getValue());
         measurementCovariance.set(index, index++, velocityErrorCovariance.getValue());
         measurementCovariance.set(index, index++, velocityErrorCovariance.getValue());
         measurementCovariance.set(index, index++, contactVelocityCovariance.getValue());
         measurementCovariance.set(index, index++, contactVelocityCovariance.getValue());
         measurementCovariance.set(index, index++, contactVelocityCovariance.getValue());
         measurementCovariance.set(index, index++, contactOrientationCovariance.getValue());
         measurementCovariance.set(index, index++, contactOrientationCovariance.getValue());
         measurementCovariance.set(index, index, contactOrientationCovariance.getValue());
      }

      setProcessCovariance(processCovariance);
      setMeasurementCovariance(measurementCovariance);
   }

   private final FramePose3D tempPose = new FramePose3D();

   private final Vector3D angularVelocityMeasurement = new Vector3D();

   /** Angular velocity of the measurement link, with respect to world. */
   private final FrameVector3D angularVelocityMeasurementLinkRelativeToWorld = new FrameVector3D();

   /** Angular velocity of the estimation link, with respect to the measurement link. */
   private final FrameVector3D angularVelocityRootJointFrameRelativeToMeasurementLink = new FrameVector3D();

   /** Twist of the estimation link, with respect to the measurement link. */
   private final Twist twistRootJointFrameRelativeToMeasurementLink = new Twist();

   private void updateRootJointTwistAngularPart()
   {
      RigidBodyBasics measurementLink = baseIMU.getMeasurementLink();
      FloatingJointBasics rootJoint = (FloatingJointBasics) baseIMU.getMeasurementLink().getParentJoint();
      ReferenceFrame rootJointFrame = rootJoint.getFrameAfterJoint();
      // T_{rootBody}^{rootBody, measurementLink}
      rootJoint.getSuccessor().getBodyFixedFrame().getTwistRelativeToOther(measurementLink.getBodyFixedFrame(), twistRootJointFrameRelativeToMeasurementLink);
      // T_{rootBody}^{rootJointFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.changeFrame(rootJointFrame);
      // T_{rootJointFrame}^{rootJointFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.setBodyFrame(rootJointFrame);

      // omega_{rootJointFrame}^{rootJointFrame, measurementLink}
      angularVelocityRootJointFrameRelativeToMeasurementLink.setIncludingFrame(twistRootJointFrameRelativeToMeasurementLink.getAngularPart());

      // omega_{measurementLink}^{measurementFrame, world}
      angularVelocityMeasurement.set(baseIMU.getAngularVelocityMeasurement());
      if (imuBiasProvider != null)
      {
         FrameVector3DReadOnly angularVelocityBiasInIMUFrame = imuBiasProvider.getAngularVelocityBiasInIMUFrame(baseIMU);
         if (angularVelocityBiasInIMUFrame != null)
            angularVelocityMeasurement.sub(angularVelocityBiasInIMUFrame);
      }
      angularVelocityMeasurementLinkRelativeToWorld.setIncludingFrame(baseIMU.getMeasurementFrame(), angularVelocityMeasurement);

      // omega_{measurementLink}^{rootJointFrame, world}
      angularVelocityMeasurementLinkRelativeToWorld.changeFrame(rootJointFrame);

      // omega_{rootJointFrame}^{rootJointFrame, world} = omega_{rootJointFrame}^{rootJointFrame, measurementLink} + omega_{measurementLink}^{rootJointFrame, world}
      rootJoint.getJointTwist().getAngularPart().add(angularVelocityRootJointFrameRelativeToMeasurementLink, angularVelocityMeasurementLinkRelativeToWorld);
      rootJoint.updateFrame();

      yoRootJointAngularVelocityMeasurement.setMatchingFrame(rootJoint.getJointTwist().getAngularPart());
      yoRootJointAngularVelocityMeasurementInWorld.setMatchingFrame(rootJoint.getJointTwist().getAngularPart());
      yoBaseAngularVelocityMeasurement.setMatchingFrame(angularVelocityMeasurementLinkRelativeToWorld);
   }

   private final FramePoint3D imuToRoot = new FramePoint3D();
   private final FrameVector3D addedAngular = new FrameVector3D();
   private final FrameVector3D momentArm = new FrameVector3D();

   private void updateRootJointTwistLinearPart(FrameVector3DReadOnly imuLinearVelocity,
                                               FrameVector3DReadOnly imuAngularVelocity,
                                               FixedFrameVector3DBasics rootLinearVelocityToPack)
   {
      FloatingJointBasics rootJoint = (FloatingJointBasics) baseIMU.getMeasurementLink().getParentJoint();
      ReferenceFrame rootJointFrame = rootJoint.getFrameAfterJoint();

      imuToRoot.setToZero(baseIMU.getMeasurementFrame());
      imuToRoot.changeFrame(rootJointFrame);

      momentArm.setIncludingFrame(imuToRoot);
      momentArm.changeFrame(worldFrame);

      addedAngular.cross(momentArm, imuAngularVelocity);

      rootLinearVelocityToPack.set(imuLinearVelocity);
      rootLinearVelocityToPack.add(addedAngular);
   }

   @Override
   public DMatrixRMaj processModel(DMatrixRMaj state)
   {
      predictedState.zero();

      // update the yo variables representing the state
      baseProcessState.set(OdometryIndexHelper.getBasePositionIndex(), state);
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         feetProcessState.get(i).set(OdometryIndexHelper.getFootPositionIndex(i), state);
      }

      // do the prediction
      for (int i = 0; i < processModels.size(); i++)
         processModels.get(i).update();

      // update the predicted state from the yo variablized state
      basePredictedState.get(OdometryIndexHelper.getBasePositionIndex(), state);
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         feetPredictedState.get(i).get(OdometryIndexHelper.getFootPositionIndex(i), state);
      }

      // return the predicted state
      return predictedState;
   }

   @Override
   public DMatrixRMaj measurementModel(DMatrixRMaj predictedState)
   {
      predictedMeasurement.zero();

      // Note that this bypasses the use of predicted state vector, as the predicted state variables are computed in the {@link #processModel(DMatrixRMaj)} call
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         int row = OdometryIndexHelper.getFootVelocityIndex(i);
         MatrixTools.setMatrixBlock(footCovariance, 0, 0, getCovariance(), row, row, 3, 3, 1.0);

         measurementModels.get(i).update(footCovariance);
         measurementModels.get(i).get(i * OdometryIndexHelper.getObservationSizePerLink(), predictedMeasurement);
      }

      // return the predicted measurement
      return predictedMeasurement;
   }

   @Override
   protected DMatrixRMaj linearizeProcessModel(DMatrixRMaj previousState)
   {
      // make sure to call zero, as the compute process model stuff has a bunch of add operators.
      AMatrix.zero();
      int rowOffset = 0;
      int colOffset = 0;
      computeProcessJacobian(baseMeasurement, baseProcessState, estimatorDT, rowOffset, colOffset, AMatrix);

      for (int i = 0; i < feetIMUs.size(); i++)
      {
         rowOffset += OdometryIndexHelper.getStateSizePerLink();
         colOffset += OdometryIndexHelper.getStateSizePerLink();
         computeProcessJacobian(feetMeasurements.get(i), feetProcessState.get(i), estimatorDT, rowOffset, colOffset, AMatrix);
      }

      return AMatrix;
   }

   @Override
   protected DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedState)
   {
      // make sure to call zero, as the compute process model stuff has a bunch of add operators.
      CMatrix.zero();
      int rowOffset = 0;
      int colOffset = 0;
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         boolean footIsInContact = measurementModels.get(i).isInContact.getBooleanValue();
         computeBaseMeasurementJacobian(basePredictedState, feetPredictedState.get(i), rowOffset, CMatrix);
         computeFootMeasurementJacobian(footIsInContact, basePredictedState, feetPredictedState.get(i), gravityVector, rowOffset, colOffset + 15, CMatrix);
         rowOffset += OdometryIndexHelper.getObservationSizePerLink();
         colOffset += OdometryIndexHelper.getStateSizePerLink();
      }

      return CMatrix;
   }

   static void toRotationMatrix(QuaternionReadOnly quaternion, DMatrixRMaj rotationMatrix)
   {
      toRotationMatrix(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS(), rotationMatrix);
   }

   static void toRotationMatrix(double qx, double qy, double qz, double qs, DMatrixRMaj rotationMatrix)
   {
      double qx2 = qx * qx;
      double qy2 = qy * qy;
      double qz2 = qz * qz;
      double qs2 = qs * qs;

      rotationMatrix.set(0, 0, qx2 - qy2 - qz2 + qs2);
      rotationMatrix.set(1, 1, -qx2 + qy2 - qz2 + qs2);
      rotationMatrix.set(2, 2, -qx2 - qy2 + qz2 + qs2);

      double xy = qx * qy;
      double xz = qx * qz;
      double yz = qy * qz;
      double xw = qx * qs;
      double yw = qy * qs;
      double zw = qz * qs;

      rotationMatrix.set(0, 1, 2.0 * (xy - zw));
      rotationMatrix.set(1, 0, 2.0 * (xy + zw));

      rotationMatrix.set(0, 2, 2.0 * (xz + yw));
      rotationMatrix.set(2, 0, 2.0 * (xz - yw));

      rotationMatrix.set(1, 2, 2.0 * (yz - xw));
      rotationMatrix.set(2, 1, 2.0 * (yz + xw));
   }

   static void toRotationMatrixInverse(QuaternionReadOnly quaternion, DMatrixRMaj rotationMatrix)
   {
      toRotationMatrixInverse(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS(), rotationMatrix);
   }

   static void toRotationMatrixInverse(double qx, double qy, double qz, double qs, DMatrixRMaj rotationMatrix)
   {
      double qx2 = qx * qx;
      double qy2 = qy * qy;
      double qz2 = qz * qz;
      double qs2 = qs * qs;

      rotationMatrix.set(0, 0, qx2 - qy2 - qz2 + qs2);
      rotationMatrix.set(1, 1, -qx2 + qy2 - qz2 + qs2);
      rotationMatrix.set(2, 2, -qx2 - qy2 + qz2 + qs2);

      double xy = qx * qy;
      double xz = qx * qz;
      double yz = qy * qz;
      double xw = qx * qs;
      double yw = qy * qs;
      double zw = qz * qs;

      rotationMatrix.set(1, 0, 2.0 * (xy - zw));
      rotationMatrix.set(0, 1, 2.0 * (xy + zw));

      rotationMatrix.set(2, 0, 2.0 * (xz + yw));
      rotationMatrix.set(0, 2, 2.0 * (xz - yw));

      rotationMatrix.set(2, 1, 2.0 * (yz - xw));
      rotationMatrix.set(1, 2, 2.0 * (yz + xw));
   }

   /**
    * This is equation 39
    */
   static void computeProcessJacobian(MeasuredVariables measuredVariables, StateVariables stateVariables, double estimateDT, int rowOffset, int colOffset, DMatrixRMaj jacobianToPack)
   {
      // Do the partial derivative with respect to the base position state
      CommonOps_DDRM.insert(eye3x3, jacobianToPack, rowOffset + OdometryIndexHelper.getStatePositionIndex(), colOffset + OdometryIndexHelper.getStatePositionIndex());
      MatrixTools.setMatrixBlock(jacobianToPack,
                                 rowOffset + OdometryIndexHelper.getStatePositionIndex(),
                                 colOffset + OdometryIndexHelper.getStateVelocityIndex(),
                                 eye3x3,
                                 0,
                                 0,
                                 3,
                                 3,
                                 estimateDT);

      // Do the partial derivative with respect to the base velocity state FIXME lots of garbage
      CommonOps_DDRM.insert(eye3x3, jacobianToPack, rowOffset + OdometryIndexHelper.getStateVelocityIndex(), colOffset + OdometryIndexHelper.getStateVelocityIndex());

      DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
      DMatrixRMaj skewMatrix = new DMatrixRMaj(3, 3);
      Vector3D unbiasedAcceleration = new Vector3D();
      unbiasedAcceleration.sub(measuredVariables.linearAccelerationMeasurement, stateVariables.accelBias);
      toRotationMatrix(stateVariables.orientation, rotationMatrix);
      toSkewSymmetricMatrix(unbiasedAcceleration, skewMatrix);
      MatrixTools.multAddBlock(-estimateDT, rotationMatrix, skewMatrix, jacobianToPack, rowOffset + OdometryIndexHelper.getStateVelocityIndex(), colOffset + OdometryIndexHelper.getStateOrientationIndex());

      MatrixTools.setMatrixBlock(jacobianToPack, rowOffset + OdometryIndexHelper.getStateVelocityIndex(), colOffset + OdometryIndexHelper.getStateGyroBiasIndex(), rotationMatrix, 0, 0, 3, 3, -estimateDT);

      // Do the partial derivative with respect to the orientation state
      Vector3D unbiasedVelocity = new Vector3D();
      unbiasedVelocity.sub(measuredVariables.gyroMeasurement, stateVariables.gyroBias);
      toSkewSymmetricMatrix(unbiasedVelocity, skewMatrix);
      MatrixTools.setMatrixBlock(jacobianToPack, rowOffset + OdometryIndexHelper.getStateOrientationIndex(), colOffset + OdometryIndexHelper.getStateOrientationIndex(), rotationMatrix, 0, 0, 3, 3, -estimateDT);
      MatrixTools.addMatrixBlock(jacobianToPack, rowOffset + OdometryIndexHelper.getStateOrientationIndex(), colOffset + OdometryIndexHelper.getStateOrientationIndex(), eye3x3, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(jacobianToPack, rowOffset + OdometryIndexHelper.getStateOrientationIndex(), colOffset + OdometryIndexHelper.getStateAccelerationBiasIndex(), eye3x3, 0, 0, 3, 3, -estimateDT);

      // Do the partial derivative with respect to the acceleration bias state
      MatrixTools.setMatrixBlock(jacobianToPack, rowOffset + OdometryIndexHelper.getStateAccelerationBiasIndex(), colOffset + OdometryIndexHelper.getStateAccelerationBiasIndex(), eye3x3, 0, 0, 3, 3, 1.0);
      // Do the partial derivative with respect to the gyro bias state
      MatrixTools.setMatrixBlock(jacobianToPack, rowOffset + OdometryIndexHelper.getStateGyroBiasIndex(), colOffset + OdometryIndexHelper.getStateGyroBiasIndex(), eye3x3, 0, 0, 3, 3, 1.0);
   }

   /**
    * This is equation 41
    */
   private void computeBaseMeasurementJacobian(StateVariables baseState, StateVariables footState, int rowOffset, DMatrixRMaj jacobianToPack)
   {
      DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
      toRotationMatrix(baseState.orientation, rotationMatrix);
      CommonOps_DDRM.transpose(rotationMatrix);

      Vector3D footPositionError = new Vector3D();
      Vector3D footVelocityError = new Vector3D();
      footPositionError.sub(footState.translation, baseState.translation);
      footVelocityError.sub(footState.linearVelocity, baseState.linearVelocity);
      baseState.orientation.inverseTransform(footPositionError);
      baseState.orientation.inverseTransform(footVelocityError);

      DMatrixRMaj skewMatrix = new DMatrixRMaj(3, 3);
      toSkewSymmetricMatrix(footPositionError, skewMatrix);

      // First row
      MatrixTools.setMatrixBlock(jacobianToPack, rowOffset, 0, rotationMatrix, 0, 0, 3, 3, -1.0);
      CommonOps_DDRM.insert(skewMatrix, jacobianToPack, rowOffset, 6);

      // FIXME do the second row

      // Third row
      toSkewSymmetricMatrix(footVelocityError, skewMatrix);

      MatrixTools.setMatrixBlock(jacobianToPack, rowOffset + 6, 3, rotationMatrix, 0, 0, 3, 3, -1.0);
      CommonOps_DDRM.insert(skewMatrix, jacobianToPack, rowOffset+ 6, 6);
      // FIXME do the last term, in the fifth column
   }

   /**
    * This is equation 42
    */
   private void computeFootMeasurementJacobian(boolean isInContact, StateVariables baseState, StateVariables footState, FrameVector3DReadOnly gravityVector, int rowOffset, int colOffset, DMatrixRMaj jacobianToPack)
   {
      DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
      toRotationMatrix(baseState.orientation, rotationMatrix);
      CommonOps_DDRM.transpose(rotationMatrix);

      Vector3D footPositionError = new Vector3D();
      Vector3D footVelocityError = new Vector3D();
      footPositionError.sub(footState.translation, baseState.translation);
      footVelocityError.sub(footState.linearVelocity, baseState.linearVelocity);
      baseState.orientation.inverseTransform(footPositionError);
      baseState.orientation.inverseTransform(footVelocityError);

      DMatrixRMaj skewMatrix = new DMatrixRMaj(3, 3);
      toSkewSymmetricMatrix(footPositionError, skewMatrix);

      // First row
      CommonOps_DDRM.insert(rotationMatrix, jacobianToPack, rowOffset, colOffset);

      // FIXME do the second row

      // Third row
      CommonOps_DDRM.insert(rotationMatrix, jacobianToPack, rowOffset + 6, colOffset + 3);

      if (!isInContact)
         return;

      // Fourth row
      CommonOps_DDRM.insert(eye3x3, jacobianToPack, rowOffset + 9, colOffset + 3);

      // Fifth row
      Vector3D gravity = new Vector3D(gravityVector);
      footState.orientation.inverseTransform(gravity);
      toSkewSymmetricMatrix(gravityVector, skewMatrix);
      CommonOps_DDRM.insert(skewMatrix, jacobianToPack, rowOffset + 12, colOffset + 6);
      CommonOps_DDRM.insert(eye3x3, jacobianToPack, rowOffset + 12, colOffset + 9);
   }

   static void toSkewSymmetricMatrix(Vector3DReadOnly vector, DMatrixRMaj matrixToPack)
   {
      matrixToPack.zero();
      matrixToPack.set(0, 1, -vector.getZ());
      matrixToPack.set(0, 2, vector.getY());
      matrixToPack.set(1, 0, vector.getZ());
      matrixToPack.set(1, 2, -vector.getX());
      matrixToPack.set(2, 0, -vector.getY());
      matrixToPack.set(2, 1, -vector.getX());
   }
}
