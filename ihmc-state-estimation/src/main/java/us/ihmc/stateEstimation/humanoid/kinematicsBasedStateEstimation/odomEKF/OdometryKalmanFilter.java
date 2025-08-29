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
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.parameterEstimation.ExtendedKalmanFilter;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

/**
 * The process model in this class is defined as
 */
public class OdometryKalmanFilter extends ExtendedKalmanFilter
{
   static final boolean includeBias = false;

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
   private final YoDouble contactAccelerationCovariance = new YoDouble("contactAccelerationMeasureCovariance", registry);

   private final YoDouble contactThreshold = new YoDouble("contactVarianceThreshold", registry);


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
   private final List<ObservationModel> observationModels = new ArrayList<>();

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

      // Initialize covariance
      processCovariance = new DMatrixRMaj(getStateSize(), getStateSize());
      measurementCovariance = new DMatrixRMaj(getMeasurementSize(), getMeasurementSize());
      MatrixTools.setDiagonal(processCovariance, 1e-2);
      MatrixTools.setDiagonal(measurementCovariance, 1e-3);

//      translationCovariance.set(1e-3);
//      velocityCovariance.set(1e-2);
//      orientationCovariance.set(1e-2);
//      biasCovariance.set(1e-4);
//
//      measuredTranslationCovariance.set(1e-5);
//      measuredOrientationCovariance.set(1e-2);
//      velocityErrorCovariance.set(1e-3);
//      contactVelocityCovariance.set(1e-5);
//      contactAccelerationCovariance.set(1.0);

            translationCovariance.set(1e-4);
            velocityCovariance.set(1e-4);
            orientationCovariance.set(1e-4);
            biasCovariance.set(1e-4);

            measuredTranslationCovariance.set(1e-4);
            measuredOrientationCovariance.set(1e-4);
            velocityErrorCovariance.set(1e-4);
            contactVelocityCovariance.set(1e-4);
            contactAccelerationCovariance.set(1e-4);

      contactThreshold.set(Double.MAX_VALUE);

      this.baseIMU = baseIMU;
      this.feetIMUs = feetIMUs;
      this.imuBiasProvider = baseImuBiasProvider;
      this.estimatorDT = estimatorDT;

      predictedState = new DMatrixRMaj(getStateSize(), 1);
      predictedMeasurement = new DMatrixRMaj(getMeasurementSize(), 1);
      AMatrix = new DMatrixRMaj(getStateSize(), getStateSize());
      CMatrix = new DMatrixRMaj(getMeasurementSize(), getStateSize());
      observationVector = new DMatrixRMaj(OdometryIndexHelper.getObservationSizePerLink() * feetIMUs.size(), 1);

      gravityVector = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(() -> worldFrame, new Vector3D(0, 0, Math.abs(gravitationalAcceleration)));

      baseMeasurement = new MeasuredVariables("base", baseIMU, baseIMU, imuBiasProvider, registry);
      baseProcessState = new StateVariables("baseProcessState", baseIMU.getMeasurementFrame(), registry);
      basePredictedState = new StateVariables("basePredicted",  baseIMU.getMeasurementFrame(), registry);
      processModels.add(new ProcessModel(baseProcessState,
                                         basePredictedState,
                                         cancelGravityFromAccelerationMeasurement,
                                         gravityVector,
                                         estimatorDT));
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         MeasuredVariables footMeasurements = new MeasuredVariables("foot" + i, baseIMU, feetIMUs.get(i), imuBiasProvider, registry);
         StateVariables footProcessState = new StateVariables("foot" + i + "ProcessState", feetIMUs.get(i).getMeasurementFrame(), registry);
         StateVariables footPredictedState = new StateVariables("foot" + i + "PredictedState", feetIMUs.get(i).getMeasurementFrame(), registry);
         feetMeasurements.add(footMeasurements);
         feetProcessState.add(footProcessState);
         feetPredictedState.add(footPredictedState);

         observationModels.add(new ObservationModel(footMeasurements));
         processModels.add(new ProcessModel(footProcessState,
                                            footPredictedState,
                                            cancelGravityFromAccelerationMeasurement,
                                            gravityVector,
                                            estimatorDT));
         measurementModels.add(new MeasurementModel("foot" + i,
                                                    basePredictedState,
                                                    footPredictedState,
                                                    footMeasurements,
                                                    gravityVector,
                                                    contactThreshold,
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

   private void initialize()
   {
      baseProcessState.initialize();
      for (int i = 0; i < feetProcessState.size(); i++)
         feetProcessState.get(i).initialize();

      // update the predicted state from the yo variablized state
      baseProcessState.get(OdometryIndexHelper.getBasePositionIndex(), state);
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         feetProcessState.get(i).get(OdometryIndexHelper.getFootPositionIndex(i), state);
      }

      initializeState(state);
   }

   static class MeasuredVariables
   {
      // State providers
      private final IMUSensorReadOnly baseIMU;
      private final IMUSensorReadOnly imu;
      private final IMUBiasProvider imuBiasProvider;

      // State recorders
      public final YoFrameVector3D gyroMeasurementInWorld;
      public final YoFrameVector3D gyroMeasurement;
      public final YoFrameVector3D accelMeasurementInWorld;
      public final YoFrameVector3D accelMeasurement;

      public final YoFramePoint3D positionMeasurement;
      public final YoFrameQuaternion orientationMeasurement;
      public final YoFrameVector3D linearVelocity;

      // Temp variables
      private final FrameVector3D linearAcceleration = new FrameVector3D();
      private final FrameVector3D angularVelocity = new FrameVector3D();
      private final Twist twist = new Twist();

      public MeasuredVariables(String prefix, IMUSensorReadOnly baseIMU, IMUSensorReadOnly imu, IMUBiasProvider imuBiasProvider, YoRegistry registry)
      {
         this.baseIMU = baseIMU;
         this.imu = imu;
         this.imuBiasProvider = imuBiasProvider;

         gyroMeasurementInWorld = new YoFrameVector3D(prefix + "GyroMeasurementInWorld", worldFrame, registry);
         gyroMeasurement = new YoFrameVector3D(prefix + "GyroMeasurement", imu.getMeasurementFrame(), registry);
         accelMeasurementInWorld = new YoFrameVector3D(prefix + "AccelMeasurementInWorld", worldFrame, registry);
         accelMeasurement = new YoFrameVector3D(prefix + "AccelMeasurement", imu.getMeasurementFrame(), registry);

         positionMeasurement = new YoFramePoint3D(prefix + "PositionMeasurement", baseIMU.getMeasurementFrame(), registry);
         orientationMeasurement = new YoFrameQuaternion(prefix + "OrientationMeasurement", baseIMU.getMeasurementFrame(), registry);

         linearVelocity = new YoFrameVector3D(prefix + "LinearVelocity", worldFrame, registry);
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

         accelMeasurementInWorld.setMatchingFrame(linearAcceleration);
         accelMeasurement.setMatchingFrame(linearAcceleration);

         positionMeasurement.setFromReferenceFrame(imu.getMeasurementFrame());
         orientationMeasurement.setFromReferenceFrame(imu.getMeasurementFrame());

         imu.getMeasurementFrame().getTwistRelativeToOther(baseIMU.getMeasurementFrame(), twist);
         twist.changeFrame(worldFrame);
         linearVelocity.set(twist.getLinearPart());
      }
   }

   static class StateVariables
   {
      public final YoFramePoint3D translation;
      public final YoFrameVector3D linearVelocity;
      public final YoFrameQuaternion orientation;
      public final YoFrameVector3D accelBias;
      public final YoFrameVector3D gyroBias;

      public final YoFrameVector3D unbiasedAccel;
      public final YoFrameVector3D unbiasedGyro;

      private final MovingReferenceFrame sensorFrame;

      public StateVariables(String prefix, MovingReferenceFrame sensorFrame, YoRegistry registry)
      {
         this.sensorFrame = sensorFrame;

         translation = new YoFramePoint3D(prefix + "Translation", worldFrame, registry);
         linearVelocity = new YoFrameVector3D(prefix + "LinearVelocity", worldFrame, registry);
         orientation = new YoFrameQuaternion(prefix + "Orientation", worldFrame, registry);
         accelBias = new YoFrameVector3D(prefix + "AccelBias", sensorFrame, registry);
         gyroBias = new YoFrameVector3D(prefix + "GyroBias", sensorFrame, registry);
         unbiasedAccel = new YoFrameVector3D(prefix + "UnbiasedAccel", sensorFrame, registry);
         unbiasedGyro = new YoFrameVector3D(prefix + "UnbiasedGyro", sensorFrame, registry);
      }

      public void initialize()
      {
         this.translation.setFromReferenceFrame(sensorFrame);
         this.orientation.setFromReferenceFrame(sensorFrame);
         linearVelocity.setMatchingFrame(sensorFrame.getTwistOfFrame().getLinearPart());
         accelBias.setToZero();
         gyroBias.setToZero();
      }

      public void set(int start, DMatrixRMaj state, MeasuredVariables measuredVariables)
      {
         translation.set(start + OdometryIndexHelper.getStatePositionIndex(), state);
         linearVelocity.set(start + OdometryIndexHelper.getStateVelocityIndex(), state);
         orientation.set(start + OdometryIndexHelper.getStateOrientationIndex(), state);
         accelBias.set(start + OdometryIndexHelper.getStateAccelerationBiasIndex(), state);
         gyroBias.set(start + OdometryIndexHelper.getStateGyroBiasIndex(), state);

         if (OdometryKalmanFilter.includeBias)
         {
            unbiasedAccel.sub(measuredVariables.accelMeasurement, accelBias);
            unbiasedGyro.sub(measuredVariables.gyroMeasurement, gyroBias);
         }
         else
         {
            unbiasedAccel.set(measuredVariables.accelMeasurement);
            unbiasedGyro.set(measuredVariables.gyroMeasurement);
         }
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

   private boolean initialized = false;

   public void compute()
   {
      updateCovariance();

      baseMeasurement.update();
      for (int i = 0; i < feetMeasurements.size(); i++)
         feetMeasurements.get(i).update();

      for (int i = 0; i < observationModels.size(); i++)
      {
         observationModels.get(i).get(i * OdometryIndexHelper.getObservationSizePerLink(), observationVector);
      }

      if (!initialized)
      {
         initialize();
         initialized = true;
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
         for (int j = 0; j < 3; j++)
            measurementCovariance.set(index + j, index + j, measuredTranslationCovariance.getValue());
         index += 3;
         for (int j = 0; j < 3; j++)
            measurementCovariance.set(index + j, index + j, measuredOrientationCovariance.getValue());
         index += 3;
         for (int j = 0; j < 3; j++)
            measurementCovariance.set(index + j, index + j, velocityErrorCovariance.getValue());
         index += 3;
         for (int j = 0; j < 3; j++)
            measurementCovariance.set(index + j, index + j, contactVelocityCovariance.getValue());
         index += 3;
         for (int j = 0; j < 3; j++)
            measurementCovariance.set(index + j, index + j, contactAccelerationCovariance.getValue());
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
      baseProcessState.set(OdometryIndexHelper.getBasePositionIndex(), state, baseMeasurement);
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         feetProcessState.get(i).set(OdometryIndexHelper.getFootPositionIndex(i), state, feetMeasurements.get(i));
      }

      // do the prediction
      for (int i = 0; i < processModels.size(); i++)
         processModels.get(i).update();

      // update the predicted state from the yo variablized state
      basePredictedState.get(OdometryIndexHelper.getBasePositionIndex(), predictedState);
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         feetPredictedState.get(i).get(OdometryIndexHelper.getFootPositionIndex(i), predictedState);
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
      int offset = 0;
      ProcessModel.computeProcessJacobian(baseProcessState, estimatorDT, offset, AMatrix);

      for (int i = 0; i < feetIMUs.size(); i++)
      {
         offset += OdometryIndexHelper.getStateSizePerLink();
         ProcessModel.computeProcessJacobian(feetProcessState.get(i), estimatorDT, offset, AMatrix);
      }

      return AMatrix;
   }

   @Override
   protected DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedState)
   {
      // make sure to call zero, as the compute process model stuff has a bunch of add operators.
      CMatrix.zero();
      int rowOffset = 0;
      int colOffset = OdometryIndexHelper.getStateSizePerLink();
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         MeasurementModel measurementModel = measurementModels.get(i);
         measurementModel.computeBaseMeasurementJacobian(basePredictedState, feetPredictedState.get(i), feetMeasurements.get(i), rowOffset, CMatrix);
         measurementModel.computeFootMeasurementJacobian(basePredictedState, feetPredictedState.get(i), feetMeasurements.get(i), gravityVector, rowOffset, colOffset, CMatrix);
         rowOffset += OdometryIndexHelper.getObservationSizePerLink();
         colOffset += OdometryIndexHelper.getStateSizePerLink();
      }

      return CMatrix;
   }

   static void l3Operator(QuaternionReadOnly quaternion, DMatrixRMaj matrixToPack)
   {
      OdometryTools.toSkewSymmetricMatrix(quaternion.getX(), quaternion.getY(), quaternion.getZ(), matrixToPack);
      CommonOps_DDRM.addEquals(matrixToPack, quaternion.getS(), eye3x3);
   }

   static void r3Operator(QuaternionReadOnly quaternion, DMatrixRMaj matrixToPack)
   {
      OdometryTools.toSkewSymmetricMatrix(quaternion.getX(), quaternion.getY(), quaternion.getZ(), matrixToPack);
      CommonOps_DDRM.scale(-1.0, matrixToPack);
      CommonOps_DDRM.addEquals(matrixToPack, quaternion.getS(), eye3x3);
   }
}
