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
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;
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

   // Internal variables for state holding
   private final YoFrameVector3D yoBaseAngularVelocityMeasurement;

   private final YoFrameVector3D yoRootJointAngularVelocityMeasurementInWorld;
   private final YoFrameVector3D yoRootJointAngularVelocityMeasurement;

   // Outputs
   private final DMatrixRMaj predictedState;
   private final DMatrixRMaj predictedMeasurement;

   private final DMatrixRMaj AMatrix;
   private final DMatrixRMaj CMatrix;

   private final MeasuredVariables baseMeasurement;
   private final List<MeasuredVariables> feetMeasurements = new ArrayList<>();

   private final EstimatedVariables baseCurrentState;
   private final EstimatedVariables basePredictedState;
   private final List<EstimatedVariables> feetCurrentState = new ArrayList<>();
   private final List<EstimatedVariables> feetPredictedState = new ArrayList<>();

   private final List<SensorProcess> sensorProcess = new ArrayList<>();
   private final List<SensorMeasurement> sensorMeasurements = new ArrayList<>();

   private final EstimatedVariables baseObserveState;
   private final List<EstimatedVariables> feetObservationState = new ArrayList<>();

   private final YoFrameQuaternion filteredBaseOrientation;
   private final YoFramePoint3D filteredBaseTranslation;
   private final YoFrameVector3D filteredBaseLinearVelocity;

   private final YoFramePose3D estimatedRootJointPose;
   private final YoFrameVector3D estimatedRootJointLinearVelocity;
   private final YoFrameVector3D estimatedRootJointAngularVelocity;

   private final RigidBodyTransformReadOnly transformToRootJoint;


   // Temporary variables
   private final FrameVector3D linearAcceleration = new FrameVector3D();
   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final Vector3D deltaVector = new Vector3D();
   private final Quaternion estimatedRootJointRotation = new Quaternion();
   private final DMatrixRMaj rotation3x3 = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj offsetVector = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj footCovariance = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj eye3x3 = CommonOps_DDRM.identity(3);
   private final Twist twist = new Twist();

   public OdometryKalmanFilter(IMUSensorReadOnly baseIMU,
                               List<? extends IMUSensorReadOnly> feetIMUs,
                               IMUBiasProvider baseImuBiasProvider,
                               BooleanProvider cancelGravityFromAccelerationMeasurement,
                               double estimatorDT,
                               double gravitationalAcceleration,
                               YoRegistry parentRegistry)
   {
      super(10 + feetIMUs.size() * 6, 9 * feetIMUs.size());

      this.baseIMU = baseIMU;
      this.feetIMUs = feetIMUs;
      this.imuBiasProvider = baseImuBiasProvider;
      this.cancelGravityFromAccelerationMeasurement = cancelGravityFromAccelerationMeasurement;
      this.estimatorDT = estimatorDT;

      predictedState = new DMatrixRMaj(getStateSize(), 1);
      predictedMeasurement = new DMatrixRMaj(getMeasurementSize(), 1);
      AMatrix = new DMatrixRMaj(getStateSize(), getStateSize());
      CMatrix = new DMatrixRMaj(getMeasurementSize(), getStateSize());

      gravityVector = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(() -> worldFrame, new Vector3D(0, 0, -Math.abs(gravitationalAcceleration)));

      baseMeasurement = new MeasuredVariables("base", baseIMU, imuBiasProvider, registry);
      baseCurrentState = new EstimatedVariables("baseCurrent");
      baseObserveState = new EstimatedVariables("baseObserve");
      basePredictedState = new EstimatedVariables("basePredicted");
      sensorProcess.add(new SensorProcess(baseMeasurement, baseCurrentState, basePredictedState, cancelGravityFromAccelerationMeasurement, gravityVector, estimatorDT));
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         MeasuredVariables footMeasurements = new MeasuredVariables("foot" + i, feetIMUs.get(i), imuBiasProvider, registry);
         EstimatedVariables footCurrentState = new EstimatedVariables("foot" + i + "Current");
         EstimatedVariables footObservationState = new EstimatedVariables("foot" + i + "Observe");
         EstimatedVariables footPredictedState = new EstimatedVariables("foot" + i + "Predicted");
         feetMeasurements.add(footMeasurements);
         feetObservationState.add(footObservationState);
         feetCurrentState.add(footCurrentState);
         feetPredictedState.add(footPredictedState);
         sensorProcess.add(new SensorProcess(footMeasurements, footCurrentState, footPredictedState, cancelGravityFromAccelerationMeasurement, gravityVector, estimatorDT));
         sensorMeasurements.add(new SensorMeasurement("foot" + i, baseObserveState, footObservationState, baseIMU, feetIMUs.get(i), registry));
      }

      filteredBaseOrientation = new YoFrameQuaternion("filteredBaseOrientation", worldFrame, registry);
      filteredBaseTranslation = new YoFramePoint3D("filteredBaseTranslation", worldFrame, registry);
      filteredBaseLinearVelocity = new YoFrameVector3D("filteredBaseLinearVelocity", worldFrame, registry);

      estimatedRootJointPose = new YoFramePose3D("estimatedRootJointPose", worldFrame, registry);
      estimatedRootJointLinearVelocity = new YoFrameVector3D("estimatedRootJointLinearVelocity", worldFrame, registry);
      estimatedRootJointAngularVelocity = new YoFrameVector3D("estimatedRootJointAngularVelocity", worldFrame, registry);

      transformToRootJoint = baseIMU.getMeasurementFrame().getTransformToDesiredFrame(baseIMU.getMeasurementLink().getParentJoint().getFrameAfterJoint());

      yoBaseAngularVelocityMeasurement = new YoFrameVector3D("baseImuLinearAccelerationInWorld", baseIMU.getMeasurementFrame(), registry);
      yoRootJointAngularVelocityMeasurement = new YoFrameVector3D("rootJointImuLinearAccelerationInWorld", baseIMU.getMeasurementFrame(), registry);
      yoRootJointAngularVelocityMeasurementInWorld = new YoFrameVector3D("rootJointImuLinearAccelerationInWorld", worldFrame, registry);

      parentRegistry.addChild(registry);
   }

   private static class SensorProcess
   {
      // State providers
      private final MeasuredVariables measuredVariables;
      private final EstimatedVariables currentState;
      private final EstimatedVariables predictedState;
      private final BooleanProvider cancelGravityFromAccelerationMeasurement;
      private final FrameVector3DReadOnly gravityVector;
      private final double estimatorDt;

      // Temp variables
      private final Vector3D unbiasedAcceleration = new Vector3D();
      private final Vector3D unbiasedAngularVelocity = new Vector3D();
      private final Vector3D integratedVelocity = new Vector3D();
      private final Quaternion integratedRotation = new Quaternion();

      public SensorProcess(MeasuredVariables measuredVariables,
                           EstimatedVariables currentState,
                           EstimatedVariables predictedState,
                           BooleanProvider cancelGravityFromAccelerationMeasurement,
                           FrameVector3DReadOnly gravityVector,
                           double estimatorDt)
      {
         this.measuredVariables = measuredVariables;
         this.currentState = currentState;
         this.predictedState = predictedState;
         this.cancelGravityFromAccelerationMeasurement = cancelGravityFromAccelerationMeasurement;
         this.gravityVector = gravityVector;
         this.estimatorDt = estimatorDt;
      }

      public void update()
      {
         unbiasedAcceleration.sub(measuredVariables.linearAcceleration, currentState.accelBias);
         unbiasedAngularVelocity.sub(measuredVariables.angularVelocity, currentState.gyroBias);
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

   private static class SensorMeasurement
   {
      private final EstimatedVariables baseObserve;
      private final EstimatedVariables footObserve;
      private final IMUSensorReadOnly baseIMU;
      private final IMUSensorReadOnly footIMU;

      // State variables
      public final YoVector3D footPredictedRelativePosition;
      public final YoVector3D footPredictedRelativeOrientation;

      // Temp variables
      private final FramePose3D footPose = new FramePose3D();
      private final Quaternion footOrientationError = new Quaternion();
      private final Vector3D velocityError = new Vector3D();

      public SensorMeasurement(String prefix, EstimatedVariables baseObserve, EstimatedVariables footObserve, IMUSensorReadOnly baseIMU,
                               IMUSensorReadOnly footIMU, YoRegistry registry)
      {
         this.baseObserve = baseObserve;
         this.footObserve = footObserve;
         this.baseIMU = baseIMU;
         this.footIMU = footIMU;

         footPredictedRelativePosition = new YoVector3D(prefix + "PredictedRelativePosition", registry);
         footPredictedRelativeOrientation = new YoVector3D(prefix + "PredictedRelativeOrientation", registry);
      }

      public void update()
      {
         // Update the predicted foot position in world
         footPredictedRelativePosition.sub(footObserve.translation, baseObserve.translation);
         baseObserve.orientation.transform(footPredictedRelativePosition);

         // get the foot pose relative to the base link, which should be entirely based on kinematics
         footPose.setToZero(footIMU.getMeasurementFrame());
         footPose.changeFrame(baseIMU.getMeasurementFrame());

         // FIXME compute the predicted relative orientation
         footOrientationError.multiply(footPose.getOrientation().inverse(), baseObserve.orientation.inverse());
         footOrientationError.multiply(footObserve.orientation);
         OdometryIndexHelper.logMap(footOrientationError, footPredictedRelativeOrientation);


         // Compute 
         // FIXME finish the sensor measurement process

      }

      public void get(int start, DMatrixRMaj measurementToPack)
      {
         footPredictedRelativePosition.get(start, measurementToPack);
         footPredictedRelativeOrientation.get(start + 3, measurementToPack);
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

      public MeasuredVariables(String prefix,
                               IMUSensorReadOnly imu,
                               IMUBiasProvider imuBiasProvider,
                               YoRegistry registry)
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
   private class EstimatedVariables
   {
      public final YoFramePoint3D translation;
      public final YoFrameVector3D linearVelocity;
      public final YoFrameQuaternion orientation;
      public final YoFrameVector3D accelBias;
      public final YoFrameVector3D gyroBias;

      public EstimatedVariables(String prefix)
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
      updateMeasures();
      updateRootJointTwistAngularPart();

      // FIXME populate the observations, which are
      // FIXME 1. the relative distance between the IMU and the foot in the IMU frame
      // FIXME 2. the relative linear velocity between the IMU and the foot in the IMU frame
      // FIXME 3. the absolute linear velocity of the foot

      DMatrixRMaj stateEstimate = calculateEstimate();

      filteredBaseTranslation.set(stateEstimate);
      filteredBaseLinearVelocity.set(3, stateEstimate);
      filteredBaseOrientation.set(6, stateEstimate);

      // Transform the pose of the IMU in the world frame to the pose of the root joint in the world frame
      tempPose.set(filteredBaseOrientation, filteredBaseTranslation);
      tempPose.appendTransform(transformToRootJoint);
      estimatedRootJointPose.set(tempPose);
      estimatedRootJointAngularVelocity.set(yoRootJointAngularVelocityMeasurementInWorld);

      updateRootJointTwistLinearPart(filteredBaseLinearVelocity,
                                     estimatedRootJointAngularVelocity,
                                     estimatedRootJointLinearVelocity);
   }

   private final FramePose3D tempPose = new FramePose3D();


   public void updateMeasures()
   {
      // TODO re-enable this
//      if (!isEstimationEnabled())
//         return;

      baseMeasurement.update();
      for (int i = 0; i < feetMeasurements.size(); i++)
         feetMeasurements.get(i).update();
   }

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
      // update the yo variables representing the state
      baseCurrentState.set(OdometryIndexHelper.getBasePositionIndex(), state);
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         feetCurrentState.get(i).set(OdometryIndexHelper.getFootPositionIndex(i), state);
      }

      // do the prediction
      for (int i = 0; i < sensorProcess.size(); i++)
         sensorProcess.get(i).update();

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
   public DMatrixRMaj measurementModel(DMatrixRMaj state)
   {
      // update the yo variables representing the state
      baseObserveState.set(OdometryIndexHelper.getBasePositionIndex(), state);
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         feetObservationState.get(i).set(OdometryIndexHelper.getFootPositionIndex(i), state);
      }

      for (int i = 0; i < sensorMeasurements.size(); i++)
      {
         sensorMeasurements.get(i).update();
         sensorMeasurements.get(i).get(i * 10, predictedMeasurement); // FIXME fix this offset
      }

      estimatedRootJointRotation.set(6, state);

      // return the predicted measurement
      return predictedMeasurement;
   }

   private final DMatrixRMaj temp = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj tempScalar  = new DMatrixRMaj(1, 1);
   private boolean isFootInContact(int footIndex, DMatrixRMaj state)
   {
      int startRow = 10 + footIndex * 6;
      CommonOps_DDRM.extract(state, startRow, startRow + 3, 0, 1, offsetVector, 0, 0);
      CommonOps_DDRM.extract(getCovariance(), startRow, startRow + 3, startRow, startRow + 3, footCovariance, 0, 0);

      CommonOps_DDRM.mult(footCovariance, offsetVector, temp);
      CommonOps_DDRM.multTransA(offsetVector, temp, tempScalar);

      // tests the mahalonobis distance of the foot velocity being below a certain threshold.
      return tempScalar.get(0, 0) < 0.2;
   }

   @Override
   protected DMatrixRMaj linearizeProcessModel(DMatrixRMaj previousState)
   {
      // Populate the gradient of the base position w.r.t. the state
      CommonOps_DDRM.insert(eye3x3, AMatrix, 0, 0);
      MatrixTools.setMatrixBlock(AMatrix, 0, 3, eye3x3, 0, 0, 3, 3, estimatorDT);

      // Populate the gradient of the base velocity w.r.t. the state
      CommonOps_DDRM.insert(eye3x3, AMatrix, 3, 3);
      // FIXME add the effect of the gradient of the base rotation

      // FIXME do the gradient crap for the base orientation, which is going to be frustrating

      for (int i = 0; i < feetIMUs.size(); i++)
      {
         int stateOffset = 10 + i * 6;
         // Populate the gradient of the foot position w.r.t. the state
         CommonOps_DDRM.insert(eye3x3, AMatrix, stateOffset, stateOffset);
         MatrixTools.setMatrixBlock(AMatrix, stateOffset, stateOffset + 3, eye3x3, 0, 0, 3, 3, estimatorDT);

         // Populate the gradient of the foot position w.r.t. the state
         // FIXME add the effect of the gradient of the base rotation
         CommonOps_DDRM.insert(eye3x3, AMatrix, stateOffset + 3, stateOffset + 3);
      }

      return AMatrix;
   }

   @Override
   protected DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedState)
   {
      estimatedRootJointRotation.set(6, predictedState);
      toRotationMatrix(estimatedRootJointRotation, rotation3x3);

      CommonOps_DDRM.transpose(rotation3x3);

      for (int i = 0; i < feetIMUs.size(); i++)
      {
         int footStateOffset = 10 + i * 6;
         int measureOffset = 9 * i;
         // R^T * (s_k - p_k)
         CommonOps_DDRM.insert(rotation3x3, CMatrix, measureOffset, footStateOffset);
         MatrixTools.setMatrixBlock(CMatrix, measureOffset, 0, rotation3x3, 0, 0, 3, 3, -1.0);

         MatrixTools.setMatrixBlock(offsetVector, 0, 0, predictedState, footStateOffset, 0, 3, 1, 1.0);
         MatrixTools.addMatrixBlock(offsetVector, 0, 0, predictedState, 0, 0, 3, 1, -1.0);

         // TODO add the effect of the gradient of the base rotation

         // R^T * (v_k - sDot_k)
         CommonOps_DDRM.insert(rotation3x3, CMatrix, measureOffset + 3, 3);
         MatrixTools.setMatrixBlock(CMatrix, measureOffset + 3, footStateOffset + 3, rotation3x3, 0, 0, 3, 3, -1.0);
         // TODO add the effect of the gradient of the base rotation

         MatrixTools.setMatrixBlock(offsetVector, 0, 0, predictedState, 3, 0, 3, 1, 1.0);
         MatrixTools.addMatrixBlock(offsetVector, 0, 0, predictedState, footStateOffset + 3, 0, 3, 1, -1.0);

         // sDot_k
         CommonOps_DDRM.insert(eye3x3, CMatrix, measureOffset + 6, footStateOffset + 3);
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
}
