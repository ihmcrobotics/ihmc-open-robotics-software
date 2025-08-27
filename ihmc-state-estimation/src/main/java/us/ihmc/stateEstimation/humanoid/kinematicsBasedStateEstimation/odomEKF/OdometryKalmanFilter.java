package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
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
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

/**
 * The process model in this class is defined as
 */
public class OdometryKalmanFilter extends ExtendedKalmanFilter
{
   // Constants and providers
   private final BooleanProvider cancelGravityFromAccelerationMeasurement;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final IMUSensorReadOnly baseIMU;
   private final List<? extends IMUSensorReadOnly> feetIMUs;
   private final IMUBiasProvider imuBiasProvider;

   private final double estimatorDT;
   private final FrameVector3DReadOnly gravityVector;

   // Internal variables for state holding
   private final YoFrameVector3D yoBaseLinearAccelerationMeasurementInWorld;
   private final YoFrameVector3D yoBaseLinearAccelerationMeasurement;
   private final YoFrameVector3D yoBaseAngularVelocityMeasurement;

   private final YoFrameVector3D yoRootJointAngularVelocityMeasurementInWorld;
   private final YoFrameVector3D yoRootJointAngularVelocityMeasurement;

   private final List<YoFrameVector3D> yoFootLinearAccelerationMeasurementInWorld = new ArrayList<>();
   private final List<YoFrameVector3D> yoFootLinearAccelerationMeasurement = new ArrayList<>();

   // Outputs
   private final DMatrixRMaj predictedState;
   private final DMatrixRMaj predictedMeasurement;

   private final DMatrixRMaj AMatrix;
   private final DMatrixRMaj CMatrix;

   private final YoFrameQuaternion predictedBaseOrientation;
   private final YoFramePoint3D predictedBaseTranslation;
   private final YoFrameVector3D predictedBaseLinearVelocity;

   private final YoFrameQuaternion filteredBaseOrientation;
   private final YoFramePoint3D filteredBaseTranslation;
   private final YoFrameVector3D filteredBaseLinearVelocity;

   private final YoFramePose3D estimatedRootJointPose;
   private final YoFrameVector3D estimatedRootJointLinearVelocity;
   private final YoFrameVector3D estimatedRootJointAngularVelocity;

   private final List<YoFramePoint3D> predictedFootTranslations = new ArrayList<>();
   private final List<YoFrameVector3D> predictedFootLinearVelocities = new ArrayList<>();

   private final RigidBodyTransformReadOnly transformToRootJoint;

   private final List<YoFramePoint3D> predictedRelativeFootPositionMeasurements = new ArrayList<>();
   private final List<YoFrameVector3D> predictedRelativeFootVelocityMeasurements = new ArrayList<>();
   private final List<YoFrameVector3D> predictedWorldFootVelocityMeasurements = new ArrayList<>();

   // Temporary variables
   private final FrameVector3D linearAcceleration = new FrameVector3D();
   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final Vector3D deltaVector = new Vector3D();
   private final Quaternion estimatedRootJointRotation = new Quaternion();
   private final DMatrixRMaj rotation3x3 = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj predictedBaseIMUPositionState = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj predictedBaseIMUVelocityState = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj predictedFootPositionState = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj predictedFootVelocityState = new DMatrixRMaj(3, 1);

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

      predictedBaseOrientation = new YoFrameQuaternion("predictedBaseOrientation", worldFrame, registry);
      predictedBaseTranslation = new YoFramePoint3D("predictedBaseTranslation", worldFrame, registry);
      predictedBaseLinearVelocity = new YoFrameVector3D("predictedBaseLinearVelocity", worldFrame, registry);

      filteredBaseOrientation = new YoFrameQuaternion("filteredBaseOrientation", worldFrame, registry);
      filteredBaseTranslation = new YoFramePoint3D("filteredBaseTranslation", worldFrame, registry);
      filteredBaseLinearVelocity = new YoFrameVector3D("filteredBaseLinearVelocity", worldFrame, registry);

      estimatedRootJointPose = new YoFramePose3D("estimatedRootJointPose", worldFrame, registry);
      estimatedRootJointLinearVelocity = new YoFrameVector3D("estimatedRootJointLinearVelocity", worldFrame, registry);
      estimatedRootJointAngularVelocity = new YoFrameVector3D("estimatedRootJointAngularVelocity", worldFrame, registry);

      transformToRootJoint = baseIMU.getMeasurementFrame().getTransformToDesiredFrame(baseIMU.getMeasurementLink().getParentJoint().getFrameAfterJoint());

      yoBaseLinearAccelerationMeasurement = new YoFrameVector3D("baseImuLinearAcceleration", baseIMU.getMeasurementFrame(), registry);
      yoBaseLinearAccelerationMeasurementInWorld = new YoFrameVector3D("baseImuLinearAccelerationInWorld", worldFrame, registry);
      yoBaseAngularVelocityMeasurement = new YoFrameVector3D("baseImuLinearAccelerationInWorld", baseIMU.getMeasurementFrame(), registry);
      yoRootJointAngularVelocityMeasurement = new YoFrameVector3D("rootJointImuLinearAccelerationInWorld", baseIMU.getMeasurementFrame(), registry);
      yoRootJointAngularVelocityMeasurementInWorld = new YoFrameVector3D("rootJointImuLinearAccelerationInWorld", worldFrame, registry);

      for (int i = 0; i < feetIMUs.size(); i++)
      {
         IMUSensorReadOnly footIMU = feetIMUs.get(i);
         YoFrameVector3D footLinearAccelerationMeasurement = new YoFrameVector3D("foot" + i + "ImuLinearAcceleration", footIMU.getMeasurementFrame(), registry);
         YoFrameVector3D footLinearAccelerationMeasurementInWorld = new YoFrameVector3D("foot" + i + "ImuLinearAccelerationInWorld", worldFrame, registry);
         yoFootLinearAccelerationMeasurement.add(footLinearAccelerationMeasurement);
         yoFootLinearAccelerationMeasurementInWorld.add(footLinearAccelerationMeasurementInWorld);

         predictedFootTranslations.add(new YoFramePoint3D("predictedFoot" + i + "Translation", worldFrame, registry));
         predictedFootLinearVelocities.add(new YoFrameVector3D("predictedFoot" + i + "LinearVelocity", worldFrame, registry));

         predictedRelativeFootPositionMeasurements.add(new YoFramePoint3D("predictedRelativeFoot" + i + "PositionMeasurement", baseIMU.getMeasurementFrame(), registry));
         predictedRelativeFootVelocityMeasurements.add(new YoFrameVector3D("predictedRelativeFoot" + i + "VelocityMeasurement", baseIMU.getMeasurementFrame(), registry));
         predictedWorldFootVelocityMeasurements.add(new YoFrameVector3D("predictedWorldFoot" + i + "VelocityMeasurement", worldFrame, registry));
      }

      parentRegistry.addChild(registry);
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

      FrameVector3DReadOnly biasInput = imuBiasProvider.getLinearAccelerationBiasInIMUFrame(baseIMU);
      Vector3DReadOnly rawInput = baseIMU.getLinearAccelerationMeasurement();

      linearAcceleration.setReferenceFrame(baseIMU.getMeasurementFrame());
      linearAcceleration.sub(rawInput, biasInput);

      // Update acceleration in world (minus gravity)
      if (cancelGravityFromAccelerationMeasurement.getValue())
      {
         // FIXME use the estimated orientation of the base instead of the measured one.
         linearAcceleration.changeFrame(worldFrame);
         linearAcceleration.add(gravityVector);
      }

      yoBaseLinearAccelerationMeasurementInWorld.setMatchingFrame(linearAcceleration);
      yoBaseLinearAccelerationMeasurement.setMatchingFrame(linearAcceleration);

      for (int i = 0; i < feetIMUs.size(); i++)
      {
         IMUSensorReadOnly footIMU = feetIMUs.get(i);
         FrameVector3DReadOnly footBiasInput = imuBiasProvider.getLinearAccelerationBiasInIMUFrame(footIMU);
         Vector3DReadOnly footRawInput = footIMU.getLinearAccelerationMeasurement();

         linearAcceleration.setReferenceFrame(footIMU.getMeasurementFrame());
         linearAcceleration.sub(footRawInput, footBiasInput);

         // Update acceleration in world (minus gravity)
         if (cancelGravityFromAccelerationMeasurement.getValue())
         {
            // FIXME use the estimated orientation of the base instead of the measured one.
            linearAcceleration.changeFrame(worldFrame);
            linearAcceleration.add(gravityVector);
         }

         yoFootLinearAccelerationMeasurementInWorld.get(i).setMatchingFrame(linearAcceleration);
         yoFootLinearAccelerationMeasurement.get(i).setMatchingFrame(linearAcceleration);
      }
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
      predictedState.set(state);
      estimatedRootJointRotation.set(6, state);

      // Compute the predicted position state
      // p_k+1 = p_k + v_k * dt
      MatrixTools.setMatrixBlock(predictedBaseIMUPositionState, 0, 0, state, 0, 0, 3, 1, 1.0);
      MatrixTools.addMatrixBlock(predictedBaseIMUPositionState, 0, 0, state, 3, 0, 3, 1, estimatorDT );
      predictedBaseTranslation.set(predictedBaseIMUPositionState);

      // Compute the predicted velocity state
      // v_k+1 = v_k + a_k * dt
      // get the acceleration of the base IMU in the base frame, but using the estimated orientation that is internal to the filter.
      linearAcceleration.setIncludingFrame(yoBaseLinearAccelerationMeasurement);
      estimatedRootJointRotation.transform(linearAcceleration); // TODO make sure this isn't an inverse transform
      // figure out the change in velocity based on integrating this acceleration
      linearAcceleration.get(predictedBaseIMUVelocityState);
      CommonOps_DDRM.scale(estimatorDT, predictedBaseIMUVelocityState);
      // add to the velocity the current velocity estimate
      MatrixTools.addMatrixBlock(predictedBaseIMUVelocityState, 0, 0, state, 3, 0, 3, 1, 1.0);
      predictedBaseLinearVelocity.set(predictedBaseIMUVelocityState);

      // Compute the predicted orientation state by integrating the angular velocity measurement
      deltaVector.setAndScale(estimatorDT, baseIMU.getAngularVelocityMeasurement());
      estimatedRootJointRotation.transform(deltaVector);
      predictedBaseOrientation.set(estimatedRootJointRotation);
      predictedBaseOrientation.append(estimatedRootJointRotation);

      // Pack everything into the predicted state vector
      predictedBaseTranslation.get(predictedState);
      predictedBaseLinearVelocity.get(3, predictedState);
      predictedBaseOrientation.get(6, predictedState);

      // Compute the predicted foot position and velocity states
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         int offset = 10 + i * 6;
         // Compute the predicted foot position state
         // p_k+1 = p_k + v_k * dt
         MatrixTools.setMatrixBlock(predictedFootPositionState, 0, 0, state, offset, 0, 3 , 1, 1.0);
         MatrixTools.addMatrixBlock(predictedFootPositionState, 0, 0, state, offset + 3, 0, 3, 1, estimatorDT);
         predictedFootTranslations.get(i).set(predictedFootPositionState);

         // Compute the predicted foot velocity state
         // v_k+1 = v_k + a_k * dt
         // First, get the acceleration of the foot IMU in the base IMU frame, which we're considering here as the root.
         linearAcceleration.setIncludingFrame(yoFootLinearAccelerationMeasurement.get(i));
         linearAcceleration.changeFrame(baseIMU.getMeasurementFrame());
         // Now, get the acceleration in the estimated world frame
         estimatedRootJointRotation.transform(linearAcceleration); // TODO make sure this isn't an inverse transform
         linearAcceleration.get(predictedFootVelocityState);
         CommonOps_DDRM.scale(estimatorDT, predictedFootVelocityState);
         // add to the velocity the current velocity estimate
         MatrixTools.addMatrixBlock(predictedFootVelocityState, 0, 0, state, offset + 3, 0, 3, 1, 1.0);
         predictedFootLinearVelocities.get(i).set(predictedFootVelocityState);

         // Pack into the prediction vector.
         predictedFootTranslations.get(i).get(offset, predictedState);
         predictedFootLinearVelocities.get(i).get(offset + 3, predictedState);
      }

      // return the predicted state
      return predictedState;
   }

   @Override
   public DMatrixRMaj measurementModel(DMatrixRMaj state)
   {
      estimatedRootJointRotation.set(6, state);

      for (int i = 0; i < feetIMUs.size(); i++)
      {
         int measurementOffset = 9 * i;
         int footStateOffset = 10 + i * 6;

         // Compute the predicted foot relative position measurement
         MatrixTools.setMatrixBlock(offsetVector, 0, 0, state, 0, 0, 3 , 1, 1.0);
         MatrixTools.addMatrixBlock(offsetVector, 0, 0, state, footStateOffset, 0, 3 , 1, 1.0);

         predictedRelativeFootPositionMeasurements.get(i).set(offsetVector);
         estimatedRootJointRotation.inverseTransform(predictedRelativeFootPositionMeasurements.get(i)); // TODO make sure this isn't a transform

         // Compute the predicted foot relative velocity measurement
         MatrixTools.setMatrixBlock(offsetVector, 0, 0, state, 3, 0, 3 , 1, 1.0);
         MatrixTools.addMatrixBlock(offsetVector, 0, 0, state, footStateOffset + 3, 0, 3 , 1, 1.0);

         predictedRelativeFootVelocityMeasurements.get(i).set(offsetVector);
         estimatedRootJointRotation.inverseTransform(predictedRelativeFootVelocityMeasurements.get(i)); // TODO make sure this isn't a transform

         // If the foot is in contact, set the velocity to zero, otherwise use the previous prediction state
         if (isFootInContact(i, state))
         {
            predictedWorldFootVelocityMeasurements.get(i).setToZero();
         }
         else
         {
            predictedWorldFootVelocityMeasurements.get(i).set(footStateOffset + 3, state);
         }

         predictedRelativeFootPositionMeasurements.get(i).get(measurementOffset, predictedMeasurement);
         predictedRelativeFootVelocityMeasurements.get(i).get(measurementOffset + 3, predictedMeasurement);
         predictedWorldFootVelocityMeasurements.get(i).get(measurementOffset + 6, predictedMeasurement);
      }

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
