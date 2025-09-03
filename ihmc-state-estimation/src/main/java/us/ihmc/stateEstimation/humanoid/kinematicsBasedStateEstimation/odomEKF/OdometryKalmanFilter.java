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
import us.ihmc.euclid.tuple4D.Quaternion;
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
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.odomEKF.OdometryIndexHelper.*;

/**
 * The process model in this class is defined as
 */
public class OdometryKalmanFilter extends ExtendedKalmanFilter
{
   static final boolean includeBias = true;
   static final boolean usePredictedStateInJacobian = true;

   // Constants and providers
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final IMUSensorReadOnly baseIMU;
   private final List<? extends IMUSensorReadOnly> feetIMUs;
   private final IMUBiasProvider imuBiasProvider;

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

   private final SensedVariables baseSensing;
   private final List<SensedVariables> feetSensing = new ArrayList<>();

   private final StateVariables baseProcessState;
   private final StateVariables basePredictedState;
   private final StateCorrectionVariables baseStateCorrection;
   private final List<StateVariables> feetProcessState = new ArrayList<>();
   private final List<StateVariables> feetPredictedState = new ArrayList<>();
   private final List<StateCorrectionVariables> feetStateCorrections = new ArrayList<>();
   private final List<MeasurementResidualVariables> feetMeasurementResiduals = new ArrayList<>();
   private final List<MeasurementVariables> feetMeasurements = new ArrayList<>();

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
      super(stateSizePerLink * (1 + feetIMUs.size()), errorSizePerLink * (1 + feetIMUs.size()) , measurementSizePerLink * feetIMUs.size());

      // Initialize covariance
      processCovariance = new DMatrixRMaj(getErrorSize(), getErrorSize());
      measurementCovariance = new DMatrixRMaj(getMeasurementSize(), getMeasurementSize());
      MatrixTools.setDiagonal(processCovariance, 1e-2);
      MatrixTools.setDiagonal(measurementCovariance, 1e-3);

      translationCovariance.set(1e-5);
      velocityCovariance.set(1e-2);
      orientationCovariance.set(1e-2);
      biasCovariance.set(1e-4);

      measuredTranslationCovariance.set(1e-6);
      measuredOrientationCovariance.set(1e-5);
      velocityErrorCovariance.set(1e-5);
      contactVelocityCovariance.set(1e-10);
      contactAccelerationCovariance.set(1.0);

      contactThreshold.set(Double.MAX_VALUE);

      this.baseIMU = baseIMU;
      this.feetIMUs = feetIMUs;
      this.imuBiasProvider = baseImuBiasProvider;

      predictedState = new DMatrixRMaj(getStateSize(), 1);
      predictedMeasurement = new DMatrixRMaj(getMeasurementSize(), 1);
      AMatrix = new DMatrixRMaj(getErrorSize(), getErrorSize());
      CMatrix = new DMatrixRMaj(getMeasurementSize(), getErrorSize());
      observationVector = new DMatrixRMaj(getMeasurementSize(), 1);

      gravityVector = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(() -> worldFrame, new Vector3D(0, 0, Math.abs(gravitationalAcceleration)));

      baseSensing = new SensedVariables("base", baseIMU, baseIMU, imuBiasProvider, registry);
      baseProcessState = new StateVariables("baseProcessState", baseIMU.getMeasurementFrame(), registry);
      basePredictedState = new StateVariables("basePredicted",  baseIMU.getMeasurementFrame(), registry);
      baseStateCorrection = new StateCorrectionVariables("baseStateCorrection", baseIMU.getMeasurementFrame(), registry);
      processModels.add(new ProcessModel(baseProcessState,
                                         basePredictedState,
                                         cancelGravityFromAccelerationMeasurement,
                                         gravityVector,
                                         estimatorDT));
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         SensedVariables footSensing = new SensedVariables("foot" + i, baseIMU, feetIMUs.get(i), imuBiasProvider, registry);
         StateVariables footProcessState = new StateVariables("foot" + i + "ProcessState", feetIMUs.get(i).getMeasurementFrame(), registry);
         StateVariables footPredictedState = new StateVariables("foot" + i + "PredictedState", feetIMUs.get(i).getMeasurementFrame(), registry);
         StateCorrectionVariables footStateCorrection = new StateCorrectionVariables("foot" + i + "StateCorrection", feetIMUs.get(i).getMeasurementFrame(), registry);
         MeasurementVariables footMeasurement = new MeasurementVariables("foot" + i + "Predicted", registry);
         MeasurementResidualVariables footMeasurementResidual = new MeasurementResidualVariables("foot" + i + "Residual", registry);
         feetSensing.add(footSensing);
         feetMeasurements.add(footMeasurement);
         feetProcessState.add(footProcessState);
         feetPredictedState.add(footPredictedState);
         feetStateCorrections.add(footStateCorrection);
         feetMeasurementResiduals.add(footMeasurementResidual);

         observationModels.add(new ObservationModel(footSensing));
         processModels.add(new ProcessModel(footProcessState,
                                            footPredictedState,
                                            cancelGravityFromAccelerationMeasurement,
                                            gravityVector,
                                            estimatorDT));
         measurementModels.add(new MeasurementModel("foot" + i,
                                                    basePredictedState,
                                                    footPredictedState,
                                                    footSensing,
                                                    footMeasurement,
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
      baseProcessState.get(stateTranslationIndex, state);
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         feetProcessState.get(i).get(OdometryIndexHelper.getFootPositionIndex(i), state);
      }

      initializeState(state);
   }

   private boolean initialized = false;

   public void compute()
   {
      updateCovariance();

      baseSensing.update();
      for (int i = 0; i < feetSensing.size(); i++)
         feetSensing.get(i).update();

      for (int i = 0; i < observationModels.size(); i++)
      {
         observationModels.get(i).get(i * measurementSizePerLink, observationVector);
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
         int index = i * errorSizePerLink;
         OdometryTools.setDiagonals(index + errorTranslationIndex, 3, translationCovariance.getValue(), processCovariance);
         OdometryTools.setDiagonals(index + errorLinearVelocityIndex, 3, velocityCovariance.getValue(), processCovariance);
         OdometryTools.setDiagonals(index + errorOrientationIndex, 3, orientationCovariance.getValue(), processCovariance);
         OdometryTools.setDiagonals(index + errorAccelBiasIndex, 3, biasCovariance.getValue(), processCovariance);
         OdometryTools.setDiagonals(index + errorGyroBiasIndex, 3, biasCovariance.getValue(), processCovariance);
      }

      for (int i = 0; i < feetIMUs.size(); i++)
      {
         int index = i * measurementSizePerLink;
         OdometryTools.setDiagonals(index + measurementRelativeTranslationIndex, 3, measuredTranslationCovariance.getValue(), measurementCovariance);
         OdometryTools.setDiagonals(index + measurementRelativeOrientationErrorIndex, 3, measuredOrientationCovariance.getValue(), measurementCovariance);
         OdometryTools.setDiagonals(index + measurementRelativeVelocityIndex, 3, velocityErrorCovariance.getValue(), measurementCovariance);
         OdometryTools.setDiagonals(index + measurementContactVelocityIndex, 3, contactVelocityCovariance.getValue(), measurementCovariance);
         OdometryTools.setDiagonals(index + measurementAccelIndex, 3, contactAccelerationCovariance.getValue(), measurementCovariance);
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
   protected void postSolveHook()
   {
      // Update some yo variables
      baseStateCorrection.set(0, stateCorrection);
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         feetStateCorrections.get(i).set((i + 1) * errorSizePerLink, stateCorrection);
         feetMeasurementResiduals.get(i).set(i * measurementSizePerLink, measurementResidual);
      }
   }

   @Override
   protected void applyStateCorrection(DMatrixRMaj state, DMatrixRMaj correction, DMatrixRMaj correctedState)
   {
      correctedState.set(state);
      for (int i = 0; i < feetIMUs.size() + 1; i++)
      {
         int destStart = stateSizePerLink * i;
         int sourceStart = errorSizePerLink * i;
         // Update the translation
         MatrixTools.addMatrixBlock(correctedState, destStart + stateTranslationIndex, 0, correction, sourceStart + errorTranslationIndex, 0, 3, 1, 1.0);
         // Update the linear velocity
         MatrixTools.addMatrixBlock(correctedState, destStart + stateLinearVelocityIndex, 0, correction, sourceStart + errorLinearVelocityIndex, 0, 3, 1, 1.0);
         // Update the orientation
         // FIXME garbage
         Vector3D rotationVector = new Vector3D();
         rotationVector.set(sourceStart + errorOrientationIndex, correction);
         Quaternion correctionRotation = new Quaternion(rotationVector);
         Quaternion rotation = new Quaternion();
         rotation.set(destStart + stateOrientationIndex, state);
         rotation.multiply(correctionRotation);
         rotation.get(destStart + stateOrientationIndex, correctedState);
         // Update the accel bias
         MatrixTools.addMatrixBlock(correctedState, destStart + stateAccelBiasIndex, 0, correction, sourceStart + errorAccelBiasIndex, 0, 3, 1, 1.0);
         // Update the gyro bias
         MatrixTools.addMatrixBlock(correctedState, destStart + stateGyroBiasIndex, 0, correction, sourceStart + errorGyroBiasIndex, 0, 3, 1, 1.0);
      }
   }

   @Override
   public DMatrixRMaj processModel(DMatrixRMaj state)
   {
      predictedState.zero();

      // update the yo variables representing the state
      baseProcessState.set(stateTranslationIndex, state, baseSensing);
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         feetProcessState.get(i).set(OdometryIndexHelper.getFootPositionIndex(i), state, feetSensing.get(i));
      }

      // do the prediction
      for (int i = 0; i < processModels.size(); i++)
         processModels.get(i).update();

      // update the predicted state from the yo variablized state
      basePredictedState.get(stateTranslationIndex, predictedState);
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
         int row = errorSizePerLink * (i + 1) + measurementContactVelocityIndex;
         MatrixTools.setMatrixBlock(footCovariance, 0, 0, getCovariance(), row, row, 3, 3, 1.0);

         measurementModels.get(i).update(footCovariance);
         feetMeasurements.get(i).get(measurementModels.get(i).getIsFootInContact(), i * measurementSizePerLink, predictedMeasurement);
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

      for (int i = 0; i < feetIMUs.size() + 1; i++)
      {
         processModels.get(i).computeProcessJacobian(offset, AMatrix);
         offset += errorSizePerLink;
      }

      return AMatrix;
   }

   @Override
   protected DMatrixRMaj linearizeMeasurementModel(DMatrixRMaj predictedState)
   {
      // make sure to call zero, as the compute process model stuff has a bunch of add operators.
      CMatrix.zero();
      int rowOffset = 0;
      int colOffset = errorSizePerLink;
      for (int i = 0; i < feetIMUs.size(); i++)
      {
         MeasurementModel measurementModel = measurementModels.get(i);
         measurementModel.computeBaseMeasurementJacobian(rowOffset, CMatrix);
         measurementModel.computeFootMeasurementJacobian(rowOffset, colOffset, CMatrix);
         rowOffset += measurementSizePerLink;
         colOffset += errorSizePerLink;
      }

      return CMatrix;
   }
}
