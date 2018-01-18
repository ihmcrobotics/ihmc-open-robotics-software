package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.List;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.filters.FiniteDifferenceAngularVelocityYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * PelvisRotationalStateUpdater reads and transforms the orientation and angular velocity obtained from the IMU to update the pelvis orientation and angular velocity in world. 
 * @author Sylvain
 *
 */
public class IMUBasedPelvisRotationalStateUpdater implements PelvisRotationalStateUpdaterInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameOrientation yoRootJointFrameOrientation;
   private final YoFrameQuaternion yoRootJointFrameQuaternion;
   private final YoDouble rootJointYawOffsetFromFrozenState;

   private final YoFrameVector yoRootJointAngularVelocityMeasFrame;
   private final YoFrameVector yoRootJointAngularVelocity;
   private final YoFrameVector yoRootJointAngularVelocityInWorld;

   private final FiniteDifferenceAngularVelocityYoFrameVector yoRootJointAngularVelocityFromFD;

   private final FloatingInverseDynamicsJoint rootJoint;
   private final ReferenceFrame rootJointFrame;

   private final IMUSensorReadOnly imuProcessedOutput;
   private final IMUBiasProvider imuBiasProvider;
   private final IMUYawDriftEstimator imuYawDriftEstimator;

   private final ReferenceFrame measurementFrame;
   private final RigidBody measurementLink;

   public IMUBasedPelvisRotationalStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, List<? extends IMUSensorReadOnly> imuProcessedOutputs,
         double dt, YoVariableRegistry parentRegistry)
   {
      this(inverseDynamicsStructure, imuProcessedOutputs, null, null, dt, parentRegistry);
   }

   public IMUBasedPelvisRotationalStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, List<? extends IMUSensorReadOnly> imuProcessedOutputs,
         IMUBiasProvider imuBiasProvider, IMUYawDriftEstimator imuYawDriftEstimator, double dt, YoVariableRegistry parentRegistry)
   {
      this.imuBiasProvider = imuBiasProvider;
      this.imuYawDriftEstimator = imuYawDriftEstimator;
      checkNumberOfSensors(imuProcessedOutputs);

      imuProcessedOutput = imuProcessedOutputs.get(0);

      rootJoint = inverseDynamicsStructure.getRootJoint();
      rootJointFrame = rootJoint.getFrameAfterJoint();

      measurementFrame = imuProcessedOutput.getMeasurementFrame();
      measurementLink = imuProcessedOutput.getMeasurementLink();

      yoRootJointFrameOrientation = new YoFrameOrientation("estimatedRootJoint", worldFrame, registry);
      yoRootJointFrameQuaternion = new YoFrameQuaternion("estimatedRootJoint", worldFrame, registry);

      rootJointYawOffsetFromFrozenState = new YoDouble("rootJointYawOffsetFromFrozenState", registry);

      yoRootJointAngularVelocity = new YoFrameVector("estimatedRootJointAngularVelocity", rootJointFrame, registry);
      yoRootJointAngularVelocityInWorld = new YoFrameVector("estimatedRootJointAngularVelocityWorld", worldFrame, registry);
      yoRootJointAngularVelocityMeasFrame = new YoFrameVector("estimatedRootJointAngularVelocityMeasFrame", measurementFrame, registry);

      yoRootJointAngularVelocityFromFD = new FiniteDifferenceAngularVelocityYoFrameVector("estimatedRootJointAngularVelocityFromFD", yoRootJointFrameQuaternion, dt, registry);

      parentRegistry.addChild(registry);

      angularVelocityRootJointFrameRelativeToWorld = new FrameVector3D(rootJointFrame);
   }

   public IMUSensorReadOnly getIMUUsedForEstimation()
   {
      return imuProcessedOutput;
   }

   private void checkNumberOfSensors(List<? extends IMUSensorReadOnly> imuProcessedOutputs)
   {
      if (imuProcessedOutputs.size() > 1)
         System.out.println(getClass().getSimpleName() + ": More than 1 IMU sensor, using only the first one: " + imuProcessedOutputs.get(0).getSensorName());

      if (imuProcessedOutputs.size() == 0)
         throw new RuntimeException("No sensor set up for the IMU.");
   }

   @Override
   public void initialize()
   {
      rotationFrozenOffset.setIdentity();
      updateRootJointOrientationAndAngularVelocity();
   }

   @Override
   public void initializeForFrozenState()
   {
      rotationFrozenOffset.setIdentity();

      // R_{measurementFrame}^{world}
      imuProcessedOutput.getOrientationMeasurement(orientationMeasurement);
      transformFromMeasurementFrameToWorld.setRotationAndZeroTranslation(orientationMeasurement);

      // R_{root}^{measurementFrame}
      rootJointFrame.getTransformToDesiredFrame(transformFromRootJointFrameToMeasurementFrame, measurementFrame);

      // R_{root}^{world} = R_{estimationLink}^{world} * R_{root}^{estimationLink}
      transformFromRootJointFrameToWorld.set(transformFromMeasurementFrameToWorld);
      transformFromRootJointFrameToWorld.multiply(transformFromRootJointFrameToMeasurementFrame);
      transformFromRootJointFrameToWorld.getRotation(rotationFromRootJointFrameToWorld);
      
      double initialYaw = rotationFromRootJointFrameToWorld.getYaw();

      rootJointYawOffsetFromFrozenState.set(initialYaw);
      rotationFrozenOffset.setToYawMatrix(initialYaw);

      yoRootJointFrameQuaternion.setToZero();
      yoRootJointFrameOrientation.setToZero();

      rootJoint.setRotation(yoRootJointFrameQuaternion);

      // Set the rootJoint twist to zero.
      rootJoint.getJointTwist(twistRootBodyRelativeToWorld);
      twistRootBodyRelativeToWorld.setToZero();
      rootJoint.setJointTwist(twistRootBodyRelativeToWorld);
   }

   private final RotationMatrix rotationFrozenOffset = new RotationMatrix();
   private final double[] lastComputedYawPitchRoll = new double[3];

   @Override
   public void updateForFrozenState()
   {
      // R_{measurementFrame}^{world}
      imuProcessedOutput.getOrientationMeasurement(orientationMeasurement);
      transformFromMeasurementFrameToWorld.setRotationAndZeroTranslation(orientationMeasurement);

      // R_{root}^{measurementFrame}
      rootJointFrame.getTransformToDesiredFrame(transformFromRootJointFrameToMeasurementFrame, measurementFrame);

      // R_{root}^{world} = R_{estimationLink}^{world} * R_{root}^{estimationLink}
      transformFromRootJointFrameToWorld.set(transformFromMeasurementFrameToWorld);
      transformFromRootJointFrameToWorld.multiply(transformFromRootJointFrameToMeasurementFrame);
      transformFromRootJointFrameToWorld.getRotation(rotationFromRootJointFrameToWorld);

      yoRootJointFrameQuaternion.getYawPitchRoll(lastComputedYawPitchRoll);
      double currentYaw = rotationFromRootJointFrameToWorld.getYaw();

      double yawDifference = AngleTools.computeAngleDifferenceMinusPiToPi(lastComputedYawPitchRoll[0], currentYaw);
      rootJointYawOffsetFromFrozenState.set(yawDifference);
      rotationFrozenOffset.setToYawMatrix(yawDifference);

      // Keep setting the orientation so that the localization updater works properly.
      rootJoint.setRotation(yoRootJointFrameQuaternion);

      // Set the rootJoint twist to zero.
      rootJoint.getJointTwist(twistRootBodyRelativeToWorld);
      twistRootBodyRelativeToWorld.setToZero();
      rootJoint.setJointTwist(twistRootBodyRelativeToWorld);

      updateViz();
   }

   @Override
   public void updateRootJointOrientationAndAngularVelocity()
   {
      updateRootJointRotation();
      updateRootJointTwistAngularPart();
      updateViz();
   }

   private final RigidBodyTransform transformFromMeasurementFrameToWorld = new RigidBodyTransform();

   private final RigidBodyTransform transformFromRootJointFrameToWorld = new RigidBodyTransform();
   private final RigidBodyTransform transformFromRootJointFrameToMeasurementFrame = new RigidBodyTransform();

   private final RotationMatrix rotationFromRootJointFrameToWorld = new RotationMatrix();
   private final RotationMatrix orientationMeasurement = new RotationMatrix();

   private final RotationMatrix yawBiasMatrix = new RotationMatrix();

   private void updateRootJointRotation()
   {
      // R_{measurementFrame}^{world}
      imuProcessedOutput.getOrientationMeasurement(orientationMeasurement);
      transformFromMeasurementFrameToWorld.setRotationAndZeroTranslation(orientationMeasurement);

      // R_{root}^{measurementFrame}
      rootJointFrame.getTransformToDesiredFrame(transformFromRootJointFrameToMeasurementFrame, measurementFrame);

      // R_{root}^{world} = R_{estimationLink}^{world} * R_{root}^{estimationLink}
      transformFromRootJointFrameToWorld.set(transformFromMeasurementFrameToWorld);
      transformFromRootJointFrameToWorld.multiply(transformFromRootJointFrameToMeasurementFrame);
      transformFromRootJointFrameToWorld.getRotation(rotationFromRootJointFrameToWorld);

      rotationFromRootJointFrameToWorld.preMultiply(rotationFrozenOffset);

      rootJoint.setRotation(rotationFromRootJointFrameToWorld);
      rootJointFrame.update();

      if (imuYawDriftEstimator != null)
      {
         imuYawDriftEstimator.update();
         yawBiasMatrix.setToYawMatrix(imuYawDriftEstimator.getYawBiasInWorldFrame());
         yawBiasMatrix.transpose();
         rotationFromRootJointFrameToWorld.preMultiply(yawBiasMatrix);
      }

      rootJoint.setRotation(rotationFromRootJointFrameToWorld);
      rootJointFrame.update();
   }

   private final Vector3D angularVelocityMeasurement = new Vector3D();
   private final Vector3D angularVelocityMeasurementBias = new Vector3D();

   /** Angular velocity of the measurement link, with respect to world. */
   private final FrameVector3D angularVelocityMeasurementLinkRelativeToWorld = new FrameVector3D();

   /** Angular velocity of the estimation link, with respect to the measurement link. */
   private final FrameVector3D angularVelocityRootJointFrameRelativeToMeasurementLink = new FrameVector3D();

   /** Angular velocity of the root body, with respect to world. */
   private final FrameVector3D angularVelocityRootJointFrameRelativeToWorld;

   /** Twist of the estimation link, with respect to the measurement link. */
   private final Twist twistRootJointFrameRelativeToMeasurementLink = new Twist();
   /** Twist of the root body, with respect to world. */
   private final Twist twistRootBodyRelativeToWorld = new Twist();

   private void updateRootJointTwistAngularPart()
   {
      // T_{rootBody}^{rootBody, measurementLink}
      rootJoint.getSuccessor().getBodyFixedFrame().getTwistRelativeToOther(measurementLink.getBodyFixedFrame(), twistRootJointFrameRelativeToMeasurementLink);
      // T_{rootBody}^{rootJointFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.changeFrame(rootJointFrame);
      // T_{rootJointFrame}^{rootJointFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.changeBodyFrameNoRelativeTwist(rootJointFrame);

      // omega_{rootJointFrame}^{rootJointFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.getAngularPart(angularVelocityRootJointFrameRelativeToMeasurementLink);

      // omega_{measurementLink}^{measurementFrame, world}
      imuProcessedOutput.getAngularVelocityMeasurement(angularVelocityMeasurement);
      if (imuBiasProvider != null)
      {
         imuBiasProvider.getAngularVelocityBiasInIMUFrame(imuProcessedOutput, angularVelocityMeasurementBias);
         angularVelocityMeasurement.sub(angularVelocityMeasurementBias);
      }
      angularVelocityMeasurementLinkRelativeToWorld.setIncludingFrame(measurementFrame, angularVelocityMeasurement);

      // omega_{measurementLink}^{rootJointFrame, world}
      angularVelocityMeasurementLinkRelativeToWorld.changeFrame(rootJointFrame);

      // omega_{rootJointFrame}^{rootJointFrame, world} = omega_{rootJointFrame}^{rootJointFrame, measurementLink} + omega_{measurementLink}^{rootJointFrame, world}
      angularVelocityRootJointFrameRelativeToWorld.add(angularVelocityRootJointFrameRelativeToMeasurementLink, angularVelocityMeasurementLinkRelativeToWorld);

      rootJoint.getJointTwist(twistRootBodyRelativeToWorld);
      twistRootBodyRelativeToWorld.setAngularPart(angularVelocityRootJointFrameRelativeToWorld);
      rootJoint.setJointTwist(twistRootBodyRelativeToWorld);
      rootJoint.updateFramesRecursively();

      yoRootJointAngularVelocity.setAndMatchFrame(angularVelocityMeasurementLinkRelativeToWorld);
      yoRootJointAngularVelocityMeasFrame.setAndMatchFrame(angularVelocityMeasurementLinkRelativeToWorld);
      yoRootJointAngularVelocityInWorld.setAndMatchFrame(angularVelocityRootJointFrameRelativeToWorld);
   }

   private void updateViz()
   {
      yoRootJointFrameQuaternion.checkReferenceFrameMatch(worldFrame);
      yoRootJointFrameQuaternion.set(rotationFromRootJointFrameToWorld);
      yoRootJointAngularVelocityFromFD.update();

      yoRootJointFrameOrientation.checkReferenceFrameMatch(worldFrame);
      yoRootJointFrameOrientation.set(rotationFromRootJointFrameToWorld);
   }

   @Override
   public void getEstimatedOrientation(FrameQuaternion estimatedOrientation)
   {
      estimatedOrientation.set(rotationFromRootJointFrameToWorld);
   }

   @Override
   public void getEstimatedAngularVelocity(FrameVector3D estimatedAngularVelocityToPack)
   {
      estimatedAngularVelocityToPack.setIncludingFrame(angularVelocityRootJointFrameRelativeToWorld);
   }
}
