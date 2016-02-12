package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameQuaternion;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class IMUOrientationBiasEstimator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final IMUSensorReadOnly imuToCheck;
   private final DoubleYoVariable alphaFilterLinearAcceleration = new DoubleYoVariable("alphaFilterLinearAcceleration", registry);
   private final AlphaFilteredYoFrameVector linearAccelerationFiltered;
   private final YoFrameQuaternion estimatedQuaternionBias;
   private final DoubleYoVariable alphaFilterQuaternionBias = new DoubleYoVariable("alphaFilterQuaternionBias", registry);
   private final AlphaFilteredYoFrameQuaternion estimatedQuaternionBiasFiltered;
   private final YoFrameOrientation estimatedOrientationBiasFiltered;

   private final YoFrameQuaternion estimatedRootJointQuaternionUnbiased;
   private final YoFrameQuaternion estimatedRootJointOrientationUnbiased;

   private final SixDoFJoint rootJoint;

   private final Vector3d linearAccelerationVector3d = new Vector3d();
   private final FrameVector linearAccelerationFrameVector = new FrameVector();
   private final FrameVector zDirection = new FrameVector(worldFrame, 0.0, 0.0, 1.0);
   private final FrameVector biasRotationAxis = new FrameVector();
   private final AxisAngle4d biasAxisAngle = new AxisAngle4d();
   private final Matrix3d rootJointRotation = new Matrix3d();
   private final Matrix3d biasRotation = new Matrix3d();

   public IMUOrientationBiasEstimator(FullInverseDynamicsStructure inverseDynamicsStructure, IMUSensorReadOnly imuToCheck, double updateDT, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      this.rootJoint = inverseDynamicsStructure.getRootJoint();
      this.imuToCheck = imuToCheck;

      alphaFilterLinearAcceleration.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(0.5, updateDT));
      linearAccelerationFiltered = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(imuToCheck.getSensorName() + "LinearAccelerationFiltered", "", registry, alphaFilterLinearAcceleration, worldFrame);
      estimatedQuaternionBias = new YoFrameQuaternion(imuToCheck.getSensorName() + "EstimatedBias", worldFrame, registry);
      alphaFilterQuaternionBias.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(0.5, updateDT));
      estimatedQuaternionBiasFiltered = new AlphaFilteredYoFrameQuaternion(imuToCheck.getSensorName() + "EstimatedBiasFiltered", "", estimatedQuaternionBias, alphaFilterQuaternionBias, registry);
      estimatedOrientationBiasFiltered = new YoFrameOrientation(imuToCheck.getSensorName() + "EstimatedBias", worldFrame, registry);

      estimatedRootJointQuaternionUnbiased = new YoFrameQuaternion("estimatedRootJointQuaternionUnbiased", worldFrame, registry);
      estimatedRootJointOrientationUnbiased = new YoFrameQuaternion("estimatedRootJointOrientationUnbiased", worldFrame, registry);
   }

   public void compute(RobotMotionStatus robotMotionStatus)
   {
      if (robotMotionStatus == RobotMotionStatus.STANDING)
      {
         ReferenceFrame imuFrame = imuToCheck.getMeasurementFrame();
         imuToCheck.getLinearAccelerationMeasurement(linearAccelerationVector3d);
         linearAccelerationFrameVector.setIncludingFrame(imuFrame, linearAccelerationVector3d);
         linearAccelerationFrameVector.changeFrame(worldFrame);
         linearAccelerationFiltered.update(linearAccelerationFrameVector);
         linearAccelerationFiltered.getFrameTupleIncludingFrame(linearAccelerationFrameVector);
         linearAccelerationFrameVector.normalize();
         double angleFromBias = linearAccelerationFrameVector.angle(zDirection);
         biasRotationAxis.cross(linearAccelerationFrameVector, zDirection);
         biasAxisAngle.set(biasRotationAxis.getVector(), angleFromBias);
         estimatedQuaternionBias.set(biasAxisAngle);
         estimatedQuaternionBiasFiltered.update();
         estimatedOrientationBiasFiltered.set(estimatedQuaternionBias);
         
         estimatedOrientationBiasFiltered.getMatrix3d(biasRotation);
         rootJoint.packRotation(rootJointRotation);
         rootJointRotation.mul(biasRotation, rootJointRotation);
         
         estimatedRootJointQuaternionUnbiased.set(rootJointRotation);
         estimatedRootJointOrientationUnbiased.set(rootJointRotation);
      }
      else
      {
         
      }
   }}
