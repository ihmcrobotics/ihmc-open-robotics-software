package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.stateEstimation.IMUSelectorAndDataConverter;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.OrientationStateRobotModelUpdater;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFrameQuaternion;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


/**
 * PelvisRotationalStateUpdater reads and transforms the orientation and angular velocity obtained from the IMU to update the pelvis orientation and angular velocity in world. 
 * (Based on {@link IMUSelectorAndDataConverter} and {@link OrientationStateRobotModelUpdater})
 * @author Sylvain
 *
 */
public class PelvisRotationalStateUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameOrientation yoRootJointFrameOrientation;
   private final YoFrameQuaternion yoRootJointFrameQuaternion;

   private final YoFrameVector measurementFrameAngularVelocity;
   private final YoFrameVector measurementFrameAngularVelocityInWorld;
   private final DoubleYoVariable rootJointYawOffsetFromFrozenState;
   
   private final SixDoFJoint rootJoint;
   private final ReferenceFrame rootJointFrame;
   private final TwistCalculator twistCalculator;
   
   private final IMUSensorReadOnly imuProcessedOutput;
   
   private final ReferenceFrame measurementFrame;
   private final RigidBody measurementLink;

   public PelvisRotationalStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure, List<? extends IMUSensorReadOnly> imuProcessedOutputs,
         YoVariableRegistry parentRegistry)
   {
      checkNumberOfSensors(imuProcessedOutputs);

      imuProcessedOutput = imuProcessedOutputs.get(0);
      
      rootJoint = inverseDynamicsStructure.getRootJoint();
      rootJointFrame = rootJoint.getFrameAfterJoint();
      twistCalculator = inverseDynamicsStructure.getTwistCalculator();

      this.measurementFrame = imuProcessedOutput.getMeasurementFrame();
      this.measurementLink = imuProcessedOutput.getMeasurementLink();

      yoRootJointFrameOrientation = new YoFrameOrientation("estimatedRootJointFrame", worldFrame, registry);
      yoRootJointFrameQuaternion = new YoFrameQuaternion("estimatedRootJointFrame", worldFrame, registry);
      measurementFrameAngularVelocity = new YoFrameVector("measFrameAngularVelocity", measurementFrame, registry);
      measurementFrameAngularVelocityInWorld = new YoFrameVector("measFrameAngularVelocityWorld", worldFrame, registry);
      rootJointYawOffsetFromFrozenState = new DoubleYoVariable("rootJointYawOffsetFromFrozenState", registry);
      
      parentRegistry.addChild(registry);
      
      angularVelocityRootJointFrameRelativeToWorld = new FrameVector(rootJointFrame);
   }

   private void checkNumberOfSensors(List<? extends IMUSensorReadOnly> imuProcessedOutputs)
   {
      if (imuProcessedOutputs.size() > 1)
         System.out.println(getClass().getSimpleName() + ": More than 1 IMU sensor, using only the first one: " + imuProcessedOutputs.get(0).getSensorName());
      
      if (imuProcessedOutputs.size() == 0)
         throw new RuntimeException("No sensor set up for the IMU.");
   }

   public void initialize()
   {
      rotationFrozenOffset.setIdentity();

      updateRootJointOrientationAndAngularVelocity();
   }

   private final Matrix3d rotationFrozenOffset = new Matrix3d();
   private final double[] lastComputedYawPitchRoll = new double[3];

   public void updateForFrozenState()
   {
      // R_{measurementFrame}^{world}
      imuProcessedOutput.getOrientationMeasurement(orientationMeasurement);
      transformFromMeasurementFrameToWorld.setRotationAndZeroTranslation(orientationMeasurement);

      // R_{root}^{measurementFrame}
      rootJointFrame.getTransformToDesiredFrame(transformFromRootJointFrameToMeasurementFrame, measurementFrame);

      // R_{root}^{world} = R_{estimationLink}^{world} * R_{root}^{estimationLink}
      transformFromRootJointFrameToWorld.multiply(transformFromMeasurementFrameToWorld, transformFromRootJointFrameToMeasurementFrame);
      transformFromRootJointFrameToWorld.get(rotationFromRootJointFrameToWorld);


      yoRootJointFrameQuaternion.getYawPitchRoll(lastComputedYawPitchRoll);
      double currentYaw = RotationFunctions.getYaw(rotationFromRootJointFrameToWorld);

      double yawDifference = AngleTools.computeAngleDifferenceMinusPiToPi(lastComputedYawPitchRoll[0], currentYaw);
      rootJointYawOffsetFromFrozenState.set(yawDifference);
      rotationFrozenOffset.rotZ(yawDifference);

      // Keep setting the orientation so that the localization updater works properly.
      yoRootJointFrameQuaternion.get(rotationFromRootJointFrameToWorld);
      rootJoint.setRotation(rotationFromRootJointFrameToWorld);

      // Set the rootJoint twist to zero.
      rootJoint.packJointTwist(twistRootBodyRelativeToWorld);
      twistRootBodyRelativeToWorld.setToZero();
      rootJoint.setJointTwist(twistRootBodyRelativeToWorld);

      updateViz();
   }

   public void updateRootJointOrientationAndAngularVelocity()
   {
      updateRootJointRotation();
      updateRootJointTwistAngularPart();
      updateViz();
   }

   private final RigidBodyTransform transformFromMeasurementFrameToWorld = new RigidBodyTransform();
   
   private final RigidBodyTransform transformFromRootJointFrameToWorld = new RigidBodyTransform();
   private final RigidBodyTransform transformFromRootJointFrameToMeasurementFrame = new RigidBodyTransform();

   private final Matrix3d rotationFromRootJointFrameToWorld = new Matrix3d();
   private final Matrix3d orientationMeasurement = new Matrix3d();

   private void updateRootJointRotation()
   {
      // R_{measurementFrame}^{world}
      imuProcessedOutput.getOrientationMeasurement(orientationMeasurement);
      transformFromMeasurementFrameToWorld.setRotationAndZeroTranslation(orientationMeasurement);

      // R_{root}^{measurementFrame}
      rootJointFrame.getTransformToDesiredFrame(transformFromRootJointFrameToMeasurementFrame, measurementFrame);

      // R_{root}^{world} = R_{estimationLink}^{world} * R_{root}^{estimationLink}
      transformFromRootJointFrameToWorld.multiply(transformFromMeasurementFrameToWorld, transformFromRootJointFrameToMeasurementFrame);
      transformFromRootJointFrameToWorld.get(rotationFromRootJointFrameToWorld);

      rotationFromRootJointFrameToWorld.mul(rotationFrozenOffset, rotationFromRootJointFrameToWorld);

      rootJoint.setRotation(rotationFromRootJointFrameToWorld);
      rootJointFrame.update();
   }

   private final Vector3d angularVocityMeasurement = new Vector3d();
   
   /** Angular velocity of the measurement link, with respect to world. */
   private final FrameVector angularVelocityMeasurementLinkRelativeToWorld = new FrameVector();

   /** Angular velocity of the estimation link, with respect to the measurement link. */
   private final FrameVector angularVelocityRootJointFrameRelativeToMeasurementLink = new FrameVector();

   /** Angular velocity of the root body, with respect to world. */
   private final FrameVector angularVelocityRootJointFrameRelativeToWorld;

   /** Twist of the estimation link, with respect to the measurement link. */
   private final Twist twistRootJointFrameRelativeToMeasurementLink = new Twist();
   /** Twist of the root body, with respect to world. */
   private final Twist twistRootBodyRelativeToWorld = new Twist();

   private void updateRootJointTwistAngularPart()
   {
      // T_{rootBody}^{rootBody, measurementLink}
      twistCalculator.packRelativeTwist(twistRootJointFrameRelativeToMeasurementLink, measurementLink, rootJoint.getSuccessor());
      // T_{rootBody}^{rootJointFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.changeFrame(rootJointFrame);
      // T_{rootJointFrame}^{rootJointFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.changeBodyFrameNoRelativeTwist(rootJointFrame);
      
      // omega_{rootJointFrame}^{rootJointFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.packAngularPart(angularVelocityRootJointFrameRelativeToMeasurementLink);

      // omega_{measurementLink}^{measurementFrame, world}
      imuProcessedOutput.getAngularVelocityMeasurement(angularVocityMeasurement);
      angularVelocityMeasurementLinkRelativeToWorld.setIncludingFrame(measurementFrame, angularVocityMeasurement); 
      measurementFrameAngularVelocity.set(angularVelocityMeasurementLinkRelativeToWorld);

      // omega_{measurementLink}^{rootJointFrame, world}
      angularVelocityMeasurementLinkRelativeToWorld.changeFrame(rootJointFrame);

      // omega_{rootJointFrame}^{rootJointFrame, world} = omega_{rootJointFrame}^{rootJointFrame, measurementLink} + omega_{measurementLink}^{rootJointFrame, world}
      angularVelocityRootJointFrameRelativeToWorld.add(angularVelocityRootJointFrameRelativeToMeasurementLink, angularVelocityMeasurementLinkRelativeToWorld);
      
      angularVelocityMeasurementLinkRelativeToWorld.changeFrame(worldFrame);
      measurementFrameAngularVelocityInWorld.set(angularVelocityMeasurementLinkRelativeToWorld);

      rootJoint.packJointTwist(twistRootBodyRelativeToWorld);
      twistRootBodyRelativeToWorld.setAngularPart(angularVelocityRootJointFrameRelativeToWorld);
      rootJoint.setJointTwist(twistRootBodyRelativeToWorld);
      twistCalculator.compute();
   }

   private void updateViz()
   {
      yoRootJointFrameQuaternion.checkReferenceFrameMatch(worldFrame);
      yoRootJointFrameQuaternion.set(rotationFromRootJointFrameToWorld);

      yoRootJointFrameOrientation.checkReferenceFrameMatch(worldFrame);
      yoRootJointFrameOrientation.set(rotationFromRootJointFrameToWorld);
   }

   public void getEstimatedOrientation(FrameOrientation estimatedOrientation)
   {
      estimatedOrientation.set(rotationFromRootJointFrameToWorld);
   }

   public void getEstimatedAngularVelocity(FrameVector estimatedAngularVelocityToPack)
   {
      estimatedAngularVelocityToPack.setIncludingFrame(angularVelocityRootJointFrameRelativeToWorld);
   }
}
