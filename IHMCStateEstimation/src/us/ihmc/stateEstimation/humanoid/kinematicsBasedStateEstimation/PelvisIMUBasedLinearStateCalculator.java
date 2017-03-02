package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.List;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class PelvisIMUBasedLinearStateCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable cancelGravityFromAccelerationMeasurement = new BooleanYoVariable("cancelGravityFromAccelerationMeasurement", registry);

   private final FrameVector accelerationBias = new FrameVector(worldFrame);
   private final FrameVector gravityVector = new FrameVector();

   private final YoFrameVector rootJointLinearVelocity = new YoFrameVector("imuRootJointLinearVelocity", worldFrame, registry);
   private final YoFrameVector rootJointPosition = new YoFrameVector("imuRootJointPosition", worldFrame, registry);
   private final BooleanYoVariable setRootJointPositionImuOnlyToCurrent = new BooleanYoVariable("setRootJointPositionImuOnlyToCurrent", registry);
   private final DoubleYoVariable alphaLeakIMUOnly = new DoubleYoVariable("imuOnlyAlphaLeak", registry);
   private final YoFrameVector rootJointPositionImuOnly = new YoFrameVector("imuOnlyIntregratedRootJointPosition", worldFrame, registry);
   private final YoFrameVector imuLinearVelocityIMUOnly = new YoFrameVector("imuOnlyIntegratedIMULinearVelocity", worldFrame, registry);
   private final YoFrameVector rootJointLinearVelocityIMUOnly = new YoFrameVector("imuOnlyIntegratedRootJointLinearVelocity", worldFrame, registry);

   private final YoFrameVector yoMeasurementFrameLinearVelocityInWorld;
   private final YoFrameVector yoRootJointIMUBasedLinearVelocityInWorld;
   private final YoFrameVector yoLinearAccelerationMeasurementInWorld;
   private final YoFrameVector yoLinearAccelerationMeasurement;

   private final BooleanYoVariable imuBasedStateEstimationEnabled = new BooleanYoVariable("imuBasedStateEstimationEnabled", registry);

   private final ReferenceFrame measurementFrame;

   private final FloatingInverseDynamicsJoint rootJoint;

   private final double estimatorDT;

   // Temporary variables
   private final FrameVector linearAccelerationMeasurement = new FrameVector();
   private final FrameVector tempRootJointVelocityIntegrated = new FrameVector();

   private final IMUSensorReadOnly imuProcessedOutput;
   private final IMUBiasProvider imuBiasProvider;

   public PelvisIMUBasedLinearStateCalculator(FullInverseDynamicsStructure inverseDynamicsStructure, List<? extends IMUSensorReadOnly> imuProcessedOutputs, IMUBiasProvider imuBiasProvider, double estimatorDT,
         double gravitationalAcceleration, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.imuBiasProvider = imuBiasProvider;
      this.estimatorDT = estimatorDT;
      this.rootJoint = inverseDynamicsStructure.getRootJoint();

      gravityVector.setIncludingFrame(worldFrame, 0.0, 0.0, -Math.abs(gravitationalAcceleration));

      if (imuProcessedOutputs.size() == 0)
      {
         imuProcessedOutput = null;
         imuBasedStateEstimationEnabled.set(false);
      }
      else
      {
         if (imuProcessedOutputs.size() > 1)
            System.out.println(getClass().getSimpleName() + ": More than 1 IMU sensor, using only the first one: " + imuProcessedOutputs.get(0).getSensorName());
         imuProcessedOutput = imuProcessedOutputs.get(0);
         imuBasedStateEstimationEnabled.set(true);
      }

      if (imuBasedStateEstimationEnabled.getBooleanValue())
      {
         measurementFrame = imuProcessedOutput.getMeasurementFrame();
      }
      else
      {
         measurementFrame = null;
      }

      yoMeasurementFrameLinearVelocityInWorld = new YoFrameVector("imuLinearVelocityInWorld", worldFrame, registry);
      yoRootJointIMUBasedLinearVelocityInWorld = new YoFrameVector("rootJointIMUBasedLinearVelocityInWorld", worldFrame, registry);
      yoLinearAccelerationMeasurement = new YoFrameVector("imuLinearAcceleration", measurementFrame, registry);
      yoLinearAccelerationMeasurementInWorld = new YoFrameVector("imuLinearAccelerationInWorld", worldFrame, registry);

      setRootJointPositionImuOnlyToCurrent.set(true);
      alphaLeakIMUOnly.set(0.999);

      parentRegistry.addChild(registry);
   }

   public void enableEstimationModule(boolean enable)
   {
      if (imuProcessedOutput != null)
         imuBasedStateEstimationEnabled.set(enable);
   }

   public void cancelGravityFromAccelerationMeasurement(boolean cancel)
   {
      cancelGravityFromAccelerationMeasurement.set(cancel);
   }

   public boolean isEstimationEnabled()
   {
      return imuBasedStateEstimationEnabled.getBooleanValue();
   }

   private final FrameVector tempVector = new FrameVector();

   private void updateLinearAccelerationMeasurement()
   {
      if (!isEstimationEnabled())
         return;

      imuBiasProvider.getLinearAccelerationBiasInIMUFrame(imuProcessedOutput, accelerationBias);

      linearAccelerationMeasurement.setToZero(measurementFrame);
      imuProcessedOutput.getLinearAccelerationMeasurement(linearAccelerationMeasurement.getVector());
      linearAccelerationMeasurement.sub(accelerationBias);

      // Update acceleration in world (minus gravity)
      linearAccelerationMeasurement.changeFrame(worldFrame);
      if (cancelGravityFromAccelerationMeasurement.getBooleanValue())
      {
         tempVector.setIncludingFrame(gravityVector);
         tempVector.changeFrame(linearAccelerationMeasurement.getReferenceFrame());
         linearAccelerationMeasurement.add(tempVector);
      }
      yoLinearAccelerationMeasurementInWorld.set(linearAccelerationMeasurement);

      // Update acceleration in local frame (minus gravity)
      linearAccelerationMeasurement.setToZero(measurementFrame);
      imuProcessedOutput.getLinearAccelerationMeasurement(linearAccelerationMeasurement.getVector());
      linearAccelerationMeasurement.sub(accelerationBias);

      if (cancelGravityFromAccelerationMeasurement.getBooleanValue())
      {
         tempVector.setIncludingFrame(gravityVector);
         tempVector.changeFrame(linearAccelerationMeasurement.getReferenceFrame());
         linearAccelerationMeasurement.add(tempVector);
      }
      yoLinearAccelerationMeasurement.set(linearAccelerationMeasurement);
   }

   private final FrameVector correctionVelocityForMeasurementFrameOffset = new FrameVector();

   public void updateIMUAndRootJointLinearVelocity(FrameVector rootJointVelocityToPack)
   {
      updateLinearAccelerationMeasurement();

      yoLinearAccelerationMeasurementInWorld.getFrameTupleIncludingFrame(linearAccelerationMeasurement);
      linearAccelerationMeasurement.scale(estimatorDT);

      yoMeasurementFrameLinearVelocityInWorld.add(linearAccelerationMeasurement);
      yoMeasurementFrameLinearVelocityInWorld.getFrameTupleIncludingFrame(rootJointVelocityToPack);

      getCorrectionVelocityForMeasurementFrameOffset(correctionVelocityForMeasurementFrameOffset);
      correctionVelocityForMeasurementFrameOffset.changeFrame(worldFrame);
      rootJointVelocityToPack.sub(correctionVelocityForMeasurementFrameOffset);
      yoRootJointIMUBasedLinearVelocityInWorld.set(rootJointVelocityToPack);

      imuLinearVelocityIMUOnly.add(linearAccelerationMeasurement);
      imuLinearVelocityIMUOnly.scale(alphaLeakIMUOnly.getDoubleValue());
      rootJointLinearVelocityIMUOnly.set(imuLinearVelocityIMUOnly);
      rootJointLinearVelocityIMUOnly.sub(correctionVelocityForMeasurementFrameOffset);
   }

   public void correctIMULinearVelocity(FrameVector rootJointVelocity)
   {
      rootJointLinearVelocity.set(rootJointVelocity);
      yoMeasurementFrameLinearVelocityInWorld.set(rootJointVelocity);
      yoMeasurementFrameLinearVelocityInWorld.add(correctionVelocityForMeasurementFrameOffset);
   }

   public void updatePelvisPosition(FramePoint rootJointPositionPrevValue, FramePoint rootJointPositionToPack)
   {
      if (!isEstimationEnabled())
         throw new RuntimeException("IMU estimation module for pelvis linear velocity is disabled.");

      rootJointLinearVelocity.getFrameTupleIncludingFrame(tempRootJointVelocityIntegrated);
      tempRootJointVelocityIntegrated.scale(estimatorDT);

      if(setRootJointPositionImuOnlyToCurrent.getBooleanValue())
      {
         rootJointPositionImuOnly.set(rootJointPositionPrevValue);
         setRootJointPositionImuOnlyToCurrent.set(false);
      }
      rootJointPositionImuOnly.add(tempRootJointVelocityIntegrated);

      rootJointPosition.set(rootJointPositionPrevValue);
      rootJointPosition.add(tempRootJointVelocityIntegrated);
      rootJointPosition.getFrameTupleIncludingFrame(rootJointPositionToPack);
   }

   private final Twist tempRootJointTwist = new Twist();
   private final FrameVector tempRootJointAngularVelocity = new FrameVector();
   private final FramePoint measurementOffset = new FramePoint();

   private void getCorrectionVelocityForMeasurementFrameOffset(FrameVector correctionTermToPack)
   {
      rootJoint.getJointTwist(tempRootJointTwist);
      tempRootJointTwist.getAngularPart(tempRootJointAngularVelocity);

      measurementOffset.setToZero(measurementFrame);
      measurementOffset.changeFrame(rootJoint.getFrameAfterJoint());

      correctionTermToPack.setToZero(tempRootJointAngularVelocity.getReferenceFrame());
      correctionTermToPack.cross(tempRootJointAngularVelocity, measurementOffset);
   }
}
