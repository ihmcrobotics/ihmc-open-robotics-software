package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class PelvisIMUBasedLinearStateCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean cancelGravityFromAccelerationMeasurement = new YoBoolean("cancelGravityFromAccelerationMeasurement", registry);

   private final FrameVector3D accelerationBias = new FrameVector3D(worldFrame);
   private final FrameVector3D gravityVector = new FrameVector3D();

   private final YoFrameVector rootJointLinearVelocity = new YoFrameVector("imuRootJointLinearVelocity", worldFrame, registry);
   private final YoFrameVector rootJointPosition = new YoFrameVector("imuRootJointPosition", worldFrame, registry);
   private final YoBoolean setRootJointPositionImuOnlyToCurrent = new YoBoolean("setRootJointPositionImuOnlyToCurrent", registry);
   private final YoDouble alphaLeakIMUOnly = new YoDouble("imuOnlyAlphaLeak", registry);
   private final YoFrameVector rootJointPositionImuOnly = new YoFrameVector("imuOnlyIntregratedRootJointPosition", worldFrame, registry);
   private final YoFrameVector imuLinearVelocityIMUOnly = new YoFrameVector("imuOnlyIntegratedIMULinearVelocity", worldFrame, registry);
   private final YoFrameVector rootJointLinearVelocityIMUOnly = new YoFrameVector("imuOnlyIntegratedRootJointLinearVelocity", worldFrame, registry);

   private final YoFrameVector yoMeasurementFrameLinearVelocityInWorld;
   private final YoFrameVector yoRootJointIMUBasedLinearVelocityInWorld;
   private final YoFrameVector yoLinearAccelerationMeasurementInWorld;
   private final YoFrameVector yoLinearAccelerationMeasurement;

   private final YoBoolean imuBasedStateEstimationEnabled = new YoBoolean("imuBasedStateEstimationEnabled", registry);

   private final ReferenceFrame measurementFrame;

   private final FloatingInverseDynamicsJoint rootJoint;

   private final double estimatorDT;

   // Temporary variables
   private final FrameVector3D linearAccelerationMeasurement = new FrameVector3D();
   private final FrameVector3D tempRootJointVelocityIntegrated = new FrameVector3D();

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

   private final FrameVector3D tempVector = new FrameVector3D();

   private void updateLinearAccelerationMeasurement()
   {
      if (!isEstimationEnabled())
         return;

      imuBiasProvider.getLinearAccelerationBiasInIMUFrame(imuProcessedOutput, accelerationBias);

      linearAccelerationMeasurement.setToZero(measurementFrame);
      imuProcessedOutput.getLinearAccelerationMeasurement(linearAccelerationMeasurement);
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
      imuProcessedOutput.getLinearAccelerationMeasurement(linearAccelerationMeasurement);
      linearAccelerationMeasurement.sub(accelerationBias);

      if (cancelGravityFromAccelerationMeasurement.getBooleanValue())
      {
         tempVector.setIncludingFrame(gravityVector);
         tempVector.changeFrame(linearAccelerationMeasurement.getReferenceFrame());
         linearAccelerationMeasurement.add(tempVector);
      }
      yoLinearAccelerationMeasurement.set(linearAccelerationMeasurement);
   }

   private final FrameVector3D correctionVelocityForMeasurementFrameOffset = new FrameVector3D();

   public void updateIMUAndRootJointLinearVelocity(FrameVector3D rootJointVelocityToPack)
   {
      updateLinearAccelerationMeasurement();

      linearAccelerationMeasurement.setIncludingFrame(yoLinearAccelerationMeasurementInWorld);
      linearAccelerationMeasurement.scale(estimatorDT);

      yoMeasurementFrameLinearVelocityInWorld.add(linearAccelerationMeasurement);
      rootJointVelocityToPack.setIncludingFrame(yoMeasurementFrameLinearVelocityInWorld);

      getCorrectionVelocityForMeasurementFrameOffset(correctionVelocityForMeasurementFrameOffset);
      correctionVelocityForMeasurementFrameOffset.changeFrame(worldFrame);
      rootJointVelocityToPack.sub(correctionVelocityForMeasurementFrameOffset);
      yoRootJointIMUBasedLinearVelocityInWorld.set(rootJointVelocityToPack);

      imuLinearVelocityIMUOnly.add(linearAccelerationMeasurement);
      imuLinearVelocityIMUOnly.scale(alphaLeakIMUOnly.getDoubleValue());
      rootJointLinearVelocityIMUOnly.set(imuLinearVelocityIMUOnly);
      rootJointLinearVelocityIMUOnly.sub(correctionVelocityForMeasurementFrameOffset);
   }

   public void correctIMULinearVelocity(FrameVector3D rootJointVelocity)
   {
      rootJointLinearVelocity.set(rootJointVelocity);
      yoMeasurementFrameLinearVelocityInWorld.set(rootJointVelocity);
      yoMeasurementFrameLinearVelocityInWorld.add(correctionVelocityForMeasurementFrameOffset);
   }

   public void updatePelvisPosition(FramePoint3D rootJointPositionPrevValue, FramePoint3D rootJointPositionToPack)
   {
      if (!isEstimationEnabled())
         throw new RuntimeException("IMU estimation module for pelvis linear velocity is disabled.");

      tempRootJointVelocityIntegrated.setIncludingFrame(rootJointLinearVelocity);
      tempRootJointVelocityIntegrated.scale(estimatorDT);

      if(setRootJointPositionImuOnlyToCurrent.getBooleanValue())
      {
         rootJointPositionImuOnly.set(rootJointPositionPrevValue);
         setRootJointPositionImuOnlyToCurrent.set(false);
      }
      rootJointPositionImuOnly.add(tempRootJointVelocityIntegrated);

      rootJointPosition.set(rootJointPositionPrevValue);
      rootJointPosition.add(tempRootJointVelocityIntegrated);
      rootJointPositionToPack.setIncludingFrame(rootJointPosition);
   }

   private final Twist tempRootJointTwist = new Twist();
   private final FrameVector3D tempRootJointAngularVelocity = new FrameVector3D();
   private final FramePoint3D measurementOffset = new FramePoint3D();

   private void getCorrectionVelocityForMeasurementFrameOffset(FrameVector3D correctionTermToPack)
   {
      rootJoint.getJointTwist(tempRootJointTwist);
      tempRootJointTwist.getAngularPart(tempRootJointAngularVelocity);

      measurementOffset.setToZero(measurementFrame);
      measurementOffset.changeFrame(rootJoint.getFrameAfterJoint());

      correctionTermToPack.setToZero(tempRootJointAngularVelocity.getReferenceFrame());
      correctionTermToPack.cross(tempRootJointAngularVelocity, measurementOffset);
   }
}
