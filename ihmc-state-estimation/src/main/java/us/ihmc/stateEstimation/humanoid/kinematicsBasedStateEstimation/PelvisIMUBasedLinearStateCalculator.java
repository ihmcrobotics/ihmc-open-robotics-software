package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class PelvisIMUBasedLinearStateCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanProvider cancelGravityFromAccelerationMeasurement;// = new YoBoolean("cancelGravityFromAccelerationMeasurement", registry);

   private final FrameVector3D accelerationBias = new FrameVector3D(worldFrame);
   private final FrameVector3D gravityVector = new FrameVector3D();

   private final YoFrameVector3D rootJointLinearVelocity = new YoFrameVector3D("imuRootJointLinearVelocity", worldFrame, registry);
   private final YoFrameVector3D rootJointPosition = new YoFrameVector3D("imuRootJointPosition", worldFrame, registry);
   private final YoBoolean setRootJointPositionImuOnlyToCurrent = new YoBoolean("setRootJointPositionImuOnlyToCurrent", registry);
   private final YoDouble alphaLeakIMUOnly = new YoDouble("imuOnlyAlphaLeak", registry);
   private final YoFrameVector3D rootJointPositionImuOnly = new YoFrameVector3D("imuOnlyIntregratedRootJointPosition", worldFrame, registry);
   private final YoFrameVector3D imuLinearVelocityIMUOnly = new YoFrameVector3D("imuOnlyIntegratedIMULinearVelocity", worldFrame, registry);
   private final YoFrameVector3D rootJointLinearVelocityIMUOnly = new YoFrameVector3D("imuOnlyIntegratedRootJointLinearVelocity", worldFrame, registry);

   private final YoFrameVector3D yoMeasurementFrameLinearVelocityInWorld;
   private final YoFrameVector3D yoRootJointIMUBasedLinearVelocityInWorld;
   private final YoFrameVector3D yoLinearAccelerationMeasurementInWorld;
   private final YoFrameVector3D yoLinearAccelerationMeasurement;

   private final BooleanProvider useAccelerometerForEstimation;

   private final ReferenceFrame measurementFrame;

   private final FloatingJointBasics rootJoint;

   private final double estimatorDT;

   // Temporary variables
   private final FrameVector3D linearAccelerationMeasurement = new FrameVector3D();
   private final FrameVector3D tempRootJointVelocityIntegrated = new FrameVector3D();

   private final IMUSensorReadOnly imuProcessedOutput;
   private final IMUBiasProvider imuBiasProvider;

   public PelvisIMUBasedLinearStateCalculator(FullInverseDynamicsStructure inverseDynamicsStructure, List<? extends IMUSensorReadOnly> imuProcessedOutputs,
                                              IMUBiasProvider imuBiasProvider, BooleanProvider cancelGravityFromAccelerationMeasurement, double estimatorDT,
                                              double gravitationalAcceleration, StateEstimatorParameters stateEstimatorParameters,
                                              YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.imuBiasProvider = imuBiasProvider;
      this.estimatorDT = estimatorDT;
      this.rootJoint = inverseDynamicsStructure.getRootJoint();
      this.cancelGravityFromAccelerationMeasurement = cancelGravityFromAccelerationMeasurement;

      if (stateEstimatorParameters == null)
      {
         useAccelerometerForEstimation = new BooleanParameter("useAccelerometerForEstimation", registry);
      }
      else
      {
         boolean initialValue = stateEstimatorParameters.useAccelerometerForEstimation();
         useAccelerometerForEstimation = new BooleanParameter("useAccelerometerForEstimation", registry, initialValue);
      }

      gravityVector.setIncludingFrame(worldFrame, 0.0, 0.0, -Math.abs(gravitationalAcceleration));

      if (imuProcessedOutputs.size() == 0)
      {
         imuProcessedOutput = null;
         measurementFrame = null;
      }
      else
      {
         if (imuProcessedOutputs.size() > 1)
            System.out.println(getClass().getSimpleName() + ": More than 1 IMU sensor, using only the first one: " + imuProcessedOutputs.get(0).getSensorName());
         imuProcessedOutput = imuProcessedOutputs.get(0);
         measurementFrame = imuProcessedOutput.getMeasurementFrame();
      }

      yoMeasurementFrameLinearVelocityInWorld = new YoFrameVector3D("imuLinearVelocityInWorld", worldFrame, registry);
      yoRootJointIMUBasedLinearVelocityInWorld = new YoFrameVector3D("rootJointIMUBasedLinearVelocityInWorld", worldFrame, registry);
      yoLinearAccelerationMeasurement = new YoFrameVector3D("imuLinearAcceleration", measurementFrame, registry);
      yoLinearAccelerationMeasurementInWorld = new YoFrameVector3D("imuLinearAccelerationInWorld", worldFrame, registry);

      setRootJointPositionImuOnlyToCurrent.set(true);
      alphaLeakIMUOnly.set(0.999);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      yoMeasurementFrameLinearVelocityInWorld.setToZero();
      imuLinearVelocityIMUOnly.setToZero();
      setRootJointPositionImuOnlyToCurrent.set(true);
   }

   public boolean isEstimationEnabled()
   {
      return useAccelerometerForEstimation.getValue() && imuProcessedOutput != null;
   }

   private final FrameVector3D tempVector = new FrameVector3D();

   private void updateLinearAccelerationMeasurement()
   {
      if (!isEstimationEnabled())
         return;

      imuBiasProvider.getLinearAccelerationBiasInIMUFrame(imuProcessedOutput, accelerationBias);

      linearAccelerationMeasurement.setToZero(measurementFrame);
      linearAccelerationMeasurement.set(imuProcessedOutput.getLinearAccelerationMeasurement());
      linearAccelerationMeasurement.sub(accelerationBias);

      // Update acceleration in world (minus gravity)
      linearAccelerationMeasurement.changeFrame(worldFrame);
      if (cancelGravityFromAccelerationMeasurement.getValue())
      {
         tempVector.setIncludingFrame(gravityVector);
         tempVector.changeFrame(linearAccelerationMeasurement.getReferenceFrame());
         linearAccelerationMeasurement.add(tempVector);
      }
      yoLinearAccelerationMeasurementInWorld.set(linearAccelerationMeasurement);

      // Update acceleration in local frame (minus gravity)
      linearAccelerationMeasurement.setToZero(measurementFrame);
      linearAccelerationMeasurement.set(imuProcessedOutput.getLinearAccelerationMeasurement());
      linearAccelerationMeasurement.sub(accelerationBias);

      if (cancelGravityFromAccelerationMeasurement.getValue())
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
      tempRootJointTwist.setIncludingFrame(rootJoint.getJointTwist());
      tempRootJointAngularVelocity.setIncludingFrame(tempRootJointTwist.getAngularPart());

      measurementOffset.setToZero(measurementFrame);
      measurementOffset.changeFrame(rootJoint.getFrameAfterJoint());

      correctionTermToPack.setToZero(tempRootJointAngularVelocity.getReferenceFrame());
      correctionTermToPack.cross(tempRootJointAngularVelocity, measurementOffset);
   }
}
