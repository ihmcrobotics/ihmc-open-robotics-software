package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.List;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;


public class PelvisIMUBasedLinearStateCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final BooleanYoVariable enableEstimationOfGravity = new BooleanYoVariable("enableEstimationOfGravity", registry);
   private final DoubleYoVariable alphaGravityEstimation = new DoubleYoVariable("alphaGravityEstimation", registry);
   private final AlphaFilteredYoVariable gravityEstimation = new AlphaFilteredYoVariable("gravityEstimation", registry, alphaGravityEstimation);
   private final FrameVector gravity = new FrameVector(worldFrame);
   
   private final YoFrameVector rootJointLinearVelocity = new YoFrameVector("imuRootJointLinearVelocity", worldFrame, registry);
   private final YoFrameVector rootJointPosition = new YoFrameVector("imuRootJointPosition", worldFrame, registry);

   private final YoFrameVector yoMeasurementFrameLinearVelocityInWorld;
   private final YoFrameVector yoLinearAccelerationMeasurementInWorld;
   private final YoFrameVector yoLinearAccelerationMeasurement;
   
   private final BooleanYoVariable imuBasedStateEstimationEnabled = new BooleanYoVariable("imuBasedStateEstimationEnabled", registry);

   private final ReferenceFrame measurementFrame;
   
   private final SixDoFJoint rootJoint;
   
   private final double estimatorDT;
   
   // Temporary variables
   private final FrameVector linearAccelerationMeasurement = new FrameVector();
   private final FrameVector tempRootJointVelocityIntegrated = new FrameVector();

   private final IMUSensorReadOnly imuProcessedOutput;

   public PelvisIMUBasedLinearStateCalculator(FullInverseDynamicsStructure inverseDynamicsStructure, List<? extends IMUSensorReadOnly> imuProcessedOutputs, double estimatorDT,
         double gravitationalAcceleration, YoVariableRegistry parentRegistry)
   {
      this.estimatorDT = estimatorDT;
      this.rootJoint = inverseDynamicsStructure.getRootJoint();

      enableEstimationOfGravity.set(false);
      gravityEstimation.reset();
      gravityEstimation.update(Math.abs(gravitationalAcceleration));
      gravity.setIncludingFrame(worldFrame, 0.0, 0.0, gravityEstimation.getDoubleValue());
      
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
      yoLinearAccelerationMeasurement = new YoFrameVector("imuLinearAcceleration", measurementFrame, registry);
      yoLinearAccelerationMeasurementInWorld = new YoFrameVector("imuLinearAccelerationInWorld", worldFrame, registry);
      
      parentRegistry.addChild(registry);
   }

   public void enableGravityEstimation(boolean enable)
   {
      enableEstimationOfGravity.set(enable);
   }

   public void enableEsimationModule(boolean enable)
   {
      imuBasedStateEstimationEnabled.set(enable);
   }

   public boolean isEstimationEnabled()
   {
      return imuBasedStateEstimationEnabled.getBooleanValue();
   }
   
   public void setAlphaGravityEstimation(double alphaFilter)
   {
      alphaGravityEstimation.set(alphaFilter);
   }
   
   private void updateLinearAccelerationMeasurement()
   {
      if (!isEstimationEnabled())
         return;

      linearAccelerationMeasurement.setToZero(measurementFrame);
      imuProcessedOutput.getLinearAccelerationMeasurement(linearAccelerationMeasurement.getVector());
      if (enableEstimationOfGravity.getBooleanValue())
         gravityEstimation.update(linearAccelerationMeasurement.length());
      gravity.setIncludingFrame(worldFrame, 0.0, 0.0, gravityEstimation.getDoubleValue());

      // Update acceleration in world (minus gravity)
      linearAccelerationMeasurement.changeFrame(worldFrame);
      linearAccelerationMeasurement.sub(gravity);
      yoLinearAccelerationMeasurementInWorld.set(linearAccelerationMeasurement);

      // Update acceleration in local frame (minus gravity)
      gravity.changeFrame(measurementFrame);
      linearAccelerationMeasurement.setToZero(measurementFrame);
      imuProcessedOutput.getLinearAccelerationMeasurement(linearAccelerationMeasurement.getVector());
      linearAccelerationMeasurement.sub(gravity);
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

      rootJointPosition.set(rootJointPositionPrevValue);
      rootJointPosition.add(tempRootJointVelocityIntegrated);
      rootJointPosition.getFrameTupleIncludingFrame(rootJointPositionToPack);
   }

   private final Twist tempRootJointTwist = new Twist();
   private final FrameVector tempRootJointAngularVelocity = new FrameVector();
   private final FramePoint measurementOffset = new FramePoint();

   private void getCorrectionVelocityForMeasurementFrameOffset(FrameVector correctionTermToPack)
   {
      rootJoint.packJointTwist(tempRootJointTwist);
      tempRootJointTwist.packAngularPart(tempRootJointAngularVelocity);
      
      measurementOffset.setToZero(measurementFrame);
      measurementOffset.changeFrame(rootJoint.getFrameAfterJoint());
      
      correctionTermToPack.setToZero(tempRootJointAngularVelocity.getReferenceFrame());
      correctionTermToPack.cross(tempRootJointAngularVelocity, measurementOffset);
   }
}
