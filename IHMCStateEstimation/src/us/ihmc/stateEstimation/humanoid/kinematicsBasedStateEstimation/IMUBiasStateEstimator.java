package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.tools.FormattingTools;

public class IMUBiasStateEstimator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<AlphaFilteredYoFrameVector> angularVelocityBiases = new ArrayList<>();
   private final List<AlphaFilteredYoFrameVector> linearAccelerationBiases = new ArrayList<>();

   private final DoubleYoVariable biasAlphaFilter = new DoubleYoVariable("imuBiasAlphaFilter", registry);

   private final List<DoubleYoVariable> feetToIMUAngularVelocityMagnitudes = new ArrayList<>();
   private final List<DoubleYoVariable> feetToIMULinearVelocityMagnitudes = new ArrayList<>();

   private final DoubleYoVariable imuBiasEstimationThreshold = new DoubleYoVariable("imuBiasEstimationThreshold", registry);

   private final List<? extends IMUSensorReadOnly> imuProcessedOutputs;
   private final List<RigidBody> feet;

   private final TwistCalculator twistCalculator;

   private final FrameVector gravityVector = new FrameVector();

   private final boolean cancelGravityFromAccelerationMeasurement;

   public IMUBiasStateEstimator(List<? extends IMUSensorReadOnly> imuProcessedOutputs, List<RigidBody> feet, TwistCalculator twistCalculator,
         double gravitationalAcceleration, boolean cancelGravityFromAccelerationMeasurement, YoVariableRegistry parentRegistry)
   {
      this.imuProcessedOutputs = imuProcessedOutputs;
      this.feet = feet;
      this.twistCalculator = twistCalculator;
      this.cancelGravityFromAccelerationMeasurement = cancelGravityFromAccelerationMeasurement;

      biasAlphaFilter.set(0.99995);
      imuBiasEstimationThreshold.set(0.005);

      gravityVector.setIncludingFrame(worldFrame, 0.0, 0.0, -Math.abs(gravitationalAcceleration));

      for (int i = 0; i < imuProcessedOutputs.size(); i++)
      {
         IMUSensorReadOnly imuSensor = imuProcessedOutputs.get(i);
         ReferenceFrame measurementFrame = imuSensor.getMeasurementFrame();
         String sensorName = imuSensor.getSensorName();
         sensorName = sensorName.replaceFirst(imuSensor.getMeasurementLink().getName(), "");
         sensorName = FormattingTools.underscoredToCamelCase(sensorName, true);

         AlphaFilteredYoFrameVector angularVelocityBias = createAlphaFilteredYoFrameVector("estimated" + sensorName + "AngularVelocityBias", "", registry, biasAlphaFilter, measurementFrame);
         angularVelocityBiases.add(angularVelocityBias);
         
         AlphaFilteredYoFrameVector linearAccelerationBias = createAlphaFilteredYoFrameVector("estimated" + sensorName + "LinearAccelerationBias", "", registry, biasAlphaFilter, measurementFrame);
         linearAccelerationBiases.add(linearAccelerationBias);

         feetToIMUAngularVelocityMagnitudes.add(new DoubleYoVariable("feetTo" + sensorName + "AngularVelocityMagnitude", registry));
         feetToIMULinearVelocityMagnitudes.add(new DoubleYoVariable("feetTo" + sensorName + "LinearVelocityMagnitude", registry));
      }

      parentRegistry.addChild(registry);
   }

   private final Twist twist = new Twist();

   private final Vector3d measurement = new Vector3d();
   private final FrameVector linearAcceleration = new FrameVector();

   public void estimateBiases(List<RigidBody> trustedFeet)
   {
      if (trustedFeet.size() < feet.size())
         return;

      for (int imuIndex = 0; imuIndex < imuProcessedOutputs.size(); imuIndex++)
      {
         IMUSensorReadOnly imuSensor = imuProcessedOutputs.get(imuIndex);
         RigidBody measurementLink = imuSensor.getMeasurementLink();
         ReferenceFrame measurementFrame = imuSensor.getMeasurementFrame();

         double feetToIMUAngularVelocityMagnitude = 0.0;
         double feetToIMULinearVelocityMagnitude = 0.0;

         for (int footIndex = 0; footIndex < trustedFeet.size(); footIndex++)
         {
            RigidBody trustedFoot = trustedFeet.get(footIndex);

            twistCalculator.getRelativeTwist(twist, trustedFoot, measurementLink);
            feetToIMUAngularVelocityMagnitude += twist.getAngularPartMagnitude();
            feetToIMULinearVelocityMagnitude += twist.getLinearPartMagnitude();
         }

         feetToIMUAngularVelocityMagnitudes.get(imuIndex).set(feetToIMUAngularVelocityMagnitude);
         feetToIMULinearVelocityMagnitudes.get(imuIndex).set(feetToIMULinearVelocityMagnitude);

         if (feetToIMUAngularVelocityMagnitude < imuBiasEstimationThreshold.getDoubleValue()
               && feetToIMULinearVelocityMagnitude < imuBiasEstimationThreshold.getDoubleValue())
         {
            imuSensor.getAngularVelocityMeasurement(measurement);
            angularVelocityBiases.get(imuIndex).update(measurement);

            imuSensor.getLinearAccelerationMeasurement(measurement);

            if (cancelGravityFromAccelerationMeasurement)
            {
               linearAcceleration.setIncludingFrame(measurementFrame, measurement);
               linearAcceleration.changeFrame(worldFrame);
               linearAcceleration.add(gravityVector);
               linearAcceleration.changeFrame(measurementFrame);
               linearAcceleration.get(measurement);
            }

            linearAccelerationBiases.get(imuIndex).update(measurement);
         }
      }
   }
}
