package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameQuaternion;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
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

   private final List<YoFrameQuaternion> rawOrientationBiases = new ArrayList<>();
   private final List<AlphaFilteredYoFrameQuaternion> orientationBiases = new ArrayList<>();
   private final List<DoubleYoVariable> orientationBiasMagnitudes = new ArrayList<>();
   private final List<AlphaFilteredYoFrameVector> angularVelocityBiases = new ArrayList<>();
   private final List<AlphaFilteredYoFrameVector> linearAccelerationBiases = new ArrayList<>();

   private final DoubleYoVariable biasAlphaFilter = new DoubleYoVariable("imuBiasAlphaFilter", registry);

   private final List<DoubleYoVariable> feetToIMUAngularVelocityMagnitudes = new ArrayList<>();
   private final List<DoubleYoVariable> feetToIMULinearVelocityMagnitudes = new ArrayList<>();
   private final List<BooleanYoVariable> isBiasEstimated = new ArrayList<>();

   private final DoubleYoVariable imuBiasEstimationThreshold = new DoubleYoVariable("imuBiasEstimationThreshold", registry);
   private final BooleanYoVariable isIMUOrientationBiasEstimated = new BooleanYoVariable("isIMUOrientationBiasEstimated", registry);

   private final List<? extends IMUSensorReadOnly> imuProcessedOutputs;
   private final List<RigidBody> feet;

   private final TwistCalculator twistCalculator;

   private final FrameVector gravityVector = new FrameVector();
   private final Vector3d zUpVector = new Vector3d();

   private final boolean isAccelerationIncludingGravity;

   public IMUBiasStateEstimator(List<? extends IMUSensorReadOnly> imuProcessedOutputs, List<RigidBody> feet, TwistCalculator twistCalculator,
         double gravitationalAcceleration, boolean isAccelerationIncludingGravity, YoVariableRegistry parentRegistry)
   {
      this.imuProcessedOutputs = imuProcessedOutputs;
      this.feet = feet;
      this.twistCalculator = twistCalculator;
      this.isAccelerationIncludingGravity = isAccelerationIncludingGravity;

      biasAlphaFilter.set(0.99995);
      imuBiasEstimationThreshold.set(0.015);

      gravityVector.setIncludingFrame(worldFrame, 0.0, 0.0, -Math.abs(gravitationalAcceleration));
      zUpVector.set(0.0, 0.0, 1.0);

      for (int i = 0; i < imuProcessedOutputs.size(); i++)
      {
         IMUSensorReadOnly imuSensor = imuProcessedOutputs.get(i);
         ReferenceFrame measurementFrame = imuSensor.getMeasurementFrame();
         String sensorName = imuSensor.getSensorName();
         sensorName = sensorName.replaceFirst(imuSensor.getMeasurementLink().getName(), "");
         sensorName = FormattingTools.underscoredToCamelCase(sensorName, true);

         AlphaFilteredYoFrameVector angularVelocityBias = createAlphaFilteredYoFrameVector("estimated" + sensorName + "AngularVelocityBias", "", registry, biasAlphaFilter, measurementFrame);
         angularVelocityBias.update(0.0, 0.0, 0.0);
         angularVelocityBiases.add(angularVelocityBias);
         
         AlphaFilteredYoFrameVector linearAccelerationBias = createAlphaFilteredYoFrameVector("estimated" + sensorName + "LinearAccelerationBias", "", registry, biasAlphaFilter, measurementFrame);
         linearAccelerationBias.update(0.0, 0.0, 0.0);
         linearAccelerationBiases.add(linearAccelerationBias);

         YoFrameQuaternion rawOrientationBias = new YoFrameQuaternion("estimated" + sensorName + "RawQuaternionBias", measurementFrame, registry);
         rawOrientationBiases.add(rawOrientationBias);

         AlphaFilteredYoFrameQuaternion orientationBias = new AlphaFilteredYoFrameQuaternion("estimated" + sensorName + "QuaternionBias", "", rawOrientationBias, biasAlphaFilter, registry);
         orientationBias.update();
         orientationBiases.add(orientationBias);

         orientationBiasMagnitudes.add(new DoubleYoVariable("estimated" + sensorName + "OrientationBiasMagnitude", registry));

         feetToIMUAngularVelocityMagnitudes.add(new DoubleYoVariable("feetTo" + sensorName + "AngularVelocityMagnitude", registry));
         feetToIMULinearVelocityMagnitudes.add(new DoubleYoVariable("feetTo" + sensorName + "LinearVelocityMagnitude", registry));
         isBiasEstimated.add(new BooleanYoVariable("is" + sensorName + "BiasEstimated", registry));
      }

      parentRegistry.addChild(registry);
   }

   private final Twist twist = new Twist();

   private final Vector3d measurement = new Vector3d();
   private final FrameVector linearAcceleration = new FrameVector();
   private final Vector3d biasRotationAxis = new Vector3d();
   private final AxisAngle4d biasAxisAngle = new AxisAngle4d();
   private final Matrix3d orientationMeasurement = new Matrix3d();

   public void estimateBiases(List<RigidBody> trustedFeet)
   {
      if (trustedFeet.size() < feet.size())
      {
         isIMUOrientationBiasEstimated.set(false);
         for (int i = 0; i < isBiasEstimated.size(); i++)
            isBiasEstimated.get(i).set(false);
         return;
      }

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
            isBiasEstimated.get(imuIndex).set(true);

            imuSensor.getAngularVelocityMeasurement(measurement);
            angularVelocityBiases.get(imuIndex).update(measurement);

            imuSensor.getLinearAccelerationMeasurement(measurement);

            if (isAccelerationIncludingGravity)
            {
               linearAcceleration.setIncludingFrame(measurementFrame, measurement);
               linearAcceleration.changeFrame(worldFrame);
               linearAcceleration.add(gravityVector);
               linearAcceleration.changeFrame(measurementFrame);
               linearAcceleration.get(measurement);
            }

            linearAccelerationBiases.get(imuIndex).update(measurement);

            if (isAccelerationIncludingGravity)
            {
               isIMUOrientationBiasEstimated.set(true);
               
               imuSensor.getLinearAccelerationMeasurement(measurement);
               imuSensor.getOrientationMeasurement(orientationMeasurement);
               orientationMeasurement.transform(measurement);
               measurement.normalize();

               biasRotationAxis.cross(zUpVector, measurement);
               double biasMagnitude = zUpVector.angle(measurement);

               if (Math.abs(biasMagnitude) < 1.0e-7)
               {
                  rawOrientationBiases.get(imuIndex).setToZero();
               }
               else
               {
                  biasRotationAxis.scale(biasMagnitude);
                  biasMagnitude = biasRotationAxis.length();
                  biasRotationAxis.scale(1.0 / biasMagnitude);
                  orientationMeasurement.transpose();
                  orientationMeasurement.transform(biasRotationAxis);
                  biasAxisAngle.set(biasRotationAxis, biasMagnitude);
                  rawOrientationBiases.get(imuIndex).set(biasAxisAngle);
               }

               AlphaFilteredYoFrameQuaternion yoOrientationBias = orientationBiases.get(imuIndex);
               yoOrientationBias.update();
               yoOrientationBias.get(biasAxisAngle);
               orientationBiasMagnitudes.get(imuIndex).set(Math.abs(biasAxisAngle.getAngle()));
            }
            else
            {
               isIMUOrientationBiasEstimated.set(false);
            }
         }
         else
         {
            isBiasEstimated.get(imuIndex).set(false);
            isIMUOrientationBiasEstimated.set(false);
         }
      }
   }
}
