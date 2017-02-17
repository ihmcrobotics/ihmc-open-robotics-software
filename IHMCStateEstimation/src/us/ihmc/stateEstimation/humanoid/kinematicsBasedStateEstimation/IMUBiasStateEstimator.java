package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector;
import static us.ihmc.robotics.math.filters.AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameQuaternion;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.tools.FormattingTools;

public class IMUBiasStateEstimator implements IMUBiasProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<YoFrameQuaternion> rawOrientationBiases = new ArrayList<>();
   private final List<AlphaFilteredYoFrameQuaternion> orientationBiases = new ArrayList<>();
   private final List<DoubleYoVariable> orientationBiasMagnitudes = new ArrayList<>();
   private final List<AlphaFilteredYoFrameVector> angularVelocityBiases = new ArrayList<>();
   private final List<AlphaFilteredYoFrameVector> linearAccelerationBiases = new ArrayList<>();
   private final List<YoFrameVector> angularVelocityBiasesInWorld = new ArrayList<>();
   private final List<YoFrameVector> linearAccelerationBiasesInWorld = new ArrayList<>();

   private final List<YoFrameVector> angularVelocitiesInWorld = new ArrayList<>();
   private final List<YoFrameVector> linearAccelerationsInWorld = new ArrayList<>();
   private final List<DoubleYoVariable> linearAccelerationMagnitudes = new ArrayList<>();

   private final BooleanYoVariable enableIMUBiasCompensation = new BooleanYoVariable("enableIMUBiasCompensation", registry);
   private final DoubleYoVariable imuBiasEstimationThreshold = new DoubleYoVariable("imuBiasEstimationThreshold", registry);
   private final DoubleYoVariable biasAlphaFilter = new DoubleYoVariable("imuBiasAlphaFilter", registry);

   private final List<DoubleYoVariable> feetToIMUAngularVelocityMagnitudes = new ArrayList<>();
   private final List<DoubleYoVariable> feetToIMULinearVelocityMagnitudes = new ArrayList<>();
   private final List<BooleanYoVariable> isBiasEstimated = new ArrayList<>();
   private final List<BooleanYoVariable> isIMUOrientationBiasEstimated = new ArrayList<>();

   private final List<? extends IMUSensorReadOnly> imuProcessedOutputs;
   private final Map<IMUSensorReadOnly, Integer> imuToIndexMap = new HashMap<>();
   private final List<RigidBody> feet;

   private final TwistCalculator twistCalculator;

   private final Vector3D gravityVectorInWorld = new Vector3D();
   private final Vector3D zUpVector = new Vector3D();

   private final boolean isAccelerationIncludingGravity;
   private final double updateDT;

   public IMUBiasStateEstimator(List<? extends IMUSensorReadOnly> imuProcessedOutputs, Collection<RigidBody> feet, TwistCalculator twistCalculator,
         double gravitationalAcceleration, boolean isAccelerationIncludingGravity, double updateDT, YoVariableRegistry parentRegistry)
   {
      this.imuProcessedOutputs = imuProcessedOutputs;
      this.updateDT = updateDT;
      this.feet = new ArrayList<>(feet);
      this.twistCalculator = twistCalculator;
      this.isAccelerationIncludingGravity = isAccelerationIncludingGravity;

      imuBiasEstimationThreshold.set(0.015);
      biasAlphaFilter.set(0.99995);

      gravityVectorInWorld.set(0.0, 0.0, -Math.abs(gravitationalAcceleration));
      zUpVector.set(0.0, 0.0, 1.0);

      for (int i = 0; i < imuProcessedOutputs.size(); i++)
      {
         IMUSensorReadOnly imuSensor = imuProcessedOutputs.get(i);
         ReferenceFrame measurementFrame = imuSensor.getMeasurementFrame();
         String sensorName = imuSensor.getSensorName();
         sensorName = sensorName.replaceFirst(imuSensor.getMeasurementLink().getName(), "");
         sensorName = FormattingTools.underscoredToCamelCase(sensorName, true);

         imuToIndexMap.put(imuSensor, i);

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

         angularVelocitiesInWorld.add(new YoFrameVector("unprocessed" + sensorName + "AngularVelocityInWorld", worldFrame, registry));
         
         linearAccelerationsInWorld.add(new YoFrameVector("unprocessed" + sensorName + "LinearAccelerationWorld", worldFrame, registry));
         linearAccelerationMagnitudes.add(new DoubleYoVariable("unprocessed" + sensorName + "LinearAccelerationMagnitude", registry));

         orientationBiasMagnitudes.add(new DoubleYoVariable("estimated" + sensorName + "OrientationBiasMagnitude", registry));

         feetToIMUAngularVelocityMagnitudes.add(new DoubleYoVariable("feetTo" + sensorName + "AngularVelocityMagnitude", registry));
         feetToIMULinearVelocityMagnitudes.add(new DoubleYoVariable("feetTo" + sensorName + "LinearVelocityMagnitude", registry));
         isBiasEstimated.add(new BooleanYoVariable("is" + sensorName + "BiasEstimated", registry));
         isIMUOrientationBiasEstimated.add(new BooleanYoVariable("is" + sensorName + "OrientationBiasEstimated", registry));

         angularVelocityBiasesInWorld.add(new YoFrameVector("estimated" + sensorName + "AngularVelocityBiasWorld", worldFrame, registry));
         linearAccelerationBiasesInWorld.add(new YoFrameVector("estimated" + sensorName + "LinearAccelerationBiasWorld", worldFrame, registry));
      }

      parentRegistry.addChild(registry);
   }

   public void configureModuleParameters(StateEstimatorParameters stateEstimatorParameters)
   {
      enableIMUBiasCompensation.set(stateEstimatorParameters.enableIMUBiasCompensation());
      double biasFilterBreakFrequency = stateEstimatorParameters.getIMUBiasFilterFreqInHertz();
      biasAlphaFilter.set(computeAlphaGivenBreakFrequencyProperly(biasFilterBreakFrequency, updateDT));

      imuBiasEstimationThreshold.set(stateEstimatorParameters.getIMUBiasVelocityThreshold());
   }

   public void initialize()
   {
      for (int imuIndex = 0; imuIndex < imuProcessedOutputs.size(); imuIndex++)
      {
         rawOrientationBiases.get(imuIndex).setToZero();
         orientationBiases.get(imuIndex).setToZero();
         orientationBiasMagnitudes.get(imuIndex).set(0.0);
         angularVelocityBiases.get(imuIndex).setToZero();
         linearAccelerationBiases.get(imuIndex).setToZero();
         angularVelocityBiasesInWorld.get(imuIndex).setToZero();
         linearAccelerationBiasesInWorld.get(imuIndex).setToZero();
      }
   }

   private final Twist twist = new Twist();

   private final Vector3D measurement = new Vector3D();
   private final Vector3D measurementInWorld = new Vector3D();
   private final Vector3D measurementNormalizedInWorld = new Vector3D();
   private final Vector3D measurementMinusGravity = new Vector3D();
   private final Vector3D measurementMinusGravityInWorld = new Vector3D();
   private final Vector3D measurementBias = new Vector3D();

   private final Vector3D biasRotationAxis = new Vector3D();
   private final AxisAngle biasAxisAngle = new AxisAngle();
   private final RotationMatrix orientationMeasurement = new RotationMatrix();
   private final RotationMatrix orientationMeasurementTransposed = new RotationMatrix();

   public void compute(List<RigidBody> trustedFeet)
   {
      checkIfBiasEstimationPossible(trustedFeet);
      estimateBiases();
   }

   private void checkIfBiasEstimationPossible(List<RigidBody> trustedFeet)
   {
      if (trustedFeet.size() < feet.size())
      {
         for (int imuIndex = 0; imuIndex < imuProcessedOutputs.size(); imuIndex++)
         {
            isIMUOrientationBiasEstimated.get(imuIndex).set(false);
            isBiasEstimated.get(imuIndex).set(false);
         }
         return;
      }

      for (int imuIndex = 0; imuIndex < imuProcessedOutputs.size(); imuIndex++)
      {
         IMUSensorReadOnly imuSensor = imuProcessedOutputs.get(imuIndex);
         RigidBody measurementLink = imuSensor.getMeasurementLink();

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
            isIMUOrientationBiasEstimated.get(imuIndex).set(isAccelerationIncludingGravity);
         }
         else
         {
            isBiasEstimated.get(imuIndex).set(false);
            isIMUOrientationBiasEstimated.get(imuIndex).set(false);
         }
      }
   }

   private void estimateBiases()
   {
      for (int imuIndex = 0; imuIndex < imuProcessedOutputs.size(); imuIndex++)
      {
         IMUSensorReadOnly imuSensor = imuProcessedOutputs.get(imuIndex);

         if (isBiasEstimated.get(imuIndex).getBooleanValue())
         {
            imuSensor.getOrientationMeasurement(orientationMeasurement);
            orientationMeasurementTransposed.setAndTranspose(orientationMeasurement);

            imuSensor.getAngularVelocityMeasurement(measurement);
            angularVelocityBiases.get(imuIndex).update(measurement);
            angularVelocityBiases.get(imuIndex).get(measurementBias);
            orientationMeasurement.transform(measurementBias);
            angularVelocityBiasesInWorld.get(imuIndex).set(measurementBias);

            orientationMeasurement.transform(measurement, measurementInWorld);
            angularVelocitiesInWorld.get(imuIndex).set(measurementInWorld);

            imuSensor.getLinearAccelerationMeasurement(measurement);
            orientationMeasurement.transform(measurement, measurementInWorld);
            linearAccelerationsInWorld.get(imuIndex).set(measurementInWorld);

            if (isAccelerationIncludingGravity)
            {
               measurementNormalizedInWorld.setAndNormalize(measurementInWorld);

               biasRotationAxis.cross(zUpVector, measurementNormalizedInWorld);
               double biasMagnitude = zUpVector.angle(measurementNormalizedInWorld);

               if (Math.abs(biasMagnitude) < 1.0e-7)
               {
                  rawOrientationBiases.get(imuIndex).setToZero();
               }
               else
               {
                  biasRotationAxis.scale(biasMagnitude);
                  biasMagnitude = biasRotationAxis.length();
                  biasRotationAxis.scale(1.0 / biasMagnitude);
                  orientationMeasurementTransposed.transform(biasRotationAxis);
                  biasAxisAngle.set(biasRotationAxis, biasMagnitude);
                  rawOrientationBiases.get(imuIndex).set(biasAxisAngle);
               }

               AlphaFilteredYoFrameQuaternion yoOrientationBias = orientationBiases.get(imuIndex);
               yoOrientationBias.update();
               yoOrientationBias.get(biasAxisAngle);
               orientationBiasMagnitudes.get(imuIndex).set(Math.abs(biasAxisAngle.getAngle()));

               measurementMinusGravityInWorld.add(measurementInWorld, gravityVectorInWorld);
               orientationMeasurementTransposed.transform(measurementMinusGravityInWorld, measurementMinusGravity);
               linearAccelerationBiases.get(imuIndex).update(measurementMinusGravity);
            }
            else
            {
               linearAccelerationBiases.get(imuIndex).update(measurement);
            }

            linearAccelerationBiases.get(imuIndex).get(measurementBias);
            orientationMeasurement.transform(measurementBias);
            linearAccelerationBiasesInWorld.get(imuIndex).set(measurementBias);
         }
      }
   }

   @Override
   public void getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu, Vector3D angularVelocityBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getBooleanValue() || imuIndex == null)
         angularVelocityBiasToPack.set(0.0, 0.0, 0.0);
      else
         angularVelocityBiases.get(imuIndex.intValue()).get(angularVelocityBiasToPack);
   }

   @Override
   public void getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu, FrameVector angularVelocityBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getBooleanValue() || imuIndex == null)
         angularVelocityBiasToPack.setToZero(imu.getMeasurementFrame());
      else
         angularVelocityBiases.get(imuIndex.intValue()).getFrameTupleIncludingFrame(angularVelocityBiasToPack);
   }

   @Override
   public void getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu, Vector3D angularVelocityBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getBooleanValue() || imuIndex == null)
         angularVelocityBiasToPack.set(0.0, 0.0, 0.0);
      else
         angularVelocityBiasesInWorld.get(imuIndex.intValue()).get(angularVelocityBiasToPack);
   }

   @Override
   public void getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu, FrameVector angularVelocityBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getBooleanValue() || imuIndex == null)
         angularVelocityBiasToPack.setToZero(worldFrame);
      else
         angularVelocityBiasesInWorld.get(imuIndex.intValue()).getFrameTupleIncludingFrame(angularVelocityBiasToPack);
   }

   @Override
   public void getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu, Vector3D linearAccelerationBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getBooleanValue() || imuIndex == null)
         linearAccelerationBiasToPack.set(0.0, 0.0, 0.0);
      else
         linearAccelerationBiases.get(imuIndex.intValue()).get(linearAccelerationBiasToPack);
   }

   @Override
   public void getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu, FrameVector linearAccelerationBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getBooleanValue() || imuIndex == null)
         linearAccelerationBiasToPack.setToZero(imu.getMeasurementFrame());
      else
         linearAccelerationBiases.get(imuIndex.intValue()).getFrameTupleIncludingFrame(linearAccelerationBiasToPack);
   }

   @Override
   public void getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu, Vector3D linearAccelerationBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getBooleanValue() || imuIndex == null)
         linearAccelerationBiasToPack.set(0.0, 0.0, 0.0);
      else
         linearAccelerationBiasesInWorld.get(imuIndex.intValue()).get(linearAccelerationBiasToPack);
   }

   @Override
   public void getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu, FrameVector linearAccelerationBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getBooleanValue() || imuIndex == null)
         linearAccelerationBiasToPack.setToZero(worldFrame);
      else
         linearAccelerationBiasesInWorld.get(imuIndex.intValue()).getFrameTupleIncludingFrame(linearAccelerationBiasToPack);
   }
}
