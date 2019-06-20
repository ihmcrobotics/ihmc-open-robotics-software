package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.math.filters.AlphaBasedOnBreakFrequencyProvider;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameQuaternion;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class IMUBiasStateEstimator implements IMUBiasProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<YoFrameQuaternion> rawOrientationBiases = new ArrayList<>();
   private final List<AlphaFilteredYoFrameQuaternion> orientationBiases = new ArrayList<>();
   private final List<YoDouble> orientationBiasMagnitudes = new ArrayList<>();
   private final List<AlphaFilteredYoFrameVector> angularVelocityBiases = new ArrayList<>();
   private final List<AlphaFilteredYoFrameVector> linearAccelerationBiases = new ArrayList<>();
   private final List<YoFrameVector3D> angularVelocityBiasesInWorld = new ArrayList<>();
   private final List<YoFrameVector3D> linearAccelerationBiasesInWorld = new ArrayList<>();

   private final List<YoFrameVector3D> angularVelocitiesInWorld = new ArrayList<>();
   private final List<YoFrameVector3D> linearAccelerationsInWorld = new ArrayList<>();
   private final List<YoDouble> linearAccelerationMagnitudes = new ArrayList<>();

   private final BooleanProvider isAccelerationIncludingGravity;
   private final BooleanProvider enableIMUBiasCompensation;
   private final DoubleProvider imuBiasEstimationThreshold;
   private final DoubleProvider biasFilterBreakFrequency;

   private final List<YoDouble> feetToIMUAngularVelocityMagnitudes = new ArrayList<>();
   private final List<YoDouble> feetToIMULinearVelocityMagnitudes = new ArrayList<>();
   private final List<YoBoolean> isBiasEstimated = new ArrayList<>();
   private final List<YoBoolean> isIMUOrientationBiasEstimated = new ArrayList<>();

   private final List<? extends IMUSensorReadOnly> imuProcessedOutputs;
   private final Map<IMUSensorReadOnly, Integer> imuToIndexMap = new HashMap<>();
   private final List<RigidBodyBasics> feet;
   private final BooleanParameter alwaysTrustFeet = new BooleanParameter("imuBiasEstimatorAlwaysTrustFeet", registry, false);

   private final Vector3D gravityVectorInWorld = new Vector3D();
   private final Vector3D zUpVector = new Vector3D();


   public IMUBiasStateEstimator(List<? extends IMUSensorReadOnly> imuProcessedOutputs, Collection<RigidBodyBasics> feet, double gravitationalAcceleration,
                                BooleanProvider cancelGravityFromAccelerationMeasurement, double updateDT, YoVariableRegistry parentRegistry)
   {
      this(imuProcessedOutputs, feet, gravitationalAcceleration, cancelGravityFromAccelerationMeasurement, updateDT, null, parentRegistry);
   }
   
   public IMUBiasStateEstimator(List<? extends IMUSensorReadOnly> imuProcessedOutputs, Collection<RigidBodyBasics> feet, double gravitationalAcceleration,
                                BooleanProvider cancelGravityFromAccelerationMeasurement, double updateDT, StateEstimatorParameters stateEstimatorParameters,
                                YoVariableRegistry parentRegistry)
   {
      this.imuProcessedOutputs = imuProcessedOutputs;
      this.feet = new ArrayList<>(feet);


      gravityVectorInWorld.set(0.0, 0.0, -Math.abs(gravitationalAcceleration));
      zUpVector.set(0.0, 0.0, 1.0);

      isAccelerationIncludingGravity = cancelGravityFromAccelerationMeasurement;
      if(stateEstimatorParameters != null)
      {
         enableIMUBiasCompensation = new BooleanParameter("enableIMUBiasCompensation", registry, stateEstimatorParameters.enableIMUBiasCompensation());
         biasFilterBreakFrequency = new DoubleParameter("biasFilterBreakFrequency", registry, stateEstimatorParameters.getIMUBiasFilterFreqInHertz());
         imuBiasEstimationThreshold = new DoubleParameter("imuBiasEstimationThreshold", registry, stateEstimatorParameters.getIMUBiasVelocityThreshold());
      }
      else
      {
         enableIMUBiasCompensation = new BooleanParameter("enableIMUBiasCompensation", registry);
         biasFilterBreakFrequency = new DoubleParameter("biasFilterBreakFrequency", registry);
         imuBiasEstimationThreshold = new DoubleParameter("imuBiasEstimationThreshold", registry);
      }

      
      
      AlphaBasedOnBreakFrequencyProvider alphaProvider = new AlphaBasedOnBreakFrequencyProvider(() ->  biasFilterBreakFrequency.getValue(), updateDT);
      for (int i = 0; i < imuProcessedOutputs.size(); i++)
      {
         IMUSensorReadOnly imuSensor = imuProcessedOutputs.get(i);
         ReferenceFrame measurementFrame = imuSensor.getMeasurementFrame();
         String sensorName = imuSensor.getSensorName();
         sensorName = sensorName.replaceFirst(imuSensor.getMeasurementLink().getName(), "");
         sensorName = FormattingTools.underscoredToCamelCase(sensorName, true);

         imuToIndexMap.put(imuSensor, i);

         AlphaFilteredYoFrameVector angularVelocityBias = new AlphaFilteredYoFrameVector("estimated" + sensorName + "AngularVelocityBias", "", registry, alphaProvider, measurementFrame);
         angularVelocityBiases.add(angularVelocityBias);

         AlphaFilteredYoFrameVector linearAccelerationBias = new AlphaFilteredYoFrameVector("estimated" + sensorName + "LinearAccelerationBias", "", registry, alphaProvider, measurementFrame);
         linearAccelerationBiases.add(linearAccelerationBias);

         YoFrameQuaternion rawOrientationBias = new YoFrameQuaternion("estimated" + sensorName + "RawQuaternionBias", measurementFrame, registry);
         rawOrientationBiases.add(rawOrientationBias);

         AlphaFilteredYoFrameQuaternion orientationBias = new AlphaFilteredYoFrameQuaternion("estimated" + sensorName + "QuaternionBias", "", rawOrientationBias, alphaProvider, registry);
         orientationBiases.add(orientationBias);

         angularVelocitiesInWorld.add(new YoFrameVector3D("unprocessed" + sensorName + "AngularVelocityInWorld", worldFrame, registry));
         
         linearAccelerationsInWorld.add(new YoFrameVector3D("unprocessed" + sensorName + "LinearAccelerationWorld", worldFrame, registry));
         linearAccelerationMagnitudes.add(new YoDouble("unprocessed" + sensorName + "LinearAccelerationMagnitude", registry));

         orientationBiasMagnitudes.add(new YoDouble("estimated" + sensorName + "OrientationBiasMagnitude", registry));

         feetToIMUAngularVelocityMagnitudes.add(new YoDouble("feetTo" + sensorName + "AngularVelocityMagnitude", registry));
         feetToIMULinearVelocityMagnitudes.add(new YoDouble("feetTo" + sensorName + "LinearVelocityMagnitude", registry));
         isBiasEstimated.add(new YoBoolean("is" + sensorName + "BiasEstimated", registry));
         isIMUOrientationBiasEstimated.add(new YoBoolean("is" + sensorName + "OrientationBiasEstimated", registry));

         angularVelocityBiasesInWorld.add(new YoFrameVector3D("estimated" + sensorName + "AngularVelocityBiasWorld", worldFrame, registry));
         linearAccelerationBiasesInWorld.add(new YoFrameVector3D("estimated" + sensorName + "LinearAccelerationBiasWorld", worldFrame, registry));
      }

      parentRegistry.addChild(registry);
   }
   
   public void initialize()
   {    
      for (int imuIndex = 0; imuIndex < imuProcessedOutputs.size(); imuIndex++)
      {
         angularVelocityBiases.get(imuIndex).update(0.0, 0.0, 0.0);
         linearAccelerationBiases.get(imuIndex).update(0.0, 0.0, 0.0);
         orientationBiases.get(imuIndex).update();
         
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

   public void compute(List<RigidBodyBasics> trustedFeet)
   {
      if (alwaysTrustFeet.getValue())
         trustedFeet = feet;

      checkIfBiasEstimationPossible(trustedFeet);
      estimateBiases();
   }

   private void checkIfBiasEstimationPossible(List<RigidBodyBasics> trustedFeet)
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
         RigidBodyBasics measurementLink = imuSensor.getMeasurementLink();

         double feetToIMUAngularVelocityMagnitude = 0.0;
         double feetToIMULinearVelocityMagnitude = 0.0;

         for (int footIndex = 0; footIndex < trustedFeet.size(); footIndex++)
         {
            RigidBodyBasics trustedFoot = trustedFeet.get(footIndex);

            measurementLink.getBodyFixedFrame().getTwistRelativeToOther(trustedFoot.getBodyFixedFrame(), twist);
            feetToIMUAngularVelocityMagnitude += twist.getAngularPart().length();
            feetToIMULinearVelocityMagnitude += twist.getLinearPart().length();
         }

         feetToIMUAngularVelocityMagnitudes.get(imuIndex).set(feetToIMUAngularVelocityMagnitude);
         feetToIMULinearVelocityMagnitudes.get(imuIndex).set(feetToIMULinearVelocityMagnitude);

         if (feetToIMUAngularVelocityMagnitude < imuBiasEstimationThreshold.getValue()
               && feetToIMULinearVelocityMagnitude < imuBiasEstimationThreshold.getValue())
         {
            isBiasEstimated.get(imuIndex).set(true);
            isIMUOrientationBiasEstimated.get(imuIndex).set(isAccelerationIncludingGravity.getValue());
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
            orientationMeasurement.set(imuSensor.getOrientationMeasurement());
            orientationMeasurementTransposed.setAndTranspose(orientationMeasurement);

            measurement.set(imuSensor.getAngularVelocityMeasurement());
            angularVelocityBiases.get(imuIndex).update(measurement);
            measurementBias.set(angularVelocityBiases.get(imuIndex));
            orientationMeasurement.transform(measurementBias);
            angularVelocityBiasesInWorld.get(imuIndex).set(measurementBias);

            orientationMeasurement.transform(measurement, measurementInWorld);
            angularVelocitiesInWorld.get(imuIndex).set(measurementInWorld);

            measurement.set(imuSensor.getLinearAccelerationMeasurement());
            orientationMeasurement.transform(measurement, measurementInWorld);
            linearAccelerationsInWorld.get(imuIndex).set(measurementInWorld);

            if (isAccelerationIncludingGravity.getValue())
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
               biasAxisAngle.set(yoOrientationBias);
               orientationBiasMagnitudes.get(imuIndex).set(Math.abs(biasAxisAngle.getAngle()));

               measurementMinusGravityInWorld.add(measurementInWorld, gravityVectorInWorld);
               orientationMeasurementTransposed.transform(measurementMinusGravityInWorld, measurementMinusGravity);
               linearAccelerationBiases.get(imuIndex).update(measurementMinusGravity);
            }
            else
            {
               linearAccelerationBiases.get(imuIndex).update(measurement);
            }

            measurementBias.set(linearAccelerationBiases.get(imuIndex));
            orientationMeasurement.transform(measurementBias);
            linearAccelerationBiasesInWorld.get(imuIndex).set(measurementBias);
         }
      }
   }

   @Override
   public void getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu, Vector3D angularVelocityBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getValue() || imuIndex == null)
         angularVelocityBiasToPack.set(0.0, 0.0, 0.0);
      else
         angularVelocityBiasToPack.set(angularVelocityBiases.get(imuIndex.intValue()));
   }

   @Override
   public void getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu, FrameVector3D angularVelocityBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getValue() || imuIndex == null)
         angularVelocityBiasToPack.setToZero(imu.getMeasurementFrame());
      else
         angularVelocityBiasToPack.setIncludingFrame(angularVelocityBiases.get(imuIndex.intValue()));
   }

   @Override
   public void getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu, Vector3D angularVelocityBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getValue() || imuIndex == null)
         angularVelocityBiasToPack.set(0.0, 0.0, 0.0);
      else
         angularVelocityBiasToPack.set(angularVelocityBiasesInWorld.get(imuIndex.intValue()));
   }

   @Override
   public void getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu, FrameVector3D angularVelocityBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getValue() || imuIndex == null)
         angularVelocityBiasToPack.setToZero(worldFrame);
      else
         angularVelocityBiasToPack.setIncludingFrame(angularVelocityBiasesInWorld.get(imuIndex.intValue()));
   }

   @Override
   public void getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu, Vector3D linearAccelerationBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getValue() || imuIndex == null)
         linearAccelerationBiasToPack.set(0.0, 0.0, 0.0);
      else
         linearAccelerationBiasToPack.set(linearAccelerationBiases.get(imuIndex.intValue()));
   }

   @Override
   public void getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu, FrameVector3D linearAccelerationBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getValue() || imuIndex == null)
         linearAccelerationBiasToPack.setToZero(imu.getMeasurementFrame());
      else
         linearAccelerationBiasToPack.setIncludingFrame(linearAccelerationBiases.get(imuIndex.intValue()));
   }

   @Override
   public void getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu, Vector3D linearAccelerationBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getValue() || imuIndex == null)
         linearAccelerationBiasToPack.set(0.0, 0.0, 0.0);
      else
         linearAccelerationBiasToPack.set(linearAccelerationBiasesInWorld.get(imuIndex.intValue()));
   }

   @Override
   public void getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu, FrameVector3D linearAccelerationBiasToPack)
   {
      Integer imuIndex = imuToIndexMap.get(imu);
      if (!enableIMUBiasCompensation.getValue() || imuIndex == null)
         linearAccelerationBiasToPack.setToZero(worldFrame);
      else
         linearAccelerationBiasToPack.setIncludingFrame(linearAccelerationBiasesInWorld.get(imuIndex.intValue()));
   }
}
