package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.imu.IMUSensor;
import us.ihmc.sensorProcessing.simulatedSensors.SensorFilterParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.maps.ObjectObjectMap;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoFrameQuaternion;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.FilteredVelocityYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameQuaternion;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


public class SensorProcessing implements SensorOutputMapReadOnly
{
   private final boolean USE_TWO_POLES_FOR_VELOCITY_FILTERING = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final LongYoVariable timestamp = new LongYoVariable("timestamp", registry);
   private final LongYoVariable visionSensorTimestamp = new LongYoVariable("visionSensorTimestamp", registry);
   
   private final ObjectObjectMap<OneDoFJoint, DoubleYoVariable> jointPositionRawValues = new ObjectObjectMap<OneDoFJoint, DoubleYoVariable>();
   private final ObjectObjectMap<OneDoFJoint, ElasticityCompensatorYoVariable> jointPositionsDeflected = new ObjectObjectMap<OneDoFJoint, ElasticityCompensatorYoVariable>();
   private final ObjectObjectMap<OneDoFJoint, DoubleYoVariable> jointStiffnesses = new ObjectObjectMap<OneDoFJoint, DoubleYoVariable>();
   private final ObjectObjectMap<OneDoFJoint, AlphaFilteredYoVariable> jointPositionFilteredValues = new ObjectObjectMap<OneDoFJoint, AlphaFilteredYoVariable>();
   private final DoubleYoVariable jointPositionAlpha = new DoubleYoVariable("jointPositionAlpha", registry);
   
   private final ObjectObjectMap<OneDoFJoint, DoubleYoVariable> jointVelocityRawValues = new ObjectObjectMap<OneDoFJoint, DoubleYoVariable>();
   private final ObjectObjectMap<OneDoFJoint, FilteredVelocityYoVariable> jointVelocityRawValuesFromFiniteDifference = new ObjectObjectMap<OneDoFJoint, FilteredVelocityYoVariable>();
   private final ObjectObjectMap<OneDoFJoint, AlphaFilteredYoVariable> jointVelocityIntermediateValues;
   private final ObjectObjectMap<OneDoFJoint, AlphaFilteredYoVariable> jointVelocityFilteredValues = new ObjectObjectMap<OneDoFJoint, AlphaFilteredYoVariable>();
   private final DoubleYoVariable jointVelocityAlpha = new DoubleYoVariable("jointVelocityAlpha", registry);

   private final ObjectObjectMap<IMUDefinition, YoFrameQuaternion> orientationRawValues = new ObjectObjectMap<IMUDefinition, YoFrameQuaternion>();
   private final ObjectObjectMap<IMUDefinition, AlphaFilteredYoFrameQuaternion> orientationFilteredValues = new ObjectObjectMap<IMUDefinition,
                                                                                                               AlphaFilteredYoFrameQuaternion>();
   private final DoubleYoVariable orientationAlpha = new DoubleYoVariable("orientationAlpha", registry);

   private final ObjectObjectMap<IMUDefinition, YoFrameVector> angularVelocityRawValues = new ObjectObjectMap<IMUDefinition, YoFrameVector>();
   private final ObjectObjectMap<IMUDefinition, AlphaFilteredYoFrameVector> angularVelocityIntermediateValues;
   private final ObjectObjectMap<IMUDefinition, AlphaFilteredYoFrameVector> angularVelocityFilteredValues = new ObjectObjectMap<IMUDefinition, AlphaFilteredYoFrameVector>();
   private final DoubleYoVariable angularVelocityAlpha = new DoubleYoVariable("angularVelocityAlpha", registry);

   private final ObjectObjectMap<IMUDefinition, YoFrameVector> linearAccelerationRawValues = new ObjectObjectMap<IMUDefinition, YoFrameVector>();
   private final ObjectObjectMap<IMUDefinition, AlphaFilteredYoFrameVector> linearAccelerationFilteredValues = new ObjectObjectMap<IMUDefinition, AlphaFilteredYoFrameVector>();
   private final DoubleYoVariable linearAccelerationAlpha = new DoubleYoVariable("linearAccelerationAlpha", registry);
   
   private final ObjectObjectMap<OneDoFJoint, DoubleYoVariable> jointTauRawValues = new ObjectObjectMap<OneDoFJoint, DoubleYoVariable>();
   
   private final ArrayList<IMUSensor> imuOutputs = new ArrayList<IMUSensor>();

   private final boolean useTwoPolesForIMUFiltering;
   private final boolean doFiniteDifferenceForJointVelocities;

   private final boolean doElasticityCompensation;

   public SensorProcessing(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, SensorFilterParameters sensorFilterParameters,
                                      SensorNoiseParameters sensorNoiseParameters, YoVariableRegistry parentRegistry)
   {
      if (USE_TWO_POLES_FOR_VELOCITY_FILTERING)
      {
         jointVelocityIntermediateValues = new ObjectObjectMap<OneDoFJoint, AlphaFilteredYoVariable>();
      }
      else
      {
         jointVelocityIntermediateValues = null;
      }

      useTwoPolesForIMUFiltering = sensorFilterParameters.getUseTwoPolesForIMUFiltering();

      if (useTwoPolesForIMUFiltering)
      {
         angularVelocityIntermediateValues = new ObjectObjectMap<IMUDefinition, AlphaFilteredYoFrameVector>();
      }
      else
      {
         angularVelocityIntermediateValues = null;
      }
      
      doFiniteDifferenceForJointVelocities = sensorFilterParameters.getDoFiniteDifferenceForJointVelocities();
      doElasticityCompensation = sensorFilterParameters.isDoElasticityCompensation();
      
      double updateDT = sensorFilterParameters.getEstimatorDT();
      jointPositionAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(sensorFilterParameters.getJointPositionFilterFrequencyInHertz(), updateDT));
      jointVelocityAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(sensorFilterParameters.getJointVelocityFilterFrequencyInHertz(), updateDT));
      orientationAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(sensorFilterParameters.getOrientationFilterFrequencyInHertz(), updateDT));
      angularVelocityAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(sensorFilterParameters.getAngularVelocityFilterFrequencyInHertz(), updateDT));
      linearAccelerationAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(sensorFilterParameters.getLinearAccelerationFilterFrequencyInHertz(), updateDT));

      List<OneDoFJoint> jointPositionSensorDefinitions = stateEstimatorSensorDefinitions.getJointPositionSensorDefinitions();
      for (OneDoFJoint oneDoFJoint : jointPositionSensorDefinitions)
      {
         String jointName = oneDoFJoint.getName();
         DoubleYoVariable jointTauRawValue = new DoubleYoVariable("raw_tau_" + jointName, registry);
         jointTauRawValues.add(oneDoFJoint, jointTauRawValue);
      }
      
      for (OneDoFJoint oneDoFJoint : jointPositionSensorDefinitions)
      {
         String jointName = oneDoFJoint.getName();
         DoubleYoVariable rawJointPosition = new DoubleYoVariable("raw_q_" + jointName, registry);
         jointPositionRawValues.add(oneDoFJoint, rawJointPosition);
         
         if (doElasticityCompensation)
         {
            DoubleYoVariable jointStiffness = new DoubleYoVariable(jointName + "Stiffness", registry);
            HashMap<String, Double> jointSpecificStiffness = sensorFilterParameters.getJointSpecificStiffness();
            if (jointSpecificStiffness != null && jointSpecificStiffness.get(jointName) != null)
               jointStiffness.set(jointSpecificStiffness.get(jointName));
            else
               jointStiffness.set(sensorFilterParameters.getDefaultJointStiffness());
            jointStiffnesses.add(oneDoFJoint, jointStiffness);
            
            ElasticityCompensatorYoVariable jointPositionDeflected = new ElasticityCompensatorYoVariable("stiff_q_" + jointName, jointStiffness, rawJointPosition, jointTauRawValues.get(oneDoFJoint), parentRegistry);
            jointPositionsDeflected.add(oneDoFJoint, jointPositionDeflected);
            
            rawJointPosition = jointPositionDeflected;
         }

         AlphaFilteredYoVariable filteredJointPosition = new AlphaFilteredYoVariable("filt_q_" + jointName, registry, jointPositionAlpha, rawJointPosition);
         jointPositionFilteredValues.add(oneDoFJoint, filteredJointPosition);
      }

      List<OneDoFJoint> jointVelocitySensorDefinitions = stateEstimatorSensorDefinitions.getJointVelocitySensorDefinitions();
      for (OneDoFJoint oneDoFJoint : jointVelocitySensorDefinitions)
      {
         FilteredVelocityYoVariable jointVelocityRawValueFromFD = new FilteredVelocityYoVariable("fd_qd_" + oneDoFJoint.getName(), "", jointVelocityAlpha, jointPositionRawValues.get(oneDoFJoint), updateDT, registry);
         jointVelocityRawValuesFromFiniteDifference.add(oneDoFJoint, jointVelocityRawValueFromFD);

         DoubleYoVariable rawJointVelocity = new DoubleYoVariable("raw_qd_" + oneDoFJoint.getName(), registry);
         jointVelocityRawValues.add(oneDoFJoint, rawJointVelocity);
         
         if (doFiniteDifferenceForJointVelocities) rawJointVelocity = jointVelocityRawValueFromFD;
         
         DoubleYoVariable velocityToFilter;
         if (USE_TWO_POLES_FOR_VELOCITY_FILTERING)
         {
            AlphaFilteredYoVariable intermediateJointVelocity = new AlphaFilteredYoVariable("prefilt_qd_" + oneDoFJoint.getName(), registry, jointVelocityAlpha,
                  rawJointVelocity);
            jointVelocityIntermediateValues.add(oneDoFJoint, intermediateJointVelocity);
            velocityToFilter = intermediateJointVelocity;
         }
         else
         {
            velocityToFilter = rawJointVelocity;
         }
         
         AlphaFilteredYoVariable filteredJointVelocity = new AlphaFilteredYoVariable("filt_qd_" + oneDoFJoint.getName(), registry, jointVelocityAlpha,
               velocityToFilter);
         jointVelocityFilteredValues.add(oneDoFJoint, filteredJointVelocity);
      }

      // TODO Hack! Need to refactor StateEstimatorSensorDefinitions
      List<IMUDefinition> orientationSensorDefinitions = stateEstimatorSensorDefinitions.getOrientationSensorDefinitions();
      for (IMUDefinition imuDefinition : orientationSensorDefinitions)
      {
         YoFrameQuaternion rawOrientation = new YoFrameQuaternion("raw_q_", imuDefinition.getName(), ReferenceFrame.getWorldFrame(), registry);
         orientationRawValues.add(imuDefinition, rawOrientation);

         AlphaFilteredYoFrameQuaternion filteredOrientation = new AlphaFilteredYoFrameQuaternion("filt_q_", imuDefinition.getName(), rawOrientation,
                                                                 orientationAlpha, registry);
         orientationFilteredValues.add(imuDefinition, filteredOrientation);

         YoFrameVector rawAngularVelocity = new YoFrameVector("raw_qd_w", imuDefinition.getName(), ReferenceFrame.getWorldFrame(), registry);
         angularVelocityRawValues.add(imuDefinition, rawAngularVelocity);

         YoFrameVector angularVelocityToFilter;
         if (sensorFilterParameters.getUseTwoPolesForIMUFiltering())
         {
            AlphaFilteredYoFrameVector intermediateAngularVelocity = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("prefilt_qd_w", imuDefinition.getName(),
                  registry, angularVelocityAlpha, rawAngularVelocity);
            angularVelocityIntermediateValues.add(imuDefinition, intermediateAngularVelocity);
            angularVelocityToFilter = intermediateAngularVelocity;
         }
         else
         {
            angularVelocityToFilter = rawAngularVelocity;
         }
         
         AlphaFilteredYoFrameVector filteredAngularVelocity = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("filt_qd_w", imuDefinition.getName(),
               registry, angularVelocityAlpha, angularVelocityToFilter);
         angularVelocityFilteredValues.add(imuDefinition, filteredAngularVelocity);

         YoFrameVector rawLinearAcceleration = new YoFrameVector("raw_qdd_", imuDefinition.getName(), ReferenceFrame.getWorldFrame(), registry);
         linearAccelerationRawValues.add(imuDefinition, rawLinearAcceleration);

         AlphaFilteredYoFrameVector filteredLinearAcceleration = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("filt_qdd_",
                                                                    imuDefinition.getName(), registry, linearAccelerationAlpha, rawLinearAcceleration);
         linearAccelerationFilteredValues.add(imuDefinition, filteredLinearAcceleration);

         IMUSensor imuSensor = new IMUSensor(imuDefinition, sensorNoiseParameters);
         imuOutputs.add(imuSensor);
      }
      

      parentRegistry.addChild(registry);
   }

   public void setJointPositionSensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      jointPositionRawValues.get(oneDoFJoint).set(value);
   }

   public void setJointVelocitySensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      jointVelocityRawValues.get(oneDoFJoint).set(value);
   }

   public void setJointTauSensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      jointTauRawValues.get(oneDoFJoint).set(value);
   }

   public void setOrientationSensorValue(IMUDefinition imuDefinition, Quat4d value)
   {
      YoFrameQuaternion rawOrientation = orientationRawValues.get(imuDefinition);
      if (rawOrientation == null)
         System.out.println("imuDefinition = " + imuDefinition.getName());
      rawOrientation.set(value);
   }

   public void setOrientationSensorValue(IMUDefinition imuDefinition, Matrix3d value)
   {
      YoFrameQuaternion rawOrientation = orientationRawValues.get(imuDefinition);
      if (rawOrientation == null)
         System.out.println("imuDefinition = " + imuDefinition.getName());
      rawOrientation.set(value);
   }

   public void setAngularVelocitySensorValue(IMUDefinition imuDefinition, Vector3d value)
   {
      angularVelocityRawValues.get(imuDefinition).set(value);
   }

   public void setLinearAccelerationSensorValue(IMUDefinition imuDefinition, Vector3d value)
   {
      linearAccelerationRawValues.get(imuDefinition).set(value);
   }

   public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return jointPositionFilteredValues.get(oneDoFJoint).getDoubleValue();
   }

   public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return jointVelocityFilteredValues.get(oneDoFJoint).getDoubleValue();
   }
   
   public List<IMUSensor> getIMUProcessedOutputs()
   {
      return imuOutputs;
   }
   
   private final Matrix3d tempOrientation = new Matrix3d();
   private final Vector3d tempAngularVelocity = new Vector3d();
   private final Vector3d tempLinearAcceleration = new Vector3d();
   
   public void startComputation(long timestamp, long visionSensorTimestamp)
   {
      this.timestamp.set(timestamp);
      this.visionSensorTimestamp.set(visionSensorTimestamp);
      if (doElasticityCompensation)
      {
         for (int i = 0; i < jointPositionsDeflected.getLength(); i++)
         {
            jointPositionsDeflected.getSecond(i).update();
         }
      }

      for (int i = 0; i < jointPositionFilteredValues.getLength(); i++)
      {
         AlphaFilteredYoVariable jointPosition = jointPositionFilteredValues.getSecond(i);
         jointPosition.update();
      }

      for (int i = 0; i < jointVelocityFilteredValues.getLength(); i++)
      {
         jointVelocityRawValuesFromFiniteDifference.getSecond(i).update();
         if (USE_TWO_POLES_FOR_VELOCITY_FILTERING) jointVelocityIntermediateValues.getSecond(i).update();
         AlphaFilteredYoVariable jointVelocity = jointVelocityFilteredValues.getSecond(i);
         jointVelocity.update();
      }

      for (int i = 0; i < imuOutputs.size(); i++)
      {
         AlphaFilteredYoFrameQuaternion orientation = orientationFilteredValues.getSecond(i);
         orientation.update();

         if (useTwoPolesForIMUFiltering) angularVelocityIntermediateValues.getSecond(i).update();
         AlphaFilteredYoFrameVector angularVelocity = angularVelocityFilteredValues.getSecond(i);
         angularVelocity.update();

         AlphaFilteredYoFrameVector linearAcceleration = linearAccelerationFilteredValues.getSecond(i);
         linearAcceleration.update();
         
         orientation.get(tempOrientation);
         angularVelocity.get(tempAngularVelocity);
         linearAcceleration.get(tempLinearAcceleration);

         IMUSensor imuSensor = imuOutputs.get(i);
         imuSensor.setOrientationMeasurement(tempOrientation);
         imuSensor.setAngularVelocityMeasurement(tempAngularVelocity);
         imuSensor.setLinearAccelerationMeasurement(tempLinearAcceleration);
      }
   }

   public void initialize()
   {
      startComputation(0, 0);
   }

   @Override
   public long getTimestamp()
   {
      return timestamp.getLongValue();
   }

   @Override
   public long getVisionSensorTimestamp()
   {
      return visionSensorTimestamp.getLongValue();
   }
}
