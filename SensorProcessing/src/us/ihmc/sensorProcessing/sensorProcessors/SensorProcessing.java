package us.ihmc.sensorProcessing.sensorProcessors;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.imu.IMUSensor;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoFrameQuaternion;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.BacklashCompensatingVelocityYoVariable;
import us.ihmc.yoUtilities.math.filters.FilteredVelocityYoVariable;
import us.ihmc.yoUtilities.math.filters.ProcessingYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameQuaternion;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class SensorProcessing implements SensorOutputMapReadOnly
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final LongYoVariable timestamp = new LongYoVariable("timestamp", registry);
   private final LongYoVariable visionSensorTimestamp = new LongYoVariable("visionSensorTimestamp", registry);

   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> inputJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> inputJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> inputJointTaus = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, YoFrameQuaternion> inputOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> inputAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> inputLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, YoFrameQuaternion> intermediateOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> intermediateAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> intermediateLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointVelocities = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> outputJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> outputJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> outputJointTaus = new LinkedHashMap<>();

   private final ArrayList<IMUSensor> outputIMUs = new ArrayList<IMUSensor>();

   private final List<OneDoFJoint> jointSensorDefinitions;
   private final List<IMUDefinition> imuSensorDefinitions;

   private final double updateDT;

   private final Matrix3d tempOrientation = new Matrix3d();
   private final Vector3d tempAngularVelocity = new Vector3d();
   private final Vector3d tempLinearAcceleration = new Vector3d();

   public SensorProcessing(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, SensorProcessingConfiguration sensorProcessingConfiguration,
         YoVariableRegistry parentRegistry)
   {
      this.updateDT = sensorProcessingConfiguration.getEstimatorDT();

      jointSensorDefinitions = stateEstimatorSensorDefinitions.getJointSensorDefinitions();
      imuSensorDefinitions = stateEstimatorSensorDefinitions.getIMUSensorDefinitions();

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         
         DoubleYoVariable rawJointPosition = new DoubleYoVariable("raw_q_" + jointName, registry);
         inputJointPositions.put(oneDoFJoint, rawJointPosition);
         outputJointPositions.put(oneDoFJoint, rawJointPosition);
         processedJointPositions.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         DoubleYoVariable rawJointVelocity = new DoubleYoVariable("raw_qd_" + jointName, registry);
         inputJointVelocities.put(oneDoFJoint, rawJointVelocity);
         outputJointVelocities.put(oneDoFJoint, rawJointVelocity);
         processedJointVelocities.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         DoubleYoVariable rawJointTau = new DoubleYoVariable("raw_tau_" + jointName, registry);
         inputJointTaus.put(oneDoFJoint, rawJointTau);
         outputJointTaus.put(oneDoFJoint, rawJointTau);
      }

      SensorNoiseParameters sensorNoiseParameters = sensorProcessingConfiguration.getSensorNoiseParameters();

      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();

         YoFrameQuaternion rawOrientation = new YoFrameQuaternion("raw_q_", imuName, worldFrame, registry);
         inputOrientations.put(imuDefinition, rawOrientation);
         intermediateOrientations.put(imuDefinition, rawOrientation);
         processedOrientations.put(imuDefinition, new ArrayList<ProcessingYoVariable>());

         YoFrameVector rawAngularVelocity = new YoFrameVector("raw_qd_w", imuName, worldFrame, registry);
         inputAngularVelocities.put(imuDefinition, rawAngularVelocity);
         intermediateAngularVelocities.put(imuDefinition, rawAngularVelocity);
         processedAngularVelocities.put(imuDefinition, new ArrayList<ProcessingYoVariable>());
         
         YoFrameVector rawLinearAcceleration = new YoFrameVector("raw_qdd_", imuName, worldFrame, registry);
         inputLinearAccelerations.put(imuDefinition, rawLinearAcceleration);
         intermediateLinearAccelerations.put(imuDefinition, rawLinearAcceleration);
         processedLinearAccelerations.put(imuDefinition, new ArrayList<ProcessingYoVariable>());
         
         outputIMUs.add(new IMUSensor(imuDefinition, sensorNoiseParameters));
      }

      sensorProcessingConfiguration.configureSensorProcessing(this);
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      startComputation(0, 0);
   }

   public void startComputation(long timestamp, long visionSensorTimestamp)
   {
      this.timestamp.set(timestamp);
      this.visionSensorTimestamp.set(visionSensorTimestamp);

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         
         updateProcessors(processedJointPositions.get(oneDoFJoint));
         updateProcessors(processedJointVelocities.get(oneDoFJoint));
      }
      
      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         updateProcessors(processedOrientations.get(imuDefinition));
         updateProcessors(processedAngularVelocities.get(imuDefinition));
         updateProcessors(processedLinearAccelerations.get(imuDefinition));
         
         IMUSensor outputIMU = outputIMUs.get(i);
         intermediateOrientations.get(imuDefinition).get(tempOrientation);
         intermediateAngularVelocities.get(imuDefinition).get(tempAngularVelocity);
         intermediateLinearAccelerations.get(imuDefinition).get(tempLinearAcceleration);
         outputIMU.setOrientationMeasurement(tempOrientation);
         outputIMU.setAngularVelocityMeasurement(tempAngularVelocity);
         outputIMU.setLinearAccelerationMeasurement(tempLinearAcceleration);
      }
   }

   private void updateProcessors(List<ProcessingYoVariable> processors)
   {
      for (int j = 0; j < processors.size(); j++)
      {
         processors.get(j).update();
      }
   }

   public void addJointPositionAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         DoubleYoVariable intermediateJointPosition = outputJointPositions.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointPositions.get(oneDoFJoint);
         String suffix = "_sp" + processors.size();
         AlphaFilteredYoVariable filteredJointPosition = new AlphaFilteredYoVariable("filt_q_" + jointName + suffix, registry, alphaFilter, intermediateJointPosition);
         processedJointPositions.get(oneDoFJoint).add(filteredJointPosition);
         
         if (!forVizOnly)
            outputJointPositions.put(oneDoFJoint, filteredJointPosition);
      }
   }

   public void addJointPositionElasticyCompensator(Map<OneDoFJoint, DoubleYoVariable> stiffnesses, boolean forVizOnly)
   {
      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         DoubleYoVariable stiffness = stiffnesses.get(oneDoFJoint);
         DoubleYoVariable intermediateJointPosition = outputJointPositions.get(oneDoFJoint);
         DoubleYoVariable intermediateJointTau = outputJointTaus.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointPositions.get(oneDoFJoint);
         String suffix = "_sp" + processors.size();
         ElasticityCompensatorYoVariable filteredJointPosition = new ElasticityCompensatorYoVariable("stiff_q_" + jointName + suffix, stiffness, intermediateJointPosition, intermediateJointTau, registry);
         processors.add(filteredJointPosition);
         
         if (!forVizOnly)
            outputJointPositions.put(oneDoFJoint, filteredJointPosition);
      }
   }

   public void computeJointVelocityFromFiniteDifference(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         DoubleYoVariable intermediateJointPosition = outputJointPositions.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointVelocities.get(oneDoFJoint);
         String suffix = "_sp" + processors.size();
         FilteredVelocityYoVariable jointVelocity = new FilteredVelocityYoVariable("fd_qd_" + jointName + suffix, "", alphaFilter, intermediateJointPosition, updateDT, registry);
         processors.add(jointVelocity);
         
         if (!forVizOnly)
            outputJointVelocities.put(oneDoFJoint, jointVelocity);
      }
   }

   public void computeJointVelocityWithBacklashCompensator(DoubleYoVariable alphaFilter, DoubleYoVariable slopTime, boolean forVizOnly)
   {
      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         DoubleYoVariable intermediateJointPosition = outputJointPositions.get(oneDoFJoint);

         List<ProcessingYoVariable> processors = processedJointVelocities.get(oneDoFJoint);
         String suffix = "_sp" + processors.size();
         BacklashCompensatingVelocityYoVariable jointVelocity = new BacklashCompensatingVelocityYoVariable("bl_qd_" + jointName + suffix, "", alphaFilter, intermediateJointPosition, updateDT, slopTime, registry);
         processors.add(jointVelocity);

         if (!forVizOnly)
            outputJointVelocities.put(oneDoFJoint, jointVelocity);
      }
   }

   public void addJointVelocityAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         DoubleYoVariable intermediateJointVelocity = outputJointVelocities.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointVelocities.get(oneDoFJoint);
         String suffix = "_sp" + processors.size();
         AlphaFilteredYoVariable filteredJointVelocity = new AlphaFilteredYoVariable("filt_qd_" + jointName + suffix, registry, alphaFilter, intermediateJointVelocity);
         processors.add(filteredJointVelocity);
         
         if (!forVizOnly)
            outputJointVelocities.put(oneDoFJoint, filteredJointVelocity);
      }
   }

   public void addOrientationAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();
         YoFrameQuaternion intermediateOrientation = intermediateOrientations.get(imuDefinition);
         List<ProcessingYoVariable> processors = processedOrientations.get(imuDefinition);
         String suffix = "_sp" + processors.size();
         AlphaFilteredYoFrameQuaternion filteredOrientation = new AlphaFilteredYoFrameQuaternion("filt_q_", imuName + suffix, intermediateOrientation, alphaFilter, registry);
         processors.add(filteredOrientation);
         
         if (!forVizOnly)
            intermediateOrientations.put(imuDefinition, filteredOrientation);
      }
   }

   public void addAngularVelocityAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();
         YoFrameVector intermediateAngularVelocity = intermediateAngularVelocities.get(imuDefinition);
         List<ProcessingYoVariable> processors = processedAngularVelocities.get(imuDefinition);
         String suffix = "_sp" + processors.size();
         AlphaFilteredYoFrameVector filteredAngularVelocity = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("prefilt_qd_w", imuName + suffix, registry, alphaFilter, intermediateAngularVelocity);
         processors.add(filteredAngularVelocity);
         
         if (!forVizOnly)
            intermediateAngularVelocities.put(imuDefinition, filteredAngularVelocity);
      }
   }

   public void addLinearAccelerationAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();
         YoFrameVector intermediateLinearAcceleration = intermediateLinearAccelerations.get(imuDefinition);
         List<ProcessingYoVariable> processors = processedLinearAccelerations.get(imuDefinition);
         String suffix = "_sp" + processors.size();
         AlphaFilteredYoFrameVector filteredLinearAcceleration = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("filt_qdd_", imuName + suffix, registry, alphaFilter, intermediateLinearAcceleration);
         processors.add(filteredLinearAcceleration);
         
         if (!forVizOnly)
            intermediateLinearAccelerations.put(imuDefinition, filteredLinearAcceleration);
      }
   }

   public DoubleYoVariable createAlphaFilter(String name, double breakFrequency)
   {
      DoubleYoVariable alphaFilter = new DoubleYoVariable(name, registry);
      alphaFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency, updateDT));
      return alphaFilter;
   }

   public Map<OneDoFJoint, DoubleYoVariable> createStiffness(String nameSuffix, double defaultStiffness, Map<String, Double> jointSpecificStiffness)
   {
      LinkedHashMap<OneDoFJoint, DoubleYoVariable> stiffesses = new LinkedHashMap<>();
      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         DoubleYoVariable stiffness = new DoubleYoVariable(jointName + nameSuffix, registry);

         if (jointSpecificStiffness != null && jointSpecificStiffness.containsKey(jointName))
            stiffness.set(jointSpecificStiffness.get(jointName));
         else
            stiffness.set(defaultStiffness);
         
         stiffesses.put(oneDoFJoint, stiffness);
      }
      
      return stiffesses;
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

   public void setJointPositionSensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointPositions.get(oneDoFJoint).set(value);
   }

   public void setJointVelocitySensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointVelocities.get(oneDoFJoint).set(value);
   }

   public void setJointTauSensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointTaus.get(oneDoFJoint).set(value);
   }

   public void setOrientationSensorValue(IMUDefinition imuDefinition, Quat4d value)
   {
      inputOrientations.get(imuDefinition).set(value);
   }

   public void setOrientationSensorValue(IMUDefinition imuDefinition, Matrix3d value)
   {
      inputOrientations.get(imuDefinition).set(value);
   }

   public void setAngularVelocitySensorValue(IMUDefinition imuDefinition, Vector3d value)
   {
      inputAngularVelocities.get(imuDefinition).set(value);
   }

   public void setLinearAccelerationSensorValue(IMUDefinition imuDefinition, Vector3d value)
   {
      inputLinearAccelerations.get(imuDefinition).set(value);
   }

   @Override
   public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return outputJointPositions.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return outputJointVelocities.get(oneDoFJoint).getDoubleValue();
   }

   public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return outputJointTaus.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public List<? extends IMUSensorReadOnly> getIMUProcessedOutputs()
   {
      return outputIMUs;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
