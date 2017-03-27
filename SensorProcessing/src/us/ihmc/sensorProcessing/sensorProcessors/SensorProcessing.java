package us.ihmc.sensorProcessing.sensorProcessors;

import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameQuaternion;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.BacklashProcessingYoFrameVector;
import us.ihmc.robotics.math.filters.BacklashProcessingYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;
import us.ihmc.robotics.math.filters.RevisedBacklashCompensatingVelocityYoVariable;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticUpdatable;
import us.ihmc.sensorProcessing.diagnostic.IMUSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointForceTrackingDelayEstimator;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointFourierAnalysis;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OrientationAngularVelocityConsistencyChecker;
import us.ihmc.sensorProcessing.diagnostic.PositionVelocity1DConsistencyChecker;
import us.ihmc.sensorProcessing.diagnostic.WrenchSensorValidityChecker;
import us.ihmc.sensorProcessing.imu.IMUSensor;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;

public class SensorProcessing implements SensorOutputMapReadOnly, SensorRawOutputMapReadOnly
{
   private static final String RAW = "raw";
   private static final String BACKLASH = "bl";
   private static final String ALPHA_FILTER = "filt";
   private static final String FINITE_DIFFERENCE = "fd";
   private static final String ELASTICITY_COMPENSATOR = "stiff";

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public enum SensorType
   {
      JOINT_POSITION, JOINT_VELOCITY, JOINT_ACCELERATION, JOINT_TAU, TORQUE_SENSOR, FORCE_SENSOR, IMU_ORIENTATION, IMU_ANGULAR_VELOCITY, IMU_LINEAR_ACCELERATION;

      public boolean isJointSensor()
      {
         switch (this)
         {
         case JOINT_POSITION:
         case JOINT_VELOCITY:
         case JOINT_ACCELERATION:
         case JOINT_TAU:
            return true;
         default:
            return false;
         }
      }

      public boolean isWrenchSensor()
      {
         return this == SensorType.TORQUE_SENSOR || this == FORCE_SENSOR;
      }

      public boolean isIMUSensor()
      {
         switch (this)
         {
         case IMU_ORIENTATION:
         case IMU_ANGULAR_VELOCITY:
         case IMU_LINEAR_ACCELERATION:
            return true;
         default:
            return false;
         }
      }

      public String getProcessorNamePrefix(String filterNameLowerCaseNoTrailingUnderscore)
      {
         switch (this)
         {
         case JOINT_POSITION:
         case IMU_ORIENTATION:
            return filterNameLowerCaseNoTrailingUnderscore + "_q_";
         case JOINT_VELOCITY:
            return filterNameLowerCaseNoTrailingUnderscore + "_qd_";
         case JOINT_ACCELERATION:
         case IMU_LINEAR_ACCELERATION:
            return filterNameLowerCaseNoTrailingUnderscore + "_qdd_";
         case JOINT_TAU:
            return filterNameLowerCaseNoTrailingUnderscore + "_tau_";
         case IMU_ANGULAR_VELOCITY:
            return filterNameLowerCaseNoTrailingUnderscore + "_qd_w";
         case FORCE_SENSOR:
            return filterNameLowerCaseNoTrailingUnderscore + "_force_";
         case TORQUE_SENSOR:
            return filterNameLowerCaseNoTrailingUnderscore + "_torque_";
         default:
            throw new RuntimeException("Should not get there.");
         }
      }

      public String getProcessorNameSuffix(String sensorName, int processorIndex)
      {
         String processorSignature = processorIndex >= 0 ? "_sp" + processorIndex : "";
         switch (this)
         {
         case JOINT_POSITION:
         case JOINT_VELOCITY:
         case JOINT_ACCELERATION:
         case JOINT_TAU:
            return sensorName + processorSignature;
         case IMU_ORIENTATION:
         case IMU_LINEAR_ACCELERATION:
         case IMU_ANGULAR_VELOCITY:
         case FORCE_SENSOR:
         case TORQUE_SENSOR:
            return "_" + sensorName + processorSignature;
         default:
            throw new RuntimeException("Should not get there.");
         }
      }
   };

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final LongYoVariable timestamp = new LongYoVariable("timestamp", registry);
   private final LongYoVariable visionSensorTimestamp = new LongYoVariable("visionSensorTimestamp", registry);
   private final LongYoVariable sensorHeadPPSTimetamp = new LongYoVariable("sensorHeadPPSTimetamp", registry);

   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> inputJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> inputJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> inputJointAccelerations = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> inputJointTaus = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, YoFrameQuaternion> inputOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> inputAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> inputLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector> inputForces = new LinkedHashMap<>();
   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector> inputTorques = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, YoFrameQuaternion> intermediateOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> intermediateAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector> intermediateLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector> intermediateForces = new LinkedHashMap<>();
   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector> intermediateTorques = new LinkedHashMap<>();

   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointAccelerations = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointTaus = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<ForceSensorDefinition, List<ProcessingYoVariable>> processedForces = new LinkedHashMap<>();
   private final LinkedHashMap<ForceSensorDefinition, List<ProcessingYoVariable>> processedTorques = new LinkedHashMap<>();

   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> outputJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> outputJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> outputJointAccelerations = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> outputJointTaus = new LinkedHashMap<>();

   private final ArrayList<DiagnosticUpdatable> diagnosticModules = new ArrayList<>();

   private final ArrayList<IMUSensor> inputIMUs = new ArrayList<IMUSensor>();
   private final ArrayList<IMUSensor> outputIMUs = new ArrayList<IMUSensor>();

   private final ForceSensorDataHolder inputForceSensors;
   private final ForceSensorDataHolder outputForceSensors;

   private final List<OneDoFJoint> jointSensorDefinitions;
   private final List<IMUDefinition> imuSensorDefinitions;
   private final List<ForceSensorDefinition> forceSensorDefinitions;

   private final List<String> allJointSensorNames = new ArrayList<>();
   private final List<String> allIMUSensorNames = new ArrayList<>();
   private final List<String> allForceSensorNames = new ArrayList<>();

   private final LinkedHashMap<OneDoFJoint, BooleanYoVariable> jointEnabledIndicators = new LinkedHashMap<>();

   private final double updateDT;

   private final RotationMatrix tempOrientation = new RotationMatrix();
   private final Vector3D tempAngularVelocity = new Vector3D();
   private final Vector3D tempLinearAcceleration = new Vector3D();

   private final FrameVector tempForce = new FrameVector();
   private final FrameVector tempTorque = new FrameVector();
   private final Wrench tempWrench = new Wrench();

   private AuxiliaryRobotData auxiliaryRobotData;

   public SensorProcessing(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, SensorProcessingConfiguration sensorProcessingConfiguration,
         YoVariableRegistry parentRegistry)
   {
      this.updateDT = sensorProcessingConfiguration.getEstimatorDT();

      jointSensorDefinitions = stateEstimatorSensorDefinitions.getJointSensorDefinitions();
      imuSensorDefinitions = stateEstimatorSensorDefinitions.getIMUSensorDefinitions();
      forceSensorDefinitions = stateEstimatorSensorDefinitions.getForceSensorDefinitions();
      this.auxiliaryRobotData = null;

      String prefix = null;
      String suffix = null;

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         allJointSensorNames.add(jointName);

         prefix = JOINT_POSITION.getProcessorNamePrefix(RAW);
         suffix = JOINT_POSITION.getProcessorNameSuffix(jointName, -1);
         DoubleYoVariable rawJointPosition = new DoubleYoVariable(prefix + suffix, registry);
         inputJointPositions.put(oneDoFJoint, rawJointPosition);
         outputJointPositions.put(oneDoFJoint, rawJointPosition);
         processedJointPositions.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         prefix = JOINT_VELOCITY.getProcessorNamePrefix(RAW);
         suffix = JOINT_VELOCITY.getProcessorNameSuffix(jointName, -1);
         DoubleYoVariable rawJointVelocity = new DoubleYoVariable(prefix + suffix, registry);
         inputJointVelocities.put(oneDoFJoint, rawJointVelocity);
         outputJointVelocities.put(oneDoFJoint, rawJointVelocity);
         processedJointVelocities.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         prefix = JOINT_ACCELERATION.getProcessorNamePrefix(RAW);
         suffix = JOINT_ACCELERATION.getProcessorNameSuffix(jointName, -1);
         DoubleYoVariable rawJointAcceleration = new DoubleYoVariable(prefix + suffix, registry);
         inputJointAccelerations.put(oneDoFJoint, rawJointAcceleration);
         outputJointAccelerations.put(oneDoFJoint, rawJointAcceleration);
         processedJointAccelerations.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         prefix = JOINT_TAU.getProcessorNamePrefix(RAW);
         suffix = JOINT_TAU.getProcessorNameSuffix(jointName, -1);
         DoubleYoVariable rawJointTau = new DoubleYoVariable(prefix + suffix, registry);
         inputJointTaus.put(oneDoFJoint, rawJointTau);
         outputJointTaus.put(oneDoFJoint, rawJointTau);
         processedJointTaus.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         BooleanYoVariable jointEnabledIndicator = new BooleanYoVariable("joint_enabled_" + jointName, registry);
         jointEnabledIndicator.set(true);
         jointEnabledIndicators.put(oneDoFJoint, jointEnabledIndicator);
      }

      SensorNoiseParameters sensorNoiseParameters = sensorProcessingConfiguration.getSensorNoiseParameters();

      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();
         allIMUSensorNames.add(imuName);
         ReferenceFrame sensorFrame = imuDefinition.getIMUFrame();

         prefix = IMU_ORIENTATION.getProcessorNamePrefix(RAW);
         suffix = IMU_ORIENTATION.getProcessorNameSuffix(imuName, -1);
         YoFrameQuaternion rawOrientation = new YoFrameQuaternion(prefix, suffix, worldFrame, registry);
         inputOrientations.put(imuDefinition, rawOrientation);
         intermediateOrientations.put(imuDefinition, rawOrientation);
         processedOrientations.put(imuDefinition, new ArrayList<ProcessingYoVariable>());

         prefix = IMU_ANGULAR_VELOCITY.getProcessorNamePrefix(RAW);
         suffix = IMU_ANGULAR_VELOCITY.getProcessorNameSuffix(imuName, -1);
         YoFrameVector rawAngularVelocity = new YoFrameVector(prefix, suffix, sensorFrame, registry);
         inputAngularVelocities.put(imuDefinition, rawAngularVelocity);
         intermediateAngularVelocities.put(imuDefinition, rawAngularVelocity);
         processedAngularVelocities.put(imuDefinition, new ArrayList<ProcessingYoVariable>());

         prefix = IMU_LINEAR_ACCELERATION.getProcessorNamePrefix(RAW);
         suffix = IMU_LINEAR_ACCELERATION.getProcessorNameSuffix(imuName, -1);
         YoFrameVector rawLinearAcceleration = new YoFrameVector(prefix, suffix, sensorFrame, registry);
         inputLinearAccelerations.put(imuDefinition, rawLinearAcceleration);
         intermediateLinearAccelerations.put(imuDefinition, rawLinearAcceleration);
         processedLinearAccelerations.put(imuDefinition, new ArrayList<ProcessingYoVariable>());
         
         inputIMUs.add(new IMUSensor(imuDefinition, sensorNoiseParameters));
         outputIMUs.add(new IMUSensor(imuDefinition, sensorNoiseParameters));
      }

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);
         String sensorName = forceSensorDefinition.getSensorName();
         allForceSensorNames.add(sensorName);
         ReferenceFrame sensorFrame = forceSensorDefinition.getSensorFrame();

         prefix = FORCE_SENSOR.getProcessorNamePrefix(RAW);
         suffix = FORCE_SENSOR.getProcessorNameSuffix(sensorName, -1);
         YoFrameVector rawForce = new YoFrameVector(prefix, suffix, sensorFrame, registry);
         inputForces.put(forceSensorDefinition, rawForce);
         intermediateForces.put(forceSensorDefinition, rawForce);
         processedForces.put(forceSensorDefinition, new ArrayList<ProcessingYoVariable>());

         prefix = TORQUE_SENSOR.getProcessorNamePrefix(RAW);
         suffix = TORQUE_SENSOR.getProcessorNameSuffix(sensorName, -1);
         YoFrameVector rawTorque = new YoFrameVector(prefix, suffix, sensorFrame, registry);
         inputTorques.put(forceSensorDefinition, rawTorque);
         intermediateTorques.put(forceSensorDefinition, rawTorque);
         processedTorques.put(forceSensorDefinition, new ArrayList<ProcessingYoVariable>());
      }

      inputForceSensors = new ForceSensorDataHolder(forceSensorDefinitions);
      outputForceSensors = new ForceSensorDataHolder(forceSensorDefinitions);

      sensorProcessingConfiguration.configureSensorProcessing(this);
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      startComputation(0, 0, -1);
   }

   public void startComputation(long timestamp, long visionSensorTimestamp, long sensorHeadPPSTimestamp)
   {
      this.timestamp.set(timestamp);
      this.visionSensorTimestamp.set(visionSensorTimestamp);
      this.sensorHeadPPSTimetamp.set(sensorHeadPPSTimestamp);

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);

         updateProcessors(processedJointPositions.get(oneDoFJoint));
         updateProcessors(processedJointVelocities.get(oneDoFJoint));
         updateProcessors(processedJointAccelerations.get(oneDoFJoint));
         updateProcessors(processedJointTaus.get(oneDoFJoint));
      }
      
      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);

         IMUSensor inputIMU = inputIMUs.get(i);
         inputOrientations.get(imuDefinition).get(tempOrientation);
         inputAngularVelocities.get(imuDefinition).get(tempAngularVelocity);
         inputLinearAccelerations.get(imuDefinition).get(tempLinearAcceleration);
         inputIMU.setOrientationMeasurement(tempOrientation);
         inputIMU.setAngularVelocityMeasurement(tempAngularVelocity);
         inputIMU.setLinearAccelerationMeasurement(tempLinearAcceleration);

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

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);

         inputForceSensors.getForceSensorValue(forceSensorDefinition, tempWrench);
         tempWrench.getLinearPartIncludingFrame(tempForce); 
         tempWrench.getAngularPartIncludingFrame(tempTorque); 
         inputForces.get(forceSensorDefinition).set(tempForce);
         inputTorques.get(forceSensorDefinition).set(tempTorque);
         
         updateProcessors(processedForces.get(forceSensorDefinition));
         updateProcessors(processedTorques.get(forceSensorDefinition));

         intermediateForces.get(forceSensorDefinition).getFrameTupleIncludingFrame(tempForce);
         intermediateTorques.get(forceSensorDefinition).getFrameTupleIncludingFrame(tempTorque);
         tempWrench.set(tempForce, tempTorque);
         outputForceSensors.setForceSensorValue(forceSensorDefinition, tempWrench);
      }

      for (int i = 0; i < diagnosticModules.size(); i++)
         diagnosticModules.get(i).update();
   }

   private void updateProcessors(List<ProcessingYoVariable> processors)
   {
      for (int j = 0; j < processors.size(); j++)
      {
         processors.get(j).update();
      }
   }

   /**
    * Add a low-pass filter stage for the given sensor signal.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public Map<String, Integer> addSensorAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly, SensorType sensorType)
   {
      return addSensorAlphaFilterWithSensorsToIgnore(alphaFilter, forVizOnly, sensorType);
   }

   /**
    * Add a low-pass filter stage for the given sensor signal.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param sensorsToBeProcessed list of the names of the sensors that need to be processed.
    */
   public Map<String, Integer> addSensorAlphaFilterOnlyForSpecifiedSensors(DoubleYoVariable alphaFilter, boolean forVizOnly, SensorType sensorType, String... sensorsToBeProcessed)
   {
      return addSensorAlphaFilterWithSensorsToIgnore(alphaFilter, forVizOnly, sensorType, invertSensorSelection(sensorType, sensorsToBeProcessed));
   }

   /**
    * Add a low-pass filter stage for the given sensor signal.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param sensorsToIgnore list of the names of the sensors to ignore.
    */
   public Map<String, Integer> addSensorAlphaFilterWithSensorsToIgnore(DoubleYoVariable alphaFilter, boolean forVizOnly, SensorType sensorType, String... sensorsToIgnore)
   {
      Map<String, Integer> processorIDMap;
      List<String> sensorToIgnoreList = new ArrayList<>();
      if (sensorsToIgnore != null && sensorsToIgnore.length > 0)
         sensorToIgnoreList.addAll(Arrays.asList(sensorsToIgnore));

      if (sensorType.isJointSensor())
         processorIDMap = addJointAlphaFilterWithJointsToIgnore(alphaFilter, forVizOnly, sensorType, sensorToIgnoreList);
      else if (sensorType.isWrenchSensor())
         processorIDMap = addForceSensorAlphaFilterWithSensorsToIgnore(alphaFilter, forVizOnly, sensorType, sensorToIgnoreList);
      else if (sensorType.isIMUSensor())
      {
         if (sensorType == SensorType.IMU_ORIENTATION)
            processorIDMap = addIMUOrientationAlphaFilterWithSensorsToIgnore(alphaFilter, forVizOnly, sensorToIgnoreList);
         else
            processorIDMap = addIMUVectorTypeDataAlphaFilter(alphaFilter, forVizOnly, sensorType, sensorToIgnoreList);
      }
      else
         throw new RuntimeException("Unknown type of sensor.");

      return Collections.unmodifiableMap(processorIDMap);
   }

   private Map<String, Integer> addIMUVectorTypeDataAlphaFilter(DoubleYoVariable alphaFilter, boolean forVizOnly, SensorType sensorType, List<String> sensorsToIgnore)
   {
      Map<String, Integer> processorsIDs = new HashMap<>();

      LinkedHashMap<IMUDefinition, YoFrameVector> intermediateIMUVectorTypeSignals = getIntermediateIMUVectorTypeSignals(sensorType);
      LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedIMUVectorTypeSignals = getProcessedIMUVectorTypeSignals(sensorType);

      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();

         if (sensorsToIgnore.contains(imuName))
            continue;

         YoFrameVector intermediateSignal = intermediateIMUVectorTypeSignals.get(imuDefinition);
         List<ProcessingYoVariable> processors = processedIMUVectorTypeSignals.get(imuDefinition);
         String prefix = sensorType.getProcessorNamePrefix(ALPHA_FILTER);
         int newProcessorID = processors.size();
         processorsIDs.put(imuName, newProcessorID);
         String suffix = sensorType.getProcessorNameSuffix(imuName, newProcessorID);
         AlphaFilteredYoFrameVector filter = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(prefix, suffix, registry, alphaFilter, intermediateSignal);
         processors.add(filter);
         
         if (!forVizOnly)
            intermediateIMUVectorTypeSignals.put(imuDefinition, filter);
      }

      return processorsIDs;
   }

   private Map<String, Integer> addForceSensorAlphaFilterWithSensorsToIgnore(DoubleYoVariable alphaFilter, boolean forVizOnly, SensorType sensorType, List<String> sensorsToIgnore)
   {
      Map<String, Integer> processorsIDs = new HashMap<>();

      LinkedHashMap<ForceSensorDefinition, YoFrameVector> intermediateForceSensorSignals = getIntermediateForceSensorSignals(sensorType);
      LinkedHashMap<ForceSensorDefinition, List<ProcessingYoVariable>> processedForceSensorSignals = getProcessedForceSensorSignals(sensorType);

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);
         String sensorName = forceSensorDefinition.getSensorName();

         if (sensorsToIgnore.contains(sensorName))
            continue;

         YoFrameVector intermediateSignal = intermediateForceSensorSignals.get(forceSensorDefinition);
         List<ProcessingYoVariable> processors = processedForceSensorSignals.get(forceSensorDefinition);
         String prefix = sensorType.getProcessorNamePrefix(ALPHA_FILTER);
         int newProcessorID = processors.size();
         processorsIDs.put(sensorName, newProcessorID);
         String suffix = sensorType.getProcessorNameSuffix(sensorName, newProcessorID);
         AlphaFilteredYoFrameVector filter = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(prefix, suffix, registry, alphaFilter, intermediateSignal);
         processors.add(filter);

         if (!forVizOnly)
            intermediateForceSensorSignals.put(forceSensorDefinition, filter);
      }

      return processorsIDs;
   }

   private Map<String, Integer> addJointAlphaFilterWithJointsToIgnore(DoubleYoVariable alphaFilter, boolean forVizOnly, SensorType sensorType, List<String> jointsToIgnore)
   {
      Map<String, Integer> processorsIDs = new HashMap<>();

      LinkedHashMap<OneDoFJoint, DoubleYoVariable> outputJointSignals = getOutputJointSignals(sensorType);
      LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> processedJointSignals = getProcessedJointSignals(sensorType);

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointsToIgnore.contains(jointName))
            continue;

         DoubleYoVariable intermediateJointSignal = outputJointSignals.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointSignals.get(oneDoFJoint);
         String prefix = sensorType.getProcessorNamePrefix(ALPHA_FILTER);
         int newProcessorID = processors.size();
         processorsIDs.put(jointName, newProcessorID);
         String suffix = sensorType.getProcessorNameSuffix(jointName, newProcessorID);
         AlphaFilteredYoVariable filter = new AlphaFilteredYoVariable(prefix + suffix, registry, alphaFilter, intermediateJointSignal);
         processedJointSignals.get(oneDoFJoint).add(filter);
         
         if (!forVizOnly)
            outputJointSignals.put(oneDoFJoint, filter);
      }

      return processorsIDs;
   }

   public void addJointPositionElasticyCompensator(Map<OneDoFJoint, DoubleYoVariable> stiffnesses, DoubleYoVariable maximumDeflection, boolean forVizOnly)
   {
      addJointPositionElasticyCompensatorWithJointsToIgnore(stiffnesses, maximumDeflection, null, forVizOnly);
   }

   public void addJointPositionElasticyCompensatorWithJointsToIgnore(Map<OneDoFJoint, DoubleYoVariable> stiffnesses, DoubleYoVariable maximumDeflection, boolean forVizOnly, String... jointsToIgnore)
   {
      addJointPositionElasticyCompensatorWithJointsToIgnore(stiffnesses, maximumDeflection, null, forVizOnly, jointsToIgnore);
   }

   /**
    * Apply an elasticity compensator to correct the joint positions according their torque and a given stiffness.
    * Useful when the robot has a non negligible elasticity in the links or joints.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param stiffnesses estimated stiffness for each joint.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void addJointPositionElasticyCompensator(Map<OneDoFJoint, DoubleYoVariable> stiffnesses, DoubleYoVariable maximumDeflection, Map<String, Integer> torqueProcessorIDs, boolean forVizOnly)
   {
      addJointPositionElasticyCompensatorWithJointsToIgnore(stiffnesses, maximumDeflection, forVizOnly);
   }

   public void addJointPositionElasticyCompensatorWithJointsToIgnore(Map<OneDoFJoint, DoubleYoVariable> stiffnesses, DoubleYoVariable maximumDeflection, Map<String, Integer> torqueProcessorIDs, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable stiffness = stiffnesses.get(oneDoFJoint);
         if (stiffness == null)
            continue;
         DoubleYoVariable intermediateJointPosition = outputJointPositions.get(oneDoFJoint);

         DoubleYoVariable intermediateJointTau;
         if (torqueProcessorIDs == null || !torqueProcessorIDs.containsKey(jointName))
            intermediateJointTau = outputJointTaus.get(oneDoFJoint);
         else
            intermediateJointTau = (DoubleYoVariable) processedJointTaus.get(oneDoFJoint).get(torqueProcessorIDs.get(jointName));

         List<ProcessingYoVariable> processors = processedJointPositions.get(oneDoFJoint);
         String prefix = JOINT_POSITION.getProcessorNamePrefix(ELASTICITY_COMPENSATOR);
         String suffix = JOINT_POSITION.getProcessorNameSuffix(jointName, processors.size());
         ElasticityCompensatorYoVariable filteredJointPosition = new ElasticityCompensatorYoVariable(prefix + suffix, stiffness, maximumDeflection, intermediateJointPosition, intermediateJointTau, registry);
         processors.add(filteredJointPosition);
         
         if (!forVizOnly)
            outputJointPositions.put(oneDoFJoint, filteredJointPosition);
      }
   }

   public void addJointVelocityElasticyCompensator(Map<OneDoFJoint, DoubleYoVariable> stiffnesses, DoubleYoVariable maximumDeflection, boolean forVizOnly)
   {
      addJointVelocityElasticyCompensatorWithJointsToIgnore(stiffnesses, maximumDeflection, null, forVizOnly);
   }

   public void addJointVelocityElasticyCompensatorWithJointsToIgnore(Map<OneDoFJoint, DoubleYoVariable> stiffnesses, DoubleYoVariable maximumDeflection, boolean forVizOnly, String... jointsToIgnore)
   {
      addJointVelocityElasticyCompensatorWithJointsToIgnore(stiffnesses, maximumDeflection, null, forVizOnly, jointsToIgnore);
   }

   /**
    * Apply an elasticity compensator to correct the joint positions according their torque and a given stiffness.
    * Useful when the robot has a non negligible elasticity in the links or joints.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param stiffnesses estimated stiffness for each joint.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void addJointVelocityElasticyCompensator(Map<OneDoFJoint, DoubleYoVariable> stiffnesses, DoubleYoVariable maximumDeflection, Map<String, Integer> torqueProcessorIDs, boolean forVizOnly)
   {
      addJointVelocityElasticyCompensatorWithJointsToIgnore(stiffnesses, maximumDeflection, forVizOnly);
   }

   public void addJointVelocityElasticyCompensatorWithJointsToIgnore(Map<OneDoFJoint, DoubleYoVariable> stiffnesses, DoubleYoVariable maximumDeflection, Map<String, Integer> torqueProcessorIDs, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable stiffness = stiffnesses.get(oneDoFJoint);
         if (stiffness == null)
            continue;
         DoubleYoVariable intermediateJointVelocity = outputJointVelocities.get(oneDoFJoint);

         DoubleYoVariable intermediateJointTau;
         if (torqueProcessorIDs == null || !torqueProcessorIDs.containsKey(jointName))
            intermediateJointTau = outputJointTaus.get(oneDoFJoint);
         else
            intermediateJointTau = (DoubleYoVariable) processedJointTaus.get(oneDoFJoint).get(torqueProcessorIDs.get(jointName));

         List<ProcessingYoVariable> processors = processedJointVelocities.get(oneDoFJoint);
         String prefix = JOINT_VELOCITY.getProcessorNamePrefix(ELASTICITY_COMPENSATOR);
         String suffix = JOINT_VELOCITY.getProcessorNameSuffix(jointName, processors.size());
         VelocityElasticityCompensatorYoVariable filteredJointVelocity = new VelocityElasticityCompensatorYoVariable(prefix + suffix, stiffness,
               maximumDeflection, intermediateJointVelocity, intermediateJointTau, updateDT, registry);
         processors.add(filteredJointVelocity);
         
         if (!forVizOnly)
            outputJointVelocities.put(oneDoFJoint, filteredJointVelocity);
      }
   }

   /**
    * Compute the joint velocities by calculating finite-difference on joint positions using {@link FilteredVelocityYoVariable}. It is then automatically low-pass filtered.
    * This is not cumulative and has the effect of ignoring the velocity signal provided by the robot.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void computeJointVelocityFromFiniteDifference(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      computeJointVelocityFromFiniteDifferenceWithJointsToIgnore(alphaFilter, forVizOnly);
   }

   /**
    * Compute the joint velocities (for a specific subset of joints) by calculating finite-difference on joint positions using {@link FilteredVelocityYoVariable}. It is then automatically low-pass filtered.
    * This is not cumulative and has the effect of ignoring the velocity signal provided by the robot.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToBeProcessed list of the names of the joints that need to be processed.
    */
   public void computeJointVelocityFromFiniteDifferenceOnlyForSpecifiedJoints(DoubleYoVariable alphaFilter, boolean forVizOnly, String... jointsToBeProcessed)
   {
      computeJointVelocityFromFiniteDifferenceWithJointsToIgnore(alphaFilter, forVizOnly, invertSensorSelection(allJointSensorNames, jointsToBeProcessed));
   }

   /**
    * Compute the joint velocities (for a specific subset of joints) by calculating finite-difference on joint positions using {@link FilteredVelocityYoVariable}. It is then automatically low-pass filtered.
    * This is not cumulative and has the effect of ignoring the velocity signal provided by the robot.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void computeJointVelocityFromFiniteDifferenceWithJointsToIgnore(DoubleYoVariable alphaFilter, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable intermediateJointPosition = outputJointPositions.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointVelocities.get(oneDoFJoint);
         String prefix = JOINT_VELOCITY.getProcessorNamePrefix(FINITE_DIFFERENCE);
         String suffix = JOINT_VELOCITY.getProcessorNameSuffix(jointName, processors.size());
         FilteredVelocityYoVariable jointVelocity = new FilteredVelocityYoVariable(prefix + suffix, "", alphaFilter, intermediateJointPosition, updateDT, registry);
         processors.add(jointVelocity);
         
         if (!forVizOnly)
            outputJointVelocities.put(oneDoFJoint, jointVelocity);
      }
   }

   
   /**
    * Compute the joint velocities by calculating finite-difference on joint positions and applying a backlash compensator (see {@link RevisedBacklashCompensatingVelocityYoVariable}). It is then automatically low-pass filtered.
    * This is not cumulative and has the effect of ignoring the velocity signal provided by the robot.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void computeJointVelocityWithBacklashCompensator(DoubleYoVariable alphaFilter, DoubleYoVariable slopTime, boolean forVizOnly)
   {
      computeJointVelocityWithBacklashCompensatorWithJointsToIgnore(alphaFilter, slopTime, forVizOnly);
   }

   /**
    * Compute the joint velocities (for a specific subset of joints) by calculating finite-difference on joint positions and applying a backlash compensator (see {@link RevisedBacklashCompensatingVelocityYoVariable}). It is then automatically low-pass filtered.
    * This is not cumulative and has the effect of ignoring the velocity signal provided by the robot.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToBeProcessed list of the names of the joints that need to be processed.
    */
   public void computeJointVelocityWithBacklashCompensatorOnlyForSpecifiedJoints(DoubleYoVariable alphaFilter, DoubleYoVariable slopTime, boolean forVizOnly, String... jointsToBeProcessed)
   {
      computeJointVelocityWithBacklashCompensatorWithJointsToIgnore(alphaFilter, slopTime, forVizOnly, invertSensorSelection(allJointSensorNames, jointsToBeProcessed));
   }

   /**
    * Compute the joint velocities (for a specific subset of joints) by calculating finite-difference on joint positions and applying a backlash compensator (see {@link RevisedBacklashCompensatingVelocityYoVariable}). It is then automatically low-pass filtered.
    * This is not cumulative and has the effect of ignoring the velocity signal provided by the robot.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void computeJointVelocityWithBacklashCompensatorWithJointsToIgnore(DoubleYoVariable alphaFilter, DoubleYoVariable slopTime, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable intermediateJointPosition = outputJointPositions.get(oneDoFJoint);

         List<ProcessingYoVariable> processors = processedJointVelocities.get(oneDoFJoint);
         String prefix = JOINT_VELOCITY.getProcessorNamePrefix(BACKLASH);
         String suffix = JOINT_VELOCITY.getProcessorNameSuffix(jointName, processors.size());
         RevisedBacklashCompensatingVelocityYoVariable jointVelocity = new RevisedBacklashCompensatingVelocityYoVariable(prefix + suffix, "", alphaFilter, intermediateJointPosition, updateDT, slopTime, registry);
         processors.add(jointVelocity);

         if (!forVizOnly)
            outputJointVelocities.put(oneDoFJoint, jointVelocity);
      }
      
   }

   /**
    * Apply a backlash compensator (see {@link BacklashProcessingYoVariable}) to the joint velocity.
    * Useful when the robot has backlash in its joints or simply to calm down small shakies when the robot is at rest.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param slopTime every time the velocity changes sign, a slop is engaged during which a confidence factor is ramped up from 0 to 1.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void addJointVelocityBacklashFilter(DoubleYoVariable slopTime, boolean forVizOnly)
   {
      addJointVelocityBacklashFilterWithJointsToIgnore(slopTime, forVizOnly);
   }

   /**
    * Apply a backlash compensator (see {@link BacklashProcessingYoVariable}) to the joint velocity.
    * Useful when the robot has backlash in its joints or simply to calm down small shakies when the robot is at rest.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param slopTime every time the velocity changes sign, a slop is engaged during which a confidence factor is ramped up from 0 to 1.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToBeProcessed list of the names of the joints that need to be filtered.
    */
   public void addJointVelocityBacklashFilterOnlyForSpecifiedJoints(DoubleYoVariable slopTime, boolean forVizOnly, String... jointsToBeProcessed)
   {
      addJointVelocityBacklashFilterWithJointsToIgnore(slopTime, forVizOnly, invertSensorSelection(allJointSensorNames, jointsToBeProcessed));
   }

   /**
    * Apply a backlash compensator (see {@link BacklashProcessingYoVariable}) to the joint velocity.
    * Useful when the robot has backlash in its joints or simply to calm down small shakies when the robot is at rest.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param slopTime every time the velocity changes sign, a slop is engaged during which a confidence factor is ramped up from 0 to 1.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void addJointVelocityBacklashFilterWithJointsToIgnore(DoubleYoVariable slopTime, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable intermediateJointVelocity = outputJointVelocities.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointVelocities.get(oneDoFJoint);
         String prefix = JOINT_VELOCITY.getProcessorNamePrefix(BACKLASH);
         String suffix = JOINT_VELOCITY.getProcessorNameSuffix(jointName, processors.size());
         BacklashProcessingYoVariable filteredJointVelocity = new BacklashProcessingYoVariable(prefix + suffix, "", intermediateJointVelocity, updateDT, slopTime, registry);
         processors.add(filteredJointVelocity);

         if (!forVizOnly)
            outputJointVelocities.put(oneDoFJoint, filteredJointVelocity);
      }
   }

   /**
    * Compute the joint accelerations by calculating finite-difference on joint velocities using {@link FilteredVelocityYoVariable}. It is then automatically low-pass filtered.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void computeJointAccelerationFromFiniteDifference(DoubleYoVariable alphaFilter, boolean forVizOnly)
   {
      computeJointAccelerationFromFiniteDifferenceWithJointsToIgnore(alphaFilter, forVizOnly);
   }

   /**
    * Compute the joint accelerations (for a specific subset of joints) by calculating finite-difference on joint velocities using {@link FilteredVelocityYoVariable}. It is then automatically low-pass filtered.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void computeJointAccelerationFromFiniteDifferenceWithJointsToIgnore(DoubleYoVariable alphaFilter, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable intermediateJointVelocity = outputJointVelocities.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointAccelerations.get(oneDoFJoint);
         String prefix = JOINT_ACCELERATION.getProcessorNamePrefix(FINITE_DIFFERENCE);
         String suffix = JOINT_ACCELERATION.getProcessorNameSuffix(jointName, processors.size());
         FilteredVelocityYoVariable jointAcceleration = new FilteredVelocityYoVariable(prefix + suffix, "", alphaFilter, intermediateJointVelocity, updateDT, registry);
         processors.add(jointAcceleration);

         if (!forVizOnly)
            outputJointAccelerations.put(oneDoFJoint, jointAcceleration);
      }
   }

   /**
    * Add a low-pass filter stage on the orientations provided by the IMU sensors.
    * This is cumulative, by calling this method twice for instance, you will obtain a two pole low-pass filter.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the sensors to ignore.
    */
   private Map<String, Integer> addIMUOrientationAlphaFilterWithSensorsToIgnore(DoubleYoVariable alphaFilter, boolean forVizOnly, List<String> sensorsToIgnore)
   {
      Map<String, Integer> processorIDs = new HashMap<>();

      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();

         if (sensorsToIgnore.contains(imuName))
            continue;

         YoFrameQuaternion intermediateOrientation = intermediateOrientations.get(imuDefinition);
         List<ProcessingYoVariable> processors = processedOrientations.get(imuDefinition);
         String prefix = IMU_ORIENTATION.getProcessorNamePrefix(ALPHA_FILTER);
         int newProcessorID = processors.size();
         processorIDs.put(imuName, newProcessorID);
         String suffix = IMU_ORIENTATION.getProcessorNameSuffix(imuName, newProcessorID);
         AlphaFilteredYoFrameQuaternion filteredOrientation = new AlphaFilteredYoFrameQuaternion(prefix, suffix, intermediateOrientation, alphaFilter, registry);
         processors.add(filteredOrientation);
         
         if (!forVizOnly)
            intermediateOrientations.put(imuDefinition, filteredOrientation);
      }

      return processorIDs;
   }

   /**
    * Apply a backlash compensator (see {@link BacklashProcessingYoVariable}) to each component of the IMU angular velocity.
    * Useful when the robot has backlash in its joints or simply to calm down small shakies when the robot is at rest.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param slopTime every time the velocity changes sign, a slop is engaged during which a confidence factor is ramped up from 0 to 1.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public Map<String, Integer> addIMUAngularVelocityBacklashFilter(DoubleYoVariable slopTime, boolean forVizOnly)
   {
      return addIMUAngularVelocityBacklashFilterWithSensorsToIgnore(slopTime, forVizOnly);
   }

   /**
    * Apply a backlash compensator (see {@link BacklashProcessingYoVariable}) to each component of the IMU angular velocity.
    * Useful when the robot has backlash in its joints or simply to calm down small shakies when the robot is at rest.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param slopTime every time the velocity changes sign, a slop is engaged during which a confidence factor is ramped up from 0 to 1.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param sensorsToBeProcessed list of the names of the sensors that need to be filtered.
    */
   public Map<String, Integer> addIMUAngularVelocityBacklashFilterOnlyForSpecifiedSensors(DoubleYoVariable slopTime, boolean forVizOnly, String... sensorsToBeProcessed)
   {
      return addIMUAngularVelocityBacklashFilterWithSensorsToIgnore(slopTime, forVizOnly, invertSensorSelection(IMU_ANGULAR_VELOCITY, sensorsToBeProcessed));
   }

   /**
    * Apply a backlash compensator (see {@link BacklashProcessingYoVariable}) to each component of the IMU angular velocity.
    * Useful when the robot has backlash in its joints or simply to calm down small shakies when the robot is at rest.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param slopTime every time the velocity changes sign, a slop is engaged during which a confidence factor is ramped up from 0 to 1.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the sensors to ignore.
    */
   public Map<String, Integer> addIMUAngularVelocityBacklashFilterWithSensorsToIgnore(DoubleYoVariable slopTime, boolean forVizOnly, String... sensorsToIgnore)
   {
      Map<String, Integer> processorIDs = new HashMap<>();

      List<String> sensorToIgnoreList = new ArrayList<>();
      if (sensorsToIgnore != null && sensorsToIgnore.length > 0)
         sensorToIgnoreList.addAll(Arrays.asList(sensorsToIgnore));

      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();

         if (sensorToIgnoreList.contains(imuName))
            continue;

         YoFrameVector intermediateAngularVelocity = intermediateAngularVelocities.get(imuDefinition);
         List<ProcessingYoVariable> processors = processedAngularVelocities.get(imuDefinition);
         String prefix = IMU_ANGULAR_VELOCITY.getProcessorNamePrefix(BACKLASH);
         int newProcessorID = processors.size();
         processorIDs.put(imuName, newProcessorID);
         String suffix = IMU_ANGULAR_VELOCITY.getProcessorNameSuffix(imuName, newProcessorID);
         BacklashProcessingYoFrameVector filteredAngularVelocity = BacklashProcessingYoFrameVector.createBacklashProcessingYoFrameVector(prefix, suffix, updateDT, slopTime, registry, intermediateAngularVelocity);
         processors.add(filteredAngularVelocity);
         
         if (!forVizOnly)
            intermediateAngularVelocities.put(imuDefinition, filteredAngularVelocity);
      }

      return processorIDs;
   }

   /**
    * Call this method to setup validity checkers on all the joint sensors.
    * Each validity checker will check if the sensor measurement signal is NaN, infinite, dead, etc.
    * Use only for diagnostics.
    * @return The map with all the validity checkers.
    */
   public Map<OneDoFJoint, OneDoFJointSensorValidityChecker> addJointSensorValidityCheckers(boolean enableLogging, List<String> jointsToIgnore)
   {
      LinkedHashMap<OneDoFJoint, OneDoFJointSensorValidityChecker> validityCheckerMap = new LinkedHashMap<>();

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint jointToCheck = jointSensorDefinitions.get(i);

         if (jointsToIgnore.contains(jointToCheck.getName()))
            continue;

         DoubleYoVariable position = outputJointPositions.get(jointToCheck);
         DoubleYoVariable velocity = outputJointVelocities.get(jointToCheck);
         DoubleYoVariable tau = outputJointTaus.get(jointToCheck);
         OneDoFJointSensorValidityChecker validityChecker = new OneDoFJointSensorValidityChecker(jointToCheck, position, velocity, tau, registry);
         if (enableLogging)
            validityChecker.setupForLogging();
         validityCheckerMap.put(jointToCheck, validityChecker);
         diagnosticModules.add(validityChecker);
      }

      return validityCheckerMap;
   }

   public Map<String, IMUSensorValidityChecker> addIMUSensorValidityCheckers(boolean enableLogging)
   {
      LinkedHashMap<String, IMUSensorValidityChecker> validityCheckerMap = new LinkedHashMap<>();

      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuToCheck = imuSensorDefinitions.get(i);
         YoFrameQuaternion orientation = intermediateOrientations.get(imuToCheck);
         YoFrameVector angularVelocity = intermediateAngularVelocities.get(imuToCheck);
         YoFrameVector linearAcceleration = intermediateLinearAccelerations.get(imuToCheck);
         IMUSensorValidityChecker validityChecker = new IMUSensorValidityChecker(imuToCheck, orientation, angularVelocity, linearAcceleration, registry);
         if (enableLogging)
            validityChecker.setupForLogging();
         validityCheckerMap.put(imuToCheck.getName(), validityChecker);
         diagnosticModules.add(validityChecker);
      }

      return validityCheckerMap;
   }

   public Map<ForceSensorDefinition, WrenchSensorValidityChecker> addWrenchSensorValidityCheckers(boolean enableLogging)
   {
      LinkedHashMap<ForceSensorDefinition, WrenchSensorValidityChecker> validityCheckerMap = new LinkedHashMap<>();

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         ForceSensorDefinition wrenchSensorToCheck = forceSensorDefinitions.get(i);
         YoFrameVector forceMeasurement = intermediateForces.get(wrenchSensorToCheck);
         YoFrameVector torqueMeasurement = intermediateTorques.get(wrenchSensorToCheck);
         WrenchSensorValidityChecker validityChecker = new WrenchSensorValidityChecker(wrenchSensorToCheck, forceMeasurement, torqueMeasurement, registry);
         if (enableLogging)
            validityChecker.setupForLogging();
         validityCheckerMap.put(wrenchSensorToCheck, validityChecker);
         diagnosticModules.add(validityChecker);
      }

      return validityCheckerMap;
   }

   public Map<OneDoFJoint, PositionVelocity1DConsistencyChecker> addJointPositionVelocityConsistencyCheckers(List<String> jointsToIgnore)
   {
      LinkedHashMap<OneDoFJoint, PositionVelocity1DConsistencyChecker> consistencyCheckerMap = new LinkedHashMap<>();

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint jointToCheck = jointSensorDefinitions.get(i);

         if (jointsToIgnore.contains(jointToCheck.getName()))
            continue;

         DoubleYoVariable rawPosition = inputJointPositions.get(jointToCheck);
         DoubleYoVariable rawVelocity = inputJointVelocities.get(jointToCheck);
         DoubleYoVariable processedPosition = outputJointPositions.get(jointToCheck);
         DoubleYoVariable processedVelocity = outputJointVelocities.get(jointToCheck);
         PositionVelocity1DConsistencyChecker consistencyChecker = new PositionVelocity1DConsistencyChecker(jointToCheck.getName(), rawPosition, rawVelocity, processedPosition, processedVelocity, updateDT, registry);
         consistencyCheckerMap.put(jointToCheck, consistencyChecker);
         diagnosticModules.add(consistencyChecker);
      }

      return consistencyCheckerMap;
   }

   public Map<String, OrientationAngularVelocityConsistencyChecker> addIMUOrientationAngularVelocityConsistencyCheckers()
   {
      LinkedHashMap<String, OrientationAngularVelocityConsistencyChecker> consistencyCheckerMap = new LinkedHashMap<>();

      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuToCheck = imuSensorDefinitions.get(i);

         ReferenceFrame referenceFrame = imuToCheck.getRigidBody().getBodyFixedFrame();
         YoFrameQuaternion orientation = intermediateOrientations.get(imuToCheck);
         YoFrameVector angularVelocity = intermediateAngularVelocities.get(imuToCheck);
         OrientationAngularVelocityConsistencyChecker consistencyChecker = new OrientationAngularVelocityConsistencyChecker(imuToCheck.getName(), orientation, angularVelocity, referenceFrame, updateDT, registry);
         consistencyCheckerMap.put(imuToCheck.getName(), consistencyChecker);
         diagnosticModules.add(consistencyChecker);
      }

      return consistencyCheckerMap;
   }

   public Map<OneDoFJoint, OneDoFJointForceTrackingDelayEstimator> addJointForceTrackingDelayEstimators(List<String> jointsToIgnore)
   {
      LinkedHashMap<OneDoFJoint, OneDoFJointForceTrackingDelayEstimator> delayEstimatorMap = new LinkedHashMap<>();

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint jointToCheck = jointSensorDefinitions.get(i);

         if (jointsToIgnore.contains(jointToCheck.getName()))
            continue;

         OneDoFJointForceTrackingDelayEstimator delayEstimator = new OneDoFJointForceTrackingDelayEstimator(jointToCheck, updateDT, registry);
         delayEstimatorMap.put(jointToCheck, delayEstimator);
         diagnosticModules.add(delayEstimator);
      }

      return delayEstimatorMap;
   }

   public Map<OneDoFJoint, OneDoFJointFourierAnalysis> addJointFourierAnalysis(double estimationWindow, List<String> jointsToIgnore)
   {
      LinkedHashMap<OneDoFJoint, OneDoFJointFourierAnalysis> jointFourierAnalysisMap = new LinkedHashMap<>();

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint jointToCheck = jointSensorDefinitions.get(i);

         if (jointsToIgnore.contains(jointToCheck.getName()))
            continue;

         OneDoFJointFourierAnalysis jointFourierAnalysis = new OneDoFJointFourierAnalysis(jointToCheck, estimationWindow, updateDT, registry);
         jointFourierAnalysisMap.put(jointToCheck, jointFourierAnalysis);
         diagnosticModules.add(jointFourierAnalysis);
      }

      return jointFourierAnalysisMap;
   }

   /**
    * Create an alpha filter given a name and a break frequency (in Hertz) that will be registered in the {@code SensorProcessing}'s {@code YoVariableRegistry}.
    * @param name name of the variable.
    * @param breakFrequency break frequency in Hertz
    * @return a {@code DoubleYoVariable} to be used when adding a low-pass filter stage using the methods in this class such as {@link SensorProcessing#addJointVelocityAlphaFilter(DoubleYoVariable, boolean)}.
    */
   public DoubleYoVariable createAlphaFilter(String name, double breakFrequency)
   {
      DoubleYoVariable alphaFilter = new DoubleYoVariable(name, registry);
      alphaFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency, updateDT));
      return alphaFilter;
   }

   /**
    * Helper to create easily a {@code Map<OneDoFJoint, DoubleYoVariable>} referring to the stiffness for each joint.
    * @param nameSuffix suffix to be used in the variables' name.
    * @param defaultStiffness default value of stiffness to use when not referred in the jointSpecificStiffness.
    * @param jointSpecificStiffness {@code Map<String, Double>} referring the specific stiffness value to be used for each joint. Does not need to be exhaustive, can also be empty or null in which the defaultStiffness is used for every joint.
    * @return {@code Map<OneDoFJoint, DoubleYoVariable>} to be used when calling {@link SensorProcessing#addJointPositionElasticyCompensator(Map, boolean)}.
    */
   public Map<OneDoFJoint, DoubleYoVariable> createStiffness(String nameSuffix, double defaultStiffness, Map<String, Double> jointSpecificStiffness)
   {
      return createStiffnessWithJointsToIgnore(nameSuffix, defaultStiffness, jointSpecificStiffness);
   }

   public Map<OneDoFJoint, DoubleYoVariable> createStiffnessWithJointsToIgnore(String nameSuffix, double defaultStiffness, Map<String, Double> jointSpecificStiffness, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      LinkedHashMap<OneDoFJoint, DoubleYoVariable> stiffesses = new LinkedHashMap<>();
      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         
         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleYoVariable stiffness = new DoubleYoVariable(jointName + nameSuffix, registry);

         if (jointSpecificStiffness != null && jointSpecificStiffness.containsKey(jointName))
            stiffness.set(jointSpecificStiffness.get(jointName));
         else
            stiffness.set(defaultStiffness);
         
         stiffesses.put(oneDoFJoint, stiffness);
      }
      
      return stiffesses;
   }

   /**
    * Helper to convert easily a map from joint name to @{@code DoubleYoVariable} ({@code Map<String, DoubleYoVariable>}) to a {@code Map<OneDoFJoint, DoubleYoVariable>}.
    * @param mapToConvert {@code Map<String, DoubleYoVariable>} the map to be converted, not modified.
    * @return {@code Map<OneDoFJoint, DoubleYoVariable>} the converted map.
    */
   public Map<OneDoFJoint, DoubleYoVariable> convertFromJointNameToJointMap(Map<String, DoubleYoVariable> mapToConvert)
   {
      LinkedHashMap<OneDoFJoint, DoubleYoVariable> newMap = new LinkedHashMap<>();
      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         
         DoubleYoVariable doubleYoVariable = mapToConvert.get(jointName);
         if (doubleYoVariable != null)
            newMap.put(oneDoFJoint, doubleYoVariable);
      }
      
      return newMap;
   }

   public DoubleYoVariable createMaxDeflection(String name, double defaultValue)
   {
      DoubleYoVariable maxDeflection = new DoubleYoVariable(name, registry);
      maxDeflection.set(defaultValue);
      return maxDeflection;
   }

   private String[] invertSensorSelection(SensorType sensorType, String... subSelection)
   {
      if (sensorType.isJointSensor())
         return invertSensorSelection(allJointSensorNames, subSelection);
      else if (sensorType.isIMUSensor())
         return invertSensorSelection(allIMUSensorNames, subSelection);
      else if (sensorType.isWrenchSensor())
         return invertSensorSelection(allForceSensorNames, subSelection);
      else
         throw new RuntimeException("Invert selection is not implemented for this type of sensor: sensorType = " + sensorType);
   }

   private String[] invertSensorSelection(List<String> allSensorNames, String... subSelection)
   {
      List<String> invertSelection = new ArrayList<>();
      List<String> originalJointSensorSelectionList = new ArrayList<>();
      if (subSelection != null && subSelection.length > 0)
         originalJointSensorSelectionList.addAll(Arrays.asList(subSelection));

      for (int i = 0; i < allSensorNames.size(); i++)
      {
         String jointName = allSensorNames.get(i);
         if (!originalJointSensorSelectionList.contains(jointName))
            invertSelection.add(jointName);
      }
      return invertSelection.toArray(new String[0]);
   }

   private LinkedHashMap<OneDoFJoint, List<ProcessingYoVariable>> getProcessedJointSignals(SensorType sensorType)
   {
      switch (sensorType)
      {
      case JOINT_POSITION:
         return processedJointPositions;
      case JOINT_VELOCITY:
         return processedJointVelocities;
      case JOINT_ACCELERATION:
         return processedJointAccelerations;
      case JOINT_TAU:
         return processedJointTaus;
      default:
         throw new RuntimeException("Expected a joint sensor.");
      }
   }

   private LinkedHashMap<OneDoFJoint, DoubleYoVariable> getOutputJointSignals(SensorType sensorType)
   {
      switch (sensorType)
      {
      case JOINT_POSITION:
         return outputJointPositions;
      case JOINT_VELOCITY:
         return outputJointVelocities;
      case JOINT_ACCELERATION:
         return outputJointAccelerations;
      case JOINT_TAU:
         return outputJointTaus;
      default:
         throw new RuntimeException("Expected a joint sensor.");
      }
   }

   private LinkedHashMap<ForceSensorDefinition, List<ProcessingYoVariable>> getProcessedForceSensorSignals(SensorType sensorType)
   {
      switch (sensorType)
      {
      case FORCE_SENSOR:
         return processedForces;
      case TORQUE_SENSOR:
         return processedTorques;
      default:
         throw new RuntimeException("Expected a forcce/torque sensor.");
      }
   }

   private LinkedHashMap<ForceSensorDefinition, YoFrameVector> getIntermediateForceSensorSignals(SensorType sensorType)
   {
      switch (sensorType)
      {
      case FORCE_SENSOR:
         return intermediateForces;
      case TORQUE_SENSOR:
         return intermediateTorques;
      default:
         throw new RuntimeException("Expected a forcce/torque sensor.");
      }
   }

   private LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> getProcessedIMUVectorTypeSignals(SensorType sensorType)
   {
      switch (sensorType)
      {
      case IMU_ANGULAR_VELOCITY:
         return processedAngularVelocities;
      case IMU_LINEAR_ACCELERATION:
         return processedLinearAccelerations;
      default:
         throw new RuntimeException("Expected either: " + SensorType.IMU_ANGULAR_VELOCITY + " or " + SensorType.IMU_LINEAR_ACCELERATION);
      }
   }

   private LinkedHashMap<IMUDefinition, YoFrameVector> getIntermediateIMUVectorTypeSignals(SensorType sensorType)
   {
      switch (sensorType)
      {
      case IMU_ANGULAR_VELOCITY:
         return intermediateAngularVelocities;
      case IMU_LINEAR_ACCELERATION:
         return intermediateLinearAccelerations;
      default:
         throw new RuntimeException("Expected either: " + SensorType.IMU_ANGULAR_VELOCITY + " or " + SensorType.IMU_LINEAR_ACCELERATION);
      }
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
   
   @Override
   public long getSensorHeadPPSTimestamp()
   {
      return sensorHeadPPSTimetamp.getLongValue();
   }

   public void setJointEnabled(OneDoFJoint oneDoFJoint, boolean enabled)
   {
      jointEnabledIndicators.get(oneDoFJoint).set(enabled);
   }

   public void setJointPositionSensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointPositions.get(oneDoFJoint).set(value);
   }

   public void setJointVelocitySensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointVelocities.get(oneDoFJoint).set(value);
   }
   
   public void setJointAccelerationSensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointAccelerations.get(oneDoFJoint).set(value);
   }

   public void setJointTauSensorValue(OneDoFJoint oneDoFJoint, double value)
   {
      inputJointTaus.get(oneDoFJoint).set(value);
   }

   public void setOrientationSensorValue(IMUDefinition imuDefinition, Quaternion value)
   {
      inputOrientations.get(imuDefinition).set(value);
   }

   public void setOrientationSensorValue(IMUDefinition imuDefinition, RotationMatrix value)
   {
      inputOrientations.get(imuDefinition).set(value);
   }

   public void setAngularVelocitySensorValue(IMUDefinition imuDefinition, Vector3D value)
   {
      inputAngularVelocities.get(imuDefinition).set(value);
   }

   public void setLinearAccelerationSensorValue(IMUDefinition imuDefinition, Vector3D value)
   {
      inputLinearAccelerations.get(imuDefinition).set(value);
   }

   public void setForceSensorValue(ForceSensorDefinition forceSensorDefinition, DenseMatrix64F value)
   {
      if (value.getNumRows() != Wrench.SIZE || value.getNumCols() != 1)
         throw new RuntimeException("Unexpected size");

      inputForceSensors.setForceSensorValue(forceSensorDefinition, value);
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
   
   @Override
   public double getJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return outputJointAccelerations.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return outputJointTaus.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public List<? extends IMUSensorReadOnly> getIMUProcessedOutputs()
   {
      return outputIMUs;
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorProcessedOutputs()
   {
      return outputForceSensors;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public double getJointPositionRawOutput(OneDoFJoint oneDoFJoint)
   {
      return inputJointPositions.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointVelocityRawOutput(OneDoFJoint oneDoFJoint)
   {
      return inputJointVelocities.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointAccelerationRawOutput(OneDoFJoint oneDoFJoint)
   {
      return inputJointAccelerations.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointTauRawOutput(OneDoFJoint oneDoFJoint)
   {
      return inputJointTaus.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public boolean isJointEnabled(OneDoFJoint oneDoFJoint)
   {
      return jointEnabledIndicators.get(oneDoFJoint).getBooleanValue();
   }

   @Override
   public List<? extends IMUSensorReadOnly> getIMURawOutputs()
   {
      return inputIMUs;
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorRawOutputs()
   {
      return inputForceSensors;
   }

   @Override
   public AuxiliaryRobotData getAuxiliaryRobotData()
   {
      return this.auxiliaryRobotData;
   }

   public void setAuxiliaryRobotData(AuxiliaryRobotData auxiliaryRobotData)
   {
      this.auxiliaryRobotData = auxiliaryRobotData;
   }
}
