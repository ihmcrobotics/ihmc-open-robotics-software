package us.ihmc.sensorProcessing.sensorProcessors;

import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.FORCE_SENSOR;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ANGULAR_VELOCITY;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_LINEAR_ACCELERATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.IMU_ORIENTATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_ACCELERATION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_POSITION;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_TAU;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.JOINT_VELOCITY;
import static us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType.TORQUE_SENSOR;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;

import org.ejml.data.DenseMatrix64F;

import controller_msgs.msg.dds.AtlasAuxiliaryRobotData;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.dataStructures.PolynomialReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameQuaternion;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.BacklashProcessingYoFrameVector;
import us.ihmc.robotics.math.filters.BacklashProcessingYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;
import us.ihmc.robotics.math.filters.RevisedBacklashCompensatingVelocityYoVariable;
import us.ihmc.robotics.math.filters.YoIMUMahonyFilter;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticUpdatable;
import us.ihmc.sensorProcessing.diagnostic.IMUSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointForceTrackingDelayEstimator;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointFourierAnalysis;
import us.ihmc.sensorProcessing.diagnostic.OneDoFJointSensorValidityChecker;
import us.ihmc.sensorProcessing.diagnostic.OrientationAngularVelocityConsistencyChecker;
import us.ihmc.sensorProcessing.diagnostic.PositionVelocity1DConsistencyChecker;
import us.ihmc.sensorProcessing.diagnostic.WrenchSensorValidityChecker;
import us.ihmc.sensorProcessing.imu.IMUSensor;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoLong;

public class SensorProcessing implements SensorOutputMapReadOnly, SensorRawOutputMapReadOnly
{
   private static final String RAW = "raw";
   private static final String BACKLASH = "bl";
   private static final String POLYNOMIAL = "poly";
   private static final String CONSTANT = "cst";
   private static final String AFFINE = "aff";
   private static final String MAHONY = "mah";
   private static final String COUPLING = "cpl";
   private static final String SWITCH = "switch";
   private static final String ALPHA_FILTER = "filt";
   private static final String FINITE_DIFFERENCE = "fd";
   private static final String ELASTICITY_COMPENSATOR = "stiff";

   private static final ProcessingYoVariable EMPTY_PROCESSOR = () -> {};

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

   private final YoLong timestamp = new YoLong("timestamp", registry);
   private final YoLong controllerTimestamp = new YoLong("controllerTimestamp", registry);
   private final YoLong sensorHeadPPSTimetamp = new YoLong("sensorHeadPPSTimetamp", registry);

   private final LinkedHashMap<OneDoFJointBasics, YoDouble> inputJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJointBasics, YoDouble> inputJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJointBasics, YoDouble> inputJointAccelerations = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJointBasics, YoDouble> inputJointTaus = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, YoFrameQuaternion> inputOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector3D> inputAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector3D> inputLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector3D> inputForces = new LinkedHashMap<>();
   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector3D> inputTorques = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, YoFrameQuaternion> intermediateOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector3D> intermediateAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, YoFrameVector3D> intermediateLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector3D> intermediateForces = new LinkedHashMap<>();
   private final LinkedHashMap<ForceSensorDefinition, YoFrameVector3D> intermediateTorques = new LinkedHashMap<>();

   private final LinkedHashMap<OneDoFJointBasics, List<ProcessingYoVariable>> processedJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJointBasics, List<ProcessingYoVariable>> processedJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJointBasics, List<ProcessingYoVariable>> processedJointAccelerations = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJointBasics, List<ProcessingYoVariable>> processedJointTaus = new LinkedHashMap<>();

   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedOrientations = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedAngularVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedLinearAccelerations = new LinkedHashMap<>();

   private final LinkedHashMap<ForceSensorDefinition, List<ProcessingYoVariable>> processedForces = new LinkedHashMap<>();
   private final LinkedHashMap<ForceSensorDefinition, List<ProcessingYoVariable>> processedTorques = new LinkedHashMap<>();

   private final LinkedHashMap<OneDoFJointBasics, YoDouble> outputJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJointBasics, YoDouble> outputJointVelocities = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJointBasics, YoDouble> outputJointAccelerations = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJointBasics, YoDouble> outputJointTaus = new LinkedHashMap<>();

   private final ArrayList<DiagnosticUpdatable> diagnosticModules = new ArrayList<>();

   private final ArrayList<IMUSensor> inputIMUs = new ArrayList<IMUSensor>();
   private final ArrayList<IMUSensor> outputIMUs = new ArrayList<IMUSensor>();

   private final ForceSensorDataHolder inputForceSensors;
   private final ForceSensorDataHolder outputForceSensors;

   private final List<OneDoFJointBasics> jointSensorDefinitions;
   private final List<IMUDefinition> imuSensorDefinitions;
   private final List<ForceSensorDefinition> forceSensorDefinitions;

   private final List<String> allJointSensorNames = new ArrayList<>();
   private final List<String> allIMUSensorNames = new ArrayList<>();
   private final List<String> allForceSensorNames = new ArrayList<>();

   private final LinkedHashMap<OneDoFJointBasics, YoBoolean> jointEnabledIndicators = new LinkedHashMap<>();

   private final double updateDT;

   private final RotationMatrix tempOrientation = new RotationMatrix();

   private final FrameVector3D tempForce = new FrameVector3D();
   private final FrameVector3D tempTorque = new FrameVector3D();
   private final Wrench tempWrench = new Wrench();

   private AtlasAuxiliaryRobotData auxiliaryRobotData;
   

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
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         allJointSensorNames.add(jointName);

         prefix = JOINT_POSITION.getProcessorNamePrefix(RAW);
         suffix = JOINT_POSITION.getProcessorNameSuffix(jointName, -1);
         YoDouble rawJointPosition = new YoDouble(prefix + suffix, registry);
         inputJointPositions.put(oneDoFJoint, rawJointPosition);
         outputJointPositions.put(oneDoFJoint, rawJointPosition);
         processedJointPositions.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         prefix = JOINT_VELOCITY.getProcessorNamePrefix(RAW);
         suffix = JOINT_VELOCITY.getProcessorNameSuffix(jointName, -1);
         YoDouble rawJointVelocity = new YoDouble(prefix + suffix, registry);
         inputJointVelocities.put(oneDoFJoint, rawJointVelocity);
         outputJointVelocities.put(oneDoFJoint, rawJointVelocity);
         processedJointVelocities.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         prefix = JOINT_ACCELERATION.getProcessorNamePrefix(RAW);
         suffix = JOINT_ACCELERATION.getProcessorNameSuffix(jointName, -1);
         YoDouble rawJointAcceleration = new YoDouble(prefix + suffix, registry);
         inputJointAccelerations.put(oneDoFJoint, rawJointAcceleration);
         outputJointAccelerations.put(oneDoFJoint, rawJointAcceleration);
         processedJointAccelerations.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         prefix = JOINT_TAU.getProcessorNamePrefix(RAW);
         suffix = JOINT_TAU.getProcessorNameSuffix(jointName, -1);
         YoDouble rawJointTau = new YoDouble(prefix + suffix, registry);
         inputJointTaus.put(oneDoFJoint, rawJointTau);
         outputJointTaus.put(oneDoFJoint, rawJointTau);
         processedJointTaus.put(oneDoFJoint, new ArrayList<ProcessingYoVariable>());

         YoBoolean jointEnabledIndicator = new YoBoolean("joint_enabled_" + jointName, registry);
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
         YoFrameVector3D rawAngularVelocity = new YoFrameVector3D(prefix, suffix, sensorFrame, registry);
         inputAngularVelocities.put(imuDefinition, rawAngularVelocity);
         intermediateAngularVelocities.put(imuDefinition, rawAngularVelocity);
         processedAngularVelocities.put(imuDefinition, new ArrayList<ProcessingYoVariable>());

         prefix = IMU_LINEAR_ACCELERATION.getProcessorNamePrefix(RAW);
         suffix = IMU_LINEAR_ACCELERATION.getProcessorNameSuffix(imuName, -1);
         YoFrameVector3D rawLinearAcceleration = new YoFrameVector3D(prefix, suffix, sensorFrame, registry);
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
         YoFrameVector3D rawForce = new YoFrameVector3D(prefix, suffix, sensorFrame, registry);
         inputForces.put(forceSensorDefinition, rawForce);
         intermediateForces.put(forceSensorDefinition, rawForce);
         processedForces.put(forceSensorDefinition, new ArrayList<ProcessingYoVariable>());

         prefix = TORQUE_SENSOR.getProcessorNamePrefix(RAW);
         suffix = TORQUE_SENSOR.getProcessorNameSuffix(sensorName, -1);
         YoFrameVector3D rawTorque = new YoFrameVector3D(prefix, suffix, sensorFrame, registry);
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

   /**
    * Triggers a control tick for this sensor processing.
    * <p>
    * During a control tick, every sensor processor is updated once and the timestamps are updated.
    * </p>
    * 
    * @param timestamp machine timestamp in nanoseconds.
    * @param controllerTimestamp timestamp in nanoseconds representing the controller up-time.
    * @param sensorHeadPPSTimestamp TODO define me.
    */
   public void startComputation(long timestamp, long controllerTimestamp, long sensorHeadPPSTimestamp)
   {
      this.timestamp.set(timestamp);
      this.controllerTimestamp.set(controllerTimestamp);
      this.sensorHeadPPSTimetamp.set(sensorHeadPPSTimestamp);

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);

         updateProcessors(processedJointPositions.get(oneDoFJoint));
         updateProcessors(processedJointVelocities.get(oneDoFJoint));
         updateProcessors(processedJointAccelerations.get(oneDoFJoint));
         updateProcessors(processedJointTaus.get(oneDoFJoint));
      }
      
      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);

         IMUSensor inputIMU = inputIMUs.get(i);
         tempOrientation.set(inputOrientations.get(imuDefinition));
         inputIMU.setOrientationMeasurement(tempOrientation);
         inputIMU.setAngularVelocityMeasurement(inputAngularVelocities.get(imuDefinition));
         inputIMU.setLinearAccelerationMeasurement(inputLinearAccelerations.get(imuDefinition));

         updateProcessors(processedOrientations.get(imuDefinition));
         updateProcessors(processedAngularVelocities.get(imuDefinition));
         updateProcessors(processedLinearAccelerations.get(imuDefinition));
         
         IMUSensor outputIMU = outputIMUs.get(i);
         tempOrientation.set(intermediateOrientations.get(imuDefinition));
         outputIMU.setOrientationMeasurement(tempOrientation);
         outputIMU.setAngularVelocityMeasurement(intermediateAngularVelocities.get(imuDefinition));
         outputIMU.setLinearAccelerationMeasurement(intermediateLinearAccelerations.get(imuDefinition));
      }

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);

         inputForceSensors.getForceSensorValue(forceSensorDefinition, tempWrench);
         tempForce.setIncludingFrame(tempWrench.getLinearPart()); 
         tempTorque.setIncludingFrame(tempWrench.getAngularPart()); 
         inputForces.get(forceSensorDefinition).set(tempForce);
         inputTorques.get(forceSensorDefinition).set(tempTorque);
         
         updateProcessors(processedForces.get(forceSensorDefinition));
         updateProcessors(processedTorques.get(forceSensorDefinition));

         tempForce.setIncludingFrame(intermediateForces.get(forceSensorDefinition));
         tempTorque.setIncludingFrame(intermediateTorques.get(forceSensorDefinition));
         tempWrench.set(tempTorque, tempForce);
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
   public Map<String, Integer> addSensorAlphaFilter(DoubleProvider alphaFilter, boolean forVizOnly, SensorType sensorType)
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
   public Map<String, Integer> addSensorAlphaFilterOnlyForSpecifiedSensors(DoubleProvider alphaFilter, boolean forVizOnly, SensorType sensorType, String... sensorsToBeProcessed)
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
   public Map<String, Integer> addSensorAlphaFilterWithSensorsToIgnore(DoubleProvider alphaFilter, boolean forVizOnly, SensorType sensorType, String... sensorsToIgnore)
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

   private Map<String, Integer> addIMUVectorTypeDataAlphaFilter(DoubleProvider alphaFilter, boolean forVizOnly, SensorType sensorType, List<String> sensorsToIgnore)
   {
      Map<String, Integer> processorsIDs = new HashMap<>();

      LinkedHashMap<IMUDefinition, YoFrameVector3D> intermediateIMUVectorTypeSignals = getIntermediateIMUVectorTypeSignals(sensorType);
      LinkedHashMap<IMUDefinition, List<ProcessingYoVariable>> processedIMUVectorTypeSignals = getProcessedIMUVectorTypeSignals(sensorType);

      for (int i = 0; i < imuSensorDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuSensorDefinitions.get(i);
         String imuName = imuDefinition.getName();

         if (sensorsToIgnore.contains(imuName))
            continue;

         YoFrameVector3D intermediateSignal = intermediateIMUVectorTypeSignals.get(imuDefinition);
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

   private Map<String, Integer> addForceSensorAlphaFilterWithSensorsToIgnore(DoubleProvider alphaFilter, boolean forVizOnly, SensorType sensorType, List<String> sensorsToIgnore)
   {
      Map<String, Integer> processorsIDs = new HashMap<>();

      LinkedHashMap<ForceSensorDefinition, YoFrameVector3D> intermediateForceSensorSignals = getIntermediateForceSensorSignals(sensorType);
      LinkedHashMap<ForceSensorDefinition, List<ProcessingYoVariable>> processedForceSensorSignals = getProcessedForceSensorSignals(sensorType);

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);
         String sensorName = forceSensorDefinition.getSensorName();

         if (sensorsToIgnore.contains(sensorName))
            continue;

         YoFrameVector3D intermediateSignal = intermediateForceSensorSignals.get(forceSensorDefinition);
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

   private Map<String, Integer> addJointAlphaFilterWithJointsToIgnore(DoubleProvider alphaFilter, boolean forVizOnly, SensorType sensorType, List<String> jointsToIgnore)
   {
      Map<String, Integer> processorsIDs = new HashMap<>();

      LinkedHashMap<OneDoFJointBasics, YoDouble> outputJointSignals = getOutputJointSignals(sensorType);
      LinkedHashMap<OneDoFJointBasics, List<ProcessingYoVariable>> processedJointSignals = getProcessedJointSignals(sensorType);

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointsToIgnore.contains(jointName))
            continue;

         YoDouble intermediateJointSignal = outputJointSignals.get(oneDoFJoint);
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

   /**
    * Apply a constant position and velocity override on the joints. This can be useful when dealing with a broken robot with sensors going crazy.
    * @param constantPosition the position value that will be used as the output of the sensor processing.
    * @param constantVelocity the velocity value that will be used as the output of the sensor processing.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void addJointSensorsOverride(double constantPosition, double constantVelocity, boolean forVizOnly)
   {
      addJointSensorsOverrideWithJointsToIgnore(constantPosition, constantVelocity, forVizOnly);
   }

   /**
    * Apply a constant position and velocity override on the joints. This can be useful when dealing with a broken robot with sensors going crazy.
    * @param constantPosition the position value that will be used as the output of the sensor processing.
    * @param constantVelocity the velocity value that will be used as the output of the sensor processing.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToBeProcessed list of the names of the joints that need to be filtered.
    */
   public void addJointSensorsOverrideOnlyForSpecifiedJoints(double constantPosition, double constantVelocity, boolean forVizOnly, String... jointsToBeProcessed)
   {
      addJointSensorsOverrideWithJointsToIgnore(constantPosition, constantVelocity, forVizOnly, invertSensorSelection(allJointSensorNames, jointsToBeProcessed));
   }

   /**
    * Apply a constant position and velocity override on the joints. This can be useful when dealing with a broken robot with sensors going crazy.
    * @param constantPosition the position value that will be used as the output of the sensor processing.
    * @param constantVelocity the velocity value that will be used as the output of the sensor processing.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void addJointSensorsOverrideWithJointsToIgnore(double constantPosition, double constantVelocity, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         List<ProcessingYoVariable> positionProcessors = processedJointPositions.get(oneDoFJoint);
         List<ProcessingYoVariable> velocityProcessors = processedJointVelocities.get(oneDoFJoint);

         String positionPrefix = JOINT_POSITION.getProcessorNamePrefix(CONSTANT);
         String positionSuffix = JOINT_POSITION.getProcessorNameSuffix(jointName, positionProcessors.size());
         String velocityPrefix = JOINT_VELOCITY.getProcessorNamePrefix(CONSTANT);
         String velocitySuffix = JOINT_VELOCITY.getProcessorNameSuffix(jointName, velocityProcessors.size());

         YoDouble yoConstantPosition = new YoDouble(positionPrefix + positionSuffix, registry);
         YoDouble yoConstantVelocity = new YoDouble(velocityPrefix + velocitySuffix, registry);

         yoConstantPosition.set(constantPosition);
         yoConstantVelocity.set(constantVelocity);

         positionProcessors.add(EMPTY_PROCESSOR);
         velocityProcessors.add(EMPTY_PROCESSOR);

         if (!forVizOnly)
         {
            outputJointPositions.put(oneDoFJoint, yoConstantPosition);
            outputJointVelocities.put(oneDoFJoint, yoConstantVelocity);
         }
      }
   }

   /**
    * Apply an affine transform to the joint position:
    * <p>
    * <code> q_filt = scale * q_in + bias </code>
    * </p>
    * <p>
    * Useful when the sensor measurement does not directly reflect the joint position, i.e. separated with a constant gear ratio plus an offset.
    * Implemented as a cumulative processor but should probably be called only once.
    * </p>
    * @param scale the scale factor to apply to the joint position measurement. Can be {@code null}.
    * @param bias the offset to add to the joint position measurement. Can be {@code null}.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @throws RuntimeException if both {@code scale} and {@code bias} are {@code null}.
    */
   public void addJointPositionAffineTransform(DoubleProvider scale, DoubleProvider bias, boolean forVizOnly)
   {
      addJointPositionAffineTransformWithJointsToIgnore(scale, bias, forVizOnly);
   }

   /**
    * Apply an affine transform to the joint position:
    * <p>
    * <code> q_filt = scale * q_in + bias </code>
    * </p>
    * <p>
    * Useful when the sensor measurement does not directly reflect the joint position, i.e. separated with a constant gear ratio plus an offset.
    * Implemented as a cumulative processor but should probably be called only once.
    * </p>
    * @param scale the scale factor to apply to the joint position measurement. Can be {@code null}.
    * @param bias the offset to add to the joint position measurement. Can be {@code null}.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToBeProcessed list of the names of the joints that need to be filtered.
    * @throws RuntimeException if both {@code scale} and {@code bias} are {@code null}.
    */
   public void addJointPositionAffineTransformOnlyForSpecifiedJoints(DoubleProvider scale, DoubleProvider bias, boolean forVizOnly, String... jointsToBeProcessed)
   {
      addJointPositionAffineTransformWithJointsToIgnore(scale, bias, forVizOnly, invertSensorSelection(allJointSensorNames, jointsToBeProcessed));
   }

   /**
    * Apply an affine transform to the joint position:
    * <p>
    * <code> q_filt = scale * q_in + bias </code>
    * </p>
    * <p>
    * Useful when the sensor measurement does not directly reflect the joint position, i.e. separated with a constant gear ratio plus an offset.
    * Implemented as a cumulative processor but should probably be called only once.
    * </p>
    * @param scale the scale factor to apply to the joint position measurement. Can be {@code null}.
    * @param bias the offset to add to the joint position measurement. Can be {@code null}.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    * @throws RuntimeException if both {@code scale} and {@code bias} are {@code null}.
    */
   public void addJointPositionAffineTransformWithJointsToIgnore(DoubleProvider scale, DoubleProvider bias, boolean forVizOnly, String... jointsToIgnore)
   {
      if (scale == null && bias == null)
         throw new RuntimeException("Cannot create processor, scale and bias are both null.");

      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         YoDouble intermediateJointPosition = outputJointPositions.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointPositions.get(oneDoFJoint);
         String prefix = JOINT_POSITION.getProcessorNamePrefix(AFFINE);
         String suffix = JOINT_POSITION.getProcessorNameSuffix(jointName, processors.size());
         YoDouble filteredJointPosition = new YoDouble(prefix + suffix, registry);
         ProcessingYoVariable processor;
         if (scale == null)
            processor = () -> filteredJointPosition.set(intermediateJointPosition.getDoubleValue() + bias.getValue());
         else if (bias == null)
            processor = () -> filteredJointPosition.set(scale.getValue() * intermediateJointPosition.getDoubleValue());
         else
            processor = () -> filteredJointPosition.set(scale.getValue() * intermediateJointPosition.getDoubleValue() + bias.getValue());
         processors.add(processor);

         if (!forVizOnly)
            outputJointPositions.put(oneDoFJoint, filteredJointPosition);
      }
   }

   /**
    * Apply a polynomial function to (see {@link PolynomialProcessorYoVariable}) to the joint position.
    * Useful when the joint encoders are off and the error can be approximated by a polynomial.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param polynomialToApply the polynomial to apply on the joint position signals.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    */
   public void addJointPositionPolynomialProcessor(PolynomialReadOnly polynomialToApply, boolean forVizOnly)
   {
      addJointPositionPolynomialProcessorWithJointsToIgnore(polynomialToApply, forVizOnly);
   }

   /**
    * Apply a polynomial function to (see {@link PolynomialProcessorYoVariable}) to the joint position.
    * Useful when the joint encoders are off and the error can be approximated by a polynomial.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param polynomialToApply the polynomial to apply on the joint position signals.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToBeProcessed list of the names of the joints that need to be filtered.
    */
   public void addJointPositionPolynomialProcessorOnlyForSpecifiedJoints(PolynomialReadOnly polynomialToApply, boolean forVizOnly, String... jointsToBeProcessed)
   {
      addJointPositionPolynomialProcessorWithJointsToIgnore(polynomialToApply, forVizOnly, invertSensorSelection(allJointSensorNames, jointsToBeProcessed));
   }

   /**
    * Apply a polynomial function to (see {@link PolynomialProcessorYoVariable}) to the joint position.
    * Useful when the joint encoders are off and the error can be approximated by a polynomial.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param polynomialToApply the polynomial to apply on the joint position signals.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void addJointPositionPolynomialProcessorWithJointsToIgnore(PolynomialReadOnly polynomialToApply, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         YoDouble intermediateJointPosition = outputJointPositions.get(oneDoFJoint);
         List<ProcessingYoVariable> processors = processedJointPositions.get(oneDoFJoint);
         String prefix = JOINT_POSITION.getProcessorNamePrefix(POLYNOMIAL);
         String suffix = JOINT_POSITION.getProcessorNameSuffix(jointName, processors.size());
         PolynomialProcessorYoVariable filteredJointPosition = new PolynomialProcessorYoVariable(prefix + suffix, intermediateJointPosition, polynomialToApply, registry);
         processors.add(filteredJointPosition);

         if (!forVizOnly)
            outputJointPositions.put(oneDoFJoint, filteredJointPosition);
      }
   }

   public void addJointPositionElasticyCompensator(Map<OneDoFJointBasics, ? extends DoubleProvider> stiffnesses, DoubleProvider maximumDeflection, boolean forVizOnly)
   {
      addJointPositionElasticyCompensatorWithJointsToIgnore(stiffnesses, maximumDeflection, null, forVizOnly);
   }

   public void addJointPositionElasticyCompensatorWithJointsToIgnore(Map<OneDoFJointBasics, ? extends DoubleProvider> stiffnesses, DoubleProvider maximumDeflection, boolean forVizOnly, String... jointsToIgnore)
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
   public void addJointPositionElasticyCompensator(Map<OneDoFJointBasics, ? extends DoubleProvider> stiffnesses, DoubleProvider maximumDeflection, Map<String, Integer> torqueProcessorIDs, boolean forVizOnly)
   {
      addJointPositionElasticyCompensatorWithJointsToIgnore(stiffnesses, maximumDeflection, forVizOnly);
   }

   public void addJointPositionElasticyCompensatorWithJointsToIgnore(Map<OneDoFJointBasics, ? extends DoubleProvider> stiffnesses, DoubleProvider maximumDeflection, Map<String, Integer> torqueProcessorIDs, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleProvider stiffness = stiffnesses.get(oneDoFJoint);
         if (stiffness == null)
            continue;
         YoDouble intermediateJointPosition = outputJointPositions.get(oneDoFJoint);

         YoDouble intermediateJointTau;
         if (torqueProcessorIDs == null || !torqueProcessorIDs.containsKey(jointName))
            intermediateJointTau = outputJointTaus.get(oneDoFJoint);
         else
            intermediateJointTau = (YoDouble) processedJointTaus.get(oneDoFJoint).get(torqueProcessorIDs.get(jointName));

         List<ProcessingYoVariable> processors = processedJointPositions.get(oneDoFJoint);
         String prefix = JOINT_POSITION.getProcessorNamePrefix(ELASTICITY_COMPENSATOR);
         String suffix = JOINT_POSITION.getProcessorNameSuffix(jointName, processors.size());
         ElasticityCompensatorYoVariable filteredJointPosition = new ElasticityCompensatorYoVariable(prefix + suffix, stiffness, maximumDeflection, intermediateJointPosition, intermediateJointTau, registry);
         processors.add(filteredJointPosition);
         
         if (!forVizOnly)
            outputJointPositions.put(oneDoFJoint, filteredJointPosition);
      }
   }

   /**
    * Computes the position of a slave joint from a master joint as follows:
    * <p>
    * <code> q_slave = couplingRatio * q_master + couplingBias </code>
    * </p> 
    * 
    * @param nameOfJointMaster the name of the joint from which the position of the slave is to be computed.
    * @param nameOfJointSlave the name of the joint to compute the position of.
    * @param couplingRatio the ratio between the master position and the slave position. Can be {@code null}.
    * @param couplingBias the position offset between the master and the slave. Can be {@code null}.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @return a provider to get the output of the new processor.
    * @throws RuntimeException if both {@code couplingRatio} and {@code couplingBias} are {@code null}.
    */
   public DoubleProvider computeJointPositionUsingCoupling(String nameOfJointMaster, String nameOfJointSlave, DoubleProvider couplingRatio, DoubleProvider couplingBias, boolean forVizOnly)
   {
      OneDoFJointBasics jointMaster = jointSensorDefinitions.stream().filter(joint -> joint.getName().equals(nameOfJointMaster)).findFirst().get();
      return computeJointPositionUsingCoupling(jointMaster::getQ, nameOfJointSlave, couplingRatio, couplingBias, forVizOnly);
   }

   /**
    * Computes the position of a slave joint from a master joint as follows:
    * <p>
    * <code> q_slave = couplingRatio * q_master + couplingBias </code>
    * </p> 
    * 
    * @param jointMasterPositionProvider the provider of the master joint position that is to be used to compute the slave position.
    * @param nameOfJointSlave the name of the joint to compute the position of.
    * @param couplingRatio the ratio between the master position and the slave position. Can be {@code null}.
    * @param couplingBias the position offset between the master and the slave. Can be {@code null}.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @return a provider to get the output of the new processor.
    * @throws RuntimeException if both {@code couplingRatio} and {@code couplingBias} are {@code null}.
    */
   public DoubleProvider computeJointPositionUsingCoupling(DoubleProvider jointMasterPositionProvider, String nameOfJointSlave, DoubleProvider couplingRatio, DoubleProvider couplingBias, boolean forVizOnly)
   {
      if (couplingRatio == null && couplingBias == null)
         throw new RuntimeException("Cannot create joint position coupling without giving either a couplingRatio or couplingBias.");

      OneDoFJointBasics jointSlave = jointSensorDefinitions.stream().filter(joint -> joint.getName().equals(nameOfJointSlave)).findFirst().get();

      List<ProcessingYoVariable> slaveProcessors = processedJointPositions.get(jointSlave);
      String prefix = JOINT_POSITION.getProcessorNamePrefix(COUPLING);
      int newProcessorID = slaveProcessors.size();
      String suffix = JOINT_POSITION.getProcessorNameSuffix(nameOfJointSlave, newProcessorID);
      YoDouble filteredJointSlavePosition = new YoDouble(prefix + suffix, registry);
      ProcessingYoVariable slaveProcessor;
      if (couplingRatio == null)
         slaveProcessor = () -> filteredJointSlavePosition.set(jointMasterPositionProvider.getValue() + couplingBias.getValue());
      else if (couplingBias == null)
         slaveProcessor = () -> filteredJointSlavePosition.set(couplingRatio.getValue() * jointMasterPositionProvider.getValue());
      else
         slaveProcessor = () -> filteredJointSlavePosition.set(couplingRatio.getValue() * jointMasterPositionProvider.getValue() + couplingBias.getValue());
      slaveProcessors.add(slaveProcessor);

      if (!forVizOnly)
         outputJointPositions.put(jointSlave, filteredJointSlavePosition);

      return filteredJointSlavePosition;
   }

   /**
    * Creates a new processor that allows to switch the output of this sensor processing to either
    * use the current output, which is the result of all the active ({@code forVizOnly = false})
    * processors added up to now, or use the given backup input variable.
    *
    * @param jointName the name of the joint for which the current processed position should have a
    *           backup signal.
    * @param backupInput the secondary signal to use.
    * @param backupProcessorTrigger the trigger used to determine when to switch to the secondary
    *           position signal. The {@link Predicate#test(DoubleProvider)} is called with the
    *           default signal as argument and this switch changes to the secondary signal when the
    *           test method returns {@code true}.
    */
   public void addJointPositionSensorSwitch(String jointName, DoubleProvider backupInput, Predicate<DoubleProvider> backupProcessorTrigger, boolean forVizOnly)
   {
      OneDoFJointBasics joint = jointSensorDefinitions.stream().filter(j -> j.getName().equals(jointName)).findFirst().get();
      List<ProcessingYoVariable> jointProcessors = processedJointPositions.get(joint);

      DoubleProvider defaultInput = outputJointPositions.get(joint);

      String prefix = JOINT_POSITION.getProcessorNamePrefix(SWITCH);
      int newProcessorID = jointProcessors.size();
      String suffix = JOINT_POSITION.getProcessorNameSuffix(jointName, newProcessorID);
      YoDouble filteredJointPosition = new YoDouble(prefix + suffix, registry);
      ProcessingYoVariable jointProcessor = () -> filteredJointPosition.set(backupProcessorTrigger.test(defaultInput) ? backupInput.getValue() : defaultInput.getValue());

      jointProcessors.add(jointProcessor);

      if (!forVizOnly)
         outputJointPositions.put(joint, filteredJointPosition);
   }

   public void addJointVelocityElasticyCompensator(Map<OneDoFJointBasics, ? extends DoubleProvider> stiffnesses, DoubleProvider maximumDeflection, boolean forVizOnly)
   {
      addJointVelocityElasticyCompensatorWithJointsToIgnore(stiffnesses, maximumDeflection, null, forVizOnly);
   }

   public void addJointVelocityElasticyCompensatorWithJointsToIgnore(Map<OneDoFJointBasics, ? extends DoubleProvider> stiffnesses, DoubleProvider maximumDeflection, boolean forVizOnly, String... jointsToIgnore)
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
   public void addJointVelocityElasticyCompensator(Map<OneDoFJointBasics, ? extends DoubleProvider> stiffnesses, DoubleProvider maximumDeflection, Map<String, Integer> torqueProcessorIDs, boolean forVizOnly)
   {
      addJointVelocityElasticyCompensatorWithJointsToIgnore(stiffnesses, maximumDeflection, forVizOnly);
   }

   public void addJointVelocityElasticyCompensatorWithJointsToIgnore(Map<OneDoFJointBasics, ? extends DoubleProvider> stiffnesses, DoubleProvider maximumDeflection, Map<String, Integer> torqueProcessorIDs, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleProvider stiffness = stiffnesses.get(oneDoFJoint);
         if (stiffness == null)
            continue;
         YoDouble intermediateJointVelocity = outputJointVelocities.get(oneDoFJoint);

         YoDouble intermediateJointTau;
         if (torqueProcessorIDs == null || !torqueProcessorIDs.containsKey(jointName))
            intermediateJointTau = outputJointTaus.get(oneDoFJoint);
         else
            intermediateJointTau = (YoDouble) processedJointTaus.get(oneDoFJoint).get(torqueProcessorIDs.get(jointName));

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
   public void computeJointVelocityFromFiniteDifference(DoubleProvider alphaFilter, boolean forVizOnly)
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
   public void computeJointVelocityFromFiniteDifferenceOnlyForSpecifiedJoints(DoubleProvider alphaFilter, boolean forVizOnly, String... jointsToBeProcessed)
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
   public void computeJointVelocityFromFiniteDifferenceWithJointsToIgnore(DoubleProvider alphaFilter, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         YoDouble intermediateJointPosition = outputJointPositions.get(oneDoFJoint);
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
   public void computeJointVelocityWithBacklashCompensator(DoubleProvider alphaFilter, DoubleProvider slopTime, boolean forVizOnly)
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
   public void computeJointVelocityWithBacklashCompensatorOnlyForSpecifiedJoints(DoubleProvider alphaFilter, DoubleProvider slopTime, boolean forVizOnly, String... jointsToBeProcessed)
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
   public void computeJointVelocityWithBacklashCompensatorWithJointsToIgnore(DoubleProvider alphaFilter, DoubleProvider slopTime, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         YoDouble intermediateJointPosition = outputJointPositions.get(oneDoFJoint);

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
   public void addJointVelocityBacklashFilter(DoubleProvider slopTime, boolean forVizOnly)
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
   public void addJointVelocityBacklashFilterOnlyForSpecifiedJoints(DoubleProvider slopTime, boolean forVizOnly, String... jointsToBeProcessed)
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
   public void addJointVelocityBacklashFilterWithJointsToIgnore(DoubleProvider slopTime, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         YoDouble intermediateJointVelocity = outputJointVelocities.get(oneDoFJoint);
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
   public void computeJointAccelerationFromFiniteDifference(DoubleProvider alphaFilter, boolean forVizOnly)
   {
      computeJointAccelerationFromFiniteDifferenceWithJointsToIgnore(alphaFilter, forVizOnly);
   }

   /**
    * Compute the joint accelerations (for a specific subset of joints) by calculating finite-difference on joint velocities using {@link FilteredVelocityYoVariable}. It is then automatically low-pass filtered.
    * @param alphaFilter low-pass filter parameter.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param jointsToIgnore list of the names of the joints to ignore.
    */
   public void computeJointAccelerationFromFiniteDifferenceWithJointsToIgnore(DoubleProvider alphaFilter, boolean forVizOnly, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         if (jointToIgnoreList.contains(jointName))
            continue;

         YoDouble intermediateJointVelocity = outputJointVelocities.get(oneDoFJoint);
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
    * @param sensorsToIgnore list of the names of the sensors to ignore.
    */
   private Map<String, Integer> addIMUOrientationAlphaFilterWithSensorsToIgnore(DoubleProvider alphaFilter, boolean forVizOnly, List<String> sensorsToIgnore)
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
    * Add an orientation estimator based on the Mahony filter, see
    * <a href="http://www.olliw.eu/2013/imu-data-fusing/">IMU Data Fusing</a>.
    * <p>
    * This estimator only uses gyro and accelerometer data to estimate the orientation.
    * </p>
    * 
    * @param proportionalGain gain used to correct the orientation resulting from integrating
    *           angular velocity using the accelerometer.
    * @param integralGain gain used to update the angular velocity bias.
    * @param useStateEstimatorData when {@code false}, the filter will use acceleration measurements
    *           from the IMU to correct the orientation. When {@code true}, a state estimator has to
    *           be running and this filter should NOT be used on the principal IMU; it creates a
    *           virtual accelerometer and a virtual magnetometer such that the resulting orientation
    *           is consistent with the robot state estimation. This is a hack, if you need it, we
    *           should spend time on implementing this feature properly.
    * @param forVizOnly if set to true, the result will not be used as the input of the next
    *           processing stage, nor as the output of the sensor processing.
    */
   public Map<String, Integer> addIMUMahonyFusion(double proportionalGain, double integralGain, boolean useStateEstimatorData, boolean forVizOnly)
   {
      return addIMUMahonyFusionWithSensorsToIgnore(proportionalGain, integralGain, useStateEstimatorData, forVizOnly);
   }

   /**
    * Add an orientation estimator based on the Mahony filter, see
    * <a href="http://www.olliw.eu/2013/imu-data-fusing/">IMU Data Fusing</a>.
    * <p>
    * This estimator only uses gyro and accelerometer data to estimate the orientation.
    * </p>
    * 
    * @param proportionalGain gain used to correct the orientation resulting from integrating
    *           angular velocity using the accelerometer.
    * @param integralGain gain used to update the angular velocity bias.
    * @param useStateEstimatorData when {@code false}, the filter will use acceleration measurements
    *           from the IMU to correct the orientation. When {@code true}, a state estimator has to
    *           be running and this filter should NOT be used on the principal IMU; it creates a
    *           virtual accelerometer and a virtual magnetometer such that the resulting orientation
    *           is consistent with the robot state estimation. This is a hack, if you need it, we
    *           should spend time on implementing this feature properly.
    * @param forVizOnly if set to true, the result will not be used as the input of the next
    *           processing stage, nor as the output of the sensor processing.
    * @param sensorsToBeProcessed list of the names of the sensors that need to be filtered.
    */
   public Map<String, Integer> addIMUMahonyFusionOnlyForSpecifiedSensors(double proportionalGain, double integralGain, boolean useStateEstimatorData,
                                                                         boolean forVizOnly, String... sensorsToBeProcessed)
   {
      return addIMUMahonyFusionWithSensorsToIgnore(proportionalGain, integralGain, useStateEstimatorData, forVizOnly,
                                                   invertSensorSelection(IMU_ORIENTATION, sensorsToBeProcessed));
   }

   /**
    * Add an orientation estimator based on the Mahony filter, see
    * <a href="http://www.olliw.eu/2013/imu-data-fusing/">IMU Data Fusing</a>.
    * <p>
    * This estimator only uses gyro and accelerometer data to estimate the orientation.
    * </p>
    * 
    * @param proportionalGain gain used to correct the orientation resulting from integrating
    *           angular velocity using the accelerometer.
    * @param integralGain gain used to update the angular velocity bias.
    * @param useStateEstimatorData when {@code false}, the filter will use acceleration measurements
    *           from the IMU to correct the orientation. When {@code true}, a state estimator has to
    *           be running and this filter should NOT be used on the principal IMU; it creates a
    *           virtual accelerometer and a virtual magnetometer such that the resulting orientation
    *           is consistent with the robot state estimation. This is a hack, if you need it, we
    *           should spend time on implementing this feature properly.
    * @param forVizOnly if set to true, the result will not be used as the input of the next
    *           processing stage, nor as the output of the sensor processing.
    * @param sensorsToIgnore list of the names of the sensors to ignore.
    */
   public Map<String, Integer> addIMUMahonyFusionWithSensorsToIgnore(double proportionalGain, double integralGain, boolean useStateEstimatorData,
                                                                     boolean forVizOnly, String... sensorsToIgnore)
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

         YoFrameVector3D intermediateAngularVelocity = intermediateAngularVelocities.get(imuDefinition);
         YoFrameVector3D intermediateLinearAcceleration = intermediateLinearAccelerations.get(imuDefinition);

         List<ProcessingYoVariable> orientationProcessors = processedOrientations.get(imuDefinition);
         List<ProcessingYoVariable> angularVelocityProcessors = processedAngularVelocities.get(imuDefinition);
         processorIDs.put(imuName, orientationProcessors.size());
         String orientationPrefix = IMU_ORIENTATION.getProcessorNamePrefix(MAHONY);
         String orientationSuffix = IMU_ORIENTATION.getProcessorNameSuffix(imuName, orientationProcessors.size());
         String angularVelocityPrefix = IMU_ANGULAR_VELOCITY.getProcessorNamePrefix(MAHONY);
         String angularVelocitySuffix = IMU_ANGULAR_VELOCITY.getProcessorNameSuffix(imuName, angularVelocityProcessors.size());

         ReferenceFrame sensorFrame = imuDefinition.getIMUFrame();
         YoFrameQuaternion processedOrientation = new YoFrameQuaternion(orientationPrefix, orientationSuffix, sensorFrame, registry);
         YoFrameVector3D processedAngularVelocity = new YoFrameVector3D(angularVelocityPrefix, angularVelocitySuffix, sensorFrame, registry);

         YoIMUMahonyFilter filter = new YoIMUMahonyFilter(imuName, orientationSuffix, updateDT, sensorFrame, processedOrientation, processedAngularVelocity,
                                                          registry);
         filter.setGains(proportionalGain, integralGain);

         if (useStateEstimatorData)
         {
            /*
             * TODO This is a hack. This feature has to be implemented in the state estimator. As
             * the proper implementation would require some refactor, I (Sylvain) decided to hack it
             * until the feature is really needed, in which case we should dedicate time to find a
             * proper implementation.
             */
            orientationProcessors.add(new ProcessingYoVariable()
            {
               private final FrameVector3D virtualAccelerometerMeasurement = new FrameVector3D();
               private final FrameVector3D virtualMagnetometerMeasurement = new FrameVector3D();
               private boolean reinitialize = true;

               @Override
               public void update()
               {
                  virtualAccelerometerMeasurement.setIncludingFrame(worldFrame, YoIMUMahonyFilter.ACCELERATION_REFERENCE);
                  virtualAccelerometerMeasurement.changeFrame(sensorFrame);
                  virtualMagnetometerMeasurement.setIncludingFrame(worldFrame, YoIMUMahonyFilter.NORTH_REFERENCE);
                  virtualMagnetometerMeasurement.changeFrame(sensorFrame);

                  filter.update(intermediateAngularVelocity, virtualAccelerometerMeasurement, virtualMagnetometerMeasurement);

                  if (reinitialize)
                  { // For the first tick, the frame will not be initialized properly, the filter has to be reinitilized on the second tick when all the frames are in place.
                     filter.setHasBeenInitialized(false);
                     reinitialize = false;
                  }
               }
            });
         }
         else
         {
            filter.setInputs(intermediateAngularVelocity, intermediateLinearAcceleration);
            orientationProcessors.add(filter);
         }

         angularVelocityProcessors.add(EMPTY_PROCESSOR);

         if (!forVizOnly)
         {
            intermediateOrientations.put(imuDefinition, processedOrientation);
            intermediateAngularVelocities.put(imuDefinition, processedAngularVelocity);
         }
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
   public Map<String, Integer> addIMUAngularVelocityBacklashFilter(DoubleProvider slopTime, boolean forVizOnly)
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
   public Map<String, Integer> addIMUAngularVelocityBacklashFilterOnlyForSpecifiedSensors(DoubleProvider slopTime, boolean forVizOnly, String... sensorsToBeProcessed)
   {
      return addIMUAngularVelocityBacklashFilterWithSensorsToIgnore(slopTime, forVizOnly, invertSensorSelection(IMU_ANGULAR_VELOCITY, sensorsToBeProcessed));
   }

   /**
    * Apply a backlash compensator (see {@link BacklashProcessingYoVariable}) to each component of the IMU angular velocity.
    * Useful when the robot has backlash in its joints or simply to calm down small shakies when the robot is at rest.
    * Implemented as a cumulative processor but should probably be called only once.
    * @param slopTime every time the velocity changes sign, a slop is engaged during which a confidence factor is ramped up from 0 to 1.
    * @param forVizOnly if set to true, the result will not be used as the input of the next processing stage, nor as the output of the sensor processing.
    * @param sensorsToIgnore list of the names of the sensors to ignore.
    */
   public Map<String, Integer> addIMUAngularVelocityBacklashFilterWithSensorsToIgnore(DoubleProvider slopTime, boolean forVizOnly, String... sensorsToIgnore)
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

         YoFrameVector3D intermediateAngularVelocity = intermediateAngularVelocities.get(imuDefinition);
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
   public Map<OneDoFJointBasics, OneDoFJointSensorValidityChecker> addJointSensorValidityCheckers(boolean enableLogging, JointDesiredOutputListReadOnly outputDataHolder, List<String> jointsToIgnore)
   {
      LinkedHashMap<OneDoFJointBasics, OneDoFJointSensorValidityChecker> validityCheckerMap = new LinkedHashMap<>();

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics jointToCheck = jointSensorDefinitions.get(i);

         if (jointsToIgnore.contains(jointToCheck.getName()))
            continue;

         YoDouble position = outputJointPositions.get(jointToCheck);
         YoDouble velocity = outputJointVelocities.get(jointToCheck);
         YoDouble tau = outputJointTaus.get(jointToCheck);
         OneDoFJointSensorValidityChecker validityChecker = new OneDoFJointSensorValidityChecker(jointToCheck, outputDataHolder.getJointDesiredOutput(jointToCheck), position, velocity, tau, registry);
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
         YoFrameVector3D angularVelocity = intermediateAngularVelocities.get(imuToCheck);
         YoFrameVector3D linearAcceleration = intermediateLinearAccelerations.get(imuToCheck);
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
         YoFrameVector3D forceMeasurement = intermediateForces.get(wrenchSensorToCheck);
         YoFrameVector3D torqueMeasurement = intermediateTorques.get(wrenchSensorToCheck);
         WrenchSensorValidityChecker validityChecker = new WrenchSensorValidityChecker(wrenchSensorToCheck, forceMeasurement, torqueMeasurement, registry);
         if (enableLogging)
            validityChecker.setupForLogging();
         validityCheckerMap.put(wrenchSensorToCheck, validityChecker);
         diagnosticModules.add(validityChecker);
      }

      return validityCheckerMap;
   }

   public Map<OneDoFJointBasics, PositionVelocity1DConsistencyChecker> addJointPositionVelocityConsistencyCheckers(List<String> jointsToIgnore)
   {
      LinkedHashMap<OneDoFJointBasics, PositionVelocity1DConsistencyChecker> consistencyCheckerMap = new LinkedHashMap<>();

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics jointToCheck = jointSensorDefinitions.get(i);

         if (jointsToIgnore.contains(jointToCheck.getName()))
            continue;

         YoDouble rawPosition = inputJointPositions.get(jointToCheck);
         YoDouble rawVelocity = inputJointVelocities.get(jointToCheck);
         YoDouble processedPosition = outputJointPositions.get(jointToCheck);
         YoDouble processedVelocity = outputJointVelocities.get(jointToCheck);
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
         YoFrameVector3D angularVelocity = intermediateAngularVelocities.get(imuToCheck);
         OrientationAngularVelocityConsistencyChecker consistencyChecker = new OrientationAngularVelocityConsistencyChecker(imuToCheck.getName(), orientation, angularVelocity, referenceFrame, updateDT, registry);
         consistencyCheckerMap.put(imuToCheck.getName(), consistencyChecker);
         diagnosticModules.add(consistencyChecker);
      }

      return consistencyCheckerMap;
   }

   public Map<OneDoFJointBasics, OneDoFJointForceTrackingDelayEstimator> addJointForceTrackingDelayEstimators(List<String> jointsToIgnore, JointDesiredOutputListReadOnly outputDataHolder)
   {
      LinkedHashMap<OneDoFJointBasics, OneDoFJointForceTrackingDelayEstimator> delayEstimatorMap = new LinkedHashMap<>();

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics jointToCheck = jointSensorDefinitions.get(i);

         if (jointsToIgnore.contains(jointToCheck.getName()))
            continue;

         OneDoFJointForceTrackingDelayEstimator delayEstimator = new OneDoFJointForceTrackingDelayEstimator(jointToCheck, outputDataHolder.getJointDesiredOutput(jointToCheck), updateDT, registry);
         delayEstimatorMap.put(jointToCheck, delayEstimator);
         diagnosticModules.add(delayEstimator);
      }

      return delayEstimatorMap;
   }

   public Map<OneDoFJointBasics, OneDoFJointFourierAnalysis> addJointFourierAnalysis(double estimationWindow, List<String> jointsToIgnore, JointDesiredOutputListReadOnly outputDataHolder)
   {
      LinkedHashMap<OneDoFJointBasics, OneDoFJointFourierAnalysis> jointFourierAnalysisMap = new LinkedHashMap<>();

      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics jointToCheck = jointSensorDefinitions.get(i);

         if (jointsToIgnore.contains(jointToCheck.getName()))
            continue;

         OneDoFJointFourierAnalysis jointFourierAnalysis = new OneDoFJointFourierAnalysis(jointToCheck, outputDataHolder.getJointDesiredOutput(jointToCheck), estimationWindow, updateDT, registry);
         jointFourierAnalysisMap.put(jointToCheck, jointFourierAnalysis);
         diagnosticModules.add(jointFourierAnalysis);
      }

      return jointFourierAnalysisMap;
   }

   /**
    * Create an alpha filter given a name and a break frequency (in Hertz) that will be registered in the {@code SensorProcessing}'s {@code YoVariableRegistry}.
    * @param name name of the variable.
    * @param breakFrequency break frequency in Hertz
    * @return a {@code YoDouble} to be used when adding a low-pass filter stage using the methods in this class such as {@link SensorProcessing#addJointVelocityAlphaFilter(YoDouble, boolean)}.
    */
   public DoubleProvider createAlphaFilter(String name, double breakFrequency)
   {
      DoubleProvider breakFrequencyParameter = new DoubleParameter(name, registry, breakFrequency);
      return () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequencyParameter.getValue(), updateDT);
   }

   /**
    * Helper to create easily a {@code Map<OneDoFJoint, YoDouble>} referring to the stiffness for each joint.
    * @param nameSuffix suffix to be used in the variables' name.
    * @param defaultStiffness default value of stiffness to use when not referred in the jointSpecificStiffness.
    * @param jointSpecificStiffness {@code Map<String, Double>} referring the specific stiffness value to be used for each joint. Does not need to be exhaustive, can also be empty or null in which the defaultStiffness is used for every joint.
    * @return {@code Map<OneDoFJoint, YoDouble>} to be used when calling {@link SensorProcessing#addJointPositionElasticyCompensator(Map, boolean)}.
    */
   public Map<OneDoFJointBasics, DoubleProvider> createStiffness(String nameSuffix, double defaultStiffness, Map<String, Double> jointSpecificStiffness)
   {
      return createStiffnessWithJointsToIgnore(nameSuffix, defaultStiffness, jointSpecificStiffness);
   }

   public Map<OneDoFJointBasics, DoubleProvider> createStiffnessWithJointsToIgnore(String nameSuffix, double defaultStiffness, Map<String, Double> jointSpecificStiffness, String... jointsToIgnore)
   {
      List<String> jointToIgnoreList = new ArrayList<>();
      if (jointsToIgnore != null && jointsToIgnore.length > 0)
         jointToIgnoreList.addAll(Arrays.asList(jointsToIgnore));

      LinkedHashMap<OneDoFJointBasics, DoubleProvider> stiffesses = new LinkedHashMap<>();
      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();
         
         if (jointToIgnoreList.contains(jointName))
            continue;

         DoubleProvider stiffness;
         if (jointSpecificStiffness != null && jointSpecificStiffness.containsKey(jointName))
            stiffness = new DoubleParameter(jointName + nameSuffix, registry, jointSpecificStiffness.get(jointName));
         else
            stiffness = new DoubleParameter(jointName + nameSuffix, registry, defaultStiffness);
         
         stiffesses.put(oneDoFJoint, stiffness);
      }
      
      return stiffesses;
   }

   /**
    * Helper to convert easily a map from joint name to @{@code YoDouble} ({@code Map<String, YoDouble>}) to a {@code Map<OneDoFJoint, YoDouble>}.
    * @param mapToConvert {@code Map<String, YoDouble>} the map to be converted, not modified.
    * @return {@code Map<OneDoFJoint, YoDouble>} the converted map.
    */
   public Map<OneDoFJointBasics, DoubleProvider> convertFromJointNameToJointMap(Map<String, DoubleProvider> mapToConvert)
   {
      LinkedHashMap<OneDoFJointBasics, DoubleProvider> newMap = new LinkedHashMap<>();
      for (int i = 0; i < jointSensorDefinitions.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointSensorDefinitions.get(i);
         String jointName = oneDoFJoint.getName();

         DoubleProvider yoDouble = mapToConvert.get(jointName);
         if (yoDouble != null)
            newMap.put(oneDoFJoint, yoDouble);
      }
      
      return newMap;
   }

   public DoubleProvider createMaxDeflection(String name, double defaultValue)
   {
      return new DoubleParameter(name, registry, defaultValue);
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

   private LinkedHashMap<OneDoFJointBasics, List<ProcessingYoVariable>> getProcessedJointSignals(SensorType sensorType)
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

   private LinkedHashMap<OneDoFJointBasics, YoDouble> getOutputJointSignals(SensorType sensorType)
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

   private LinkedHashMap<ForceSensorDefinition, YoFrameVector3D> getIntermediateForceSensorSignals(SensorType sensorType)
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

   private LinkedHashMap<IMUDefinition, YoFrameVector3D> getIntermediateIMUVectorTypeSignals(SensorType sensorType)
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
   public long getControllerTimestamp()
   {
      return controllerTimestamp.getLongValue();
   }

   @Override
   public long getSensorHeadPPSTimestamp()
   {
      return sensorHeadPPSTimetamp.getLongValue();
   }

   public void setJointEnabled(OneDoFJointBasics oneDoFJoint, boolean enabled)
   {
      jointEnabledIndicators.get(oneDoFJoint).set(enabled);
   }

   public void setJointPositionSensorValue(OneDoFJointBasics oneDoFJoint, double value)
   {
      inputJointPositions.get(oneDoFJoint).set(value);
   }

   public void setJointVelocitySensorValue(OneDoFJointBasics oneDoFJoint, double value)
   {
      inputJointVelocities.get(oneDoFJoint).set(value);
   }
   
   public void setJointAccelerationSensorValue(OneDoFJointBasics oneDoFJoint, double value)
   {
      inputJointAccelerations.get(oneDoFJoint).set(value);
   }

   public void setJointTauSensorValue(OneDoFJointBasics oneDoFJoint, double value)
   {
      inputJointTaus.get(oneDoFJoint).set(value);
   }

   public void setOrientationSensorValue(IMUDefinition imuDefinition, QuaternionReadOnly value)
   {
      inputOrientations.get(imuDefinition).set(value);
   }

   public void setOrientationSensorValue(IMUDefinition imuDefinition, RotationMatrixReadOnly value)
   {
      inputOrientations.get(imuDefinition).set(value);
   }

   public void setAngularVelocitySensorValue(IMUDefinition imuDefinition, Vector3DReadOnly value)
   {
      inputAngularVelocities.get(imuDefinition).set(value);
   }

   public void setLinearAccelerationSensorValue(IMUDefinition imuDefinition, Vector3DReadOnly value)
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
   public double getJointPositionProcessedOutput(OneDoFJointBasics oneDoFJoint)
   {
      return outputJointPositions.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointVelocityProcessedOutput(OneDoFJointBasics oneDoFJoint)
   {
      return outputJointVelocities.get(oneDoFJoint).getDoubleValue();
   }
   
   @Override
   public double getJointAccelerationProcessedOutput(OneDoFJointBasics oneDoFJoint)
   {
      return outputJointAccelerations.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointTauProcessedOutput(OneDoFJointBasics oneDoFJoint)
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
   public double getJointPositionRawOutput(OneDoFJointBasics oneDoFJoint)
   {
      return inputJointPositions.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointVelocityRawOutput(OneDoFJointBasics oneDoFJoint)
   {
      return inputJointVelocities.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointAccelerationRawOutput(OneDoFJointBasics oneDoFJoint)
   {
      return inputJointAccelerations.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public double getJointTauRawOutput(OneDoFJointBasics oneDoFJoint)
   {
      return inputJointTaus.get(oneDoFJoint).getDoubleValue();
   }

   @Override
   public boolean isJointEnabled(OneDoFJointBasics oneDoFJoint)
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
   public AtlasAuxiliaryRobotData getAuxiliaryRobotData()
   {
      return this.auxiliaryRobotData;
   }

   public void setAuxiliaryRobotData(AtlasAuxiliaryRobotData auxiliaryRobotData)
   {
      this.auxiliaryRobotData = auxiliaryRobotData;
   }

   public List<OneDoFJointBasics> getJointSensorDefinitions()
   {
      return jointSensorDefinitions;
   }
}
