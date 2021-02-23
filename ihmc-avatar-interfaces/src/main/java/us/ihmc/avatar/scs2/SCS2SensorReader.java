package us.ihmc.avatar.scs2;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commons.Conversions;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.sensors.ForceSensorData;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.scs2.simulation.robot.controller.SimControllerInput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyReadOnly;
import us.ihmc.scs2.simulation.robot.sensors.SimIMUSensor;
import us.ihmc.scs2.simulation.robot.sensors.SimWrenchSensor;
import us.ihmc.sensorProcessing.imu.IMUSensor;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class SCS2SensorReader implements SensorReader
{
   private final StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();

   private final SimControllerInput controllerInput;

   private final List<SimIMUSensor> simIMUSensors = new ArrayList<>();
   private final List<IMUSensor> controllerIMUSensors;
   private final List<IMUDefinition> imuSensorDefinitions = stateEstimatorSensorDefinitions.getIMUSensorDefinitions();

   private final List<SimWrenchSensor> simWrenchSensors = new ArrayList<>();
   private final List<ForceSensorData> controllerWrenchSensors;
   private final List<ForceSensorDefinition> wrenchSensorDefinitions = stateEstimatorSensorDefinitions.getForceSensorDefinitions();
   private ForceSensorDataHolder forceSensorOutputs = null;

   private final List<OneDoFJointStateReadOnly> jointSensorOutputList = new ArrayList<>();
   private final Map<OneDoFJointBasics, OneDoFJointStateReadOnly> jointToSensorOutputMap = new HashMap<>();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoLong timestamp;
   private final YoLong controllerTimestamp;
   private final YoLong sensorHeadPPSTimetamp;

   private final RigidBodyBasics rootBody;
   private final List<JointBasics> controllerJointList;
   private final List<JointReadOnly> simJointList;

   private final SensorProcessing sensorProcessing;
   private final SensorOutputMapReadOnly perfectSensorOutputMap;

   private final boolean usePerfectSensor;

   public static SCS2SensorReader newSensorReader(SimControllerInput controllerInput, FloatingJointBasics rootJoint, IMUDefinition[] imuDefinitions,
                                                  ForceSensorDataHolder forceSensorDataHolder, SensorProcessingConfiguration sensorProcessingConfiguration)
   {
      return new SCS2SensorReader(controllerInput, rootJoint, imuDefinitions, forceSensorDataHolder, sensorProcessingConfiguration, false);
   }

   public static SCS2SensorReader newPerfectSensorReader(SimControllerInput controllerInput, FloatingJointBasics rootJoint)
   {
      return new SCS2SensorReader(controllerInput, rootJoint, null, null, null, true);
   }

   private SCS2SensorReader(SimControllerInput controllerInput, FloatingJointBasics rootJoint, IMUDefinition[] imuDefinitions,
                            ForceSensorDataHolder forceSensorDataHolder, SensorProcessingConfiguration sensorProcessingConfiguration, boolean usePerfectSensor)
   {
      this.controllerInput = controllerInput;
      this.usePerfectSensor = usePerfectSensor;

      rootBody = rootJoint.getPredecessor();
      simJointList = new ArrayList<>(controllerInput.getInput().getAllJoints());
      controllerJointList = new ArrayList<>(rootJoint.subtreeList());

      for (JointBasics joint : rootJoint.subtreeIterable())
      {
         if (joint instanceof OneDoFJointBasics)
         {
            OneDoFJointBasics oneDoFJoint = (OneDoFJointBasics) joint;
            OneDoFJointStateReadOnly jointSensorOutput = OneDoFJointStateReadOnly.createFromOneDoFJoint(oneDoFJoint, true);
            jointSensorOutputList.add(jointSensorOutput);
            jointToSensorOutputMap.put(oneDoFJoint, jointSensorOutput);
            stateEstimatorSensorDefinitions.addJointSensorDefinition(oneDoFJoint);
         }
      }

      if (usePerfectSensor)
      {
         sensorProcessing = null;

         controllerIMUSensors = new ArrayList<>();
         controllerWrenchSensors = new ArrayList<>();
         timestamp = new YoLong("timestamp", registry);
         controllerTimestamp = new YoLong("controllerTimestamp", registry);
         sensorHeadPPSTimetamp = new YoLong("sensorHeadPPSTimetamp", registry);
         perfectSensorOutputMap = new SensorOutputMapReadOnly()
         {
            @Override
            public long getWallTime()
            {
               return timestamp.getLongValue();
            }

            @Override
            public long getMonotonicTime()
            {
               return controllerTimestamp.getLongValue();
            }

            @Override
            public long getSyncTimestamp()
            {
               return sensorHeadPPSTimetamp.getLongValue();
            }

            @Override
            public OneDoFJointStateReadOnly getOneDoFJointOutput(OneDoFJointBasics oneDoFJoint)
            {
               return jointToSensorOutputMap.get(oneDoFJoint);
            }

            @Override
            public List<? extends OneDoFJointStateReadOnly> getOneDoFJointOutputs()
            {
               return jointSensorOutputList;
            }

            @Override
            public List<? extends IMUSensorReadOnly> getIMUOutputs()
            {
               return controllerIMUSensors;
            }

            @Override
            public ForceSensorDataHolderReadOnly getForceSensorOutputs()
            {
               return forceSensorOutputs;
            }
         };
      }
      else
      {
         for (IMUDefinition imuDefinition : imuDefinitions)
            addIMUSensor(imuDefinition);
         addWrenchSensors(forceSensorDataHolder);
         sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, registry);
         simJointList.remove(0);
         controllerJointList.remove(0);
         controllerIMUSensors = null;
         controllerWrenchSensors = null;
         timestamp = null;
         controllerTimestamp = null;
         sensorHeadPPSTimetamp = null;
         perfectSensorOutputMap = null;
      }
   }

   private void addIMUSensor(IMUDefinition definition)
   {
      SimRigidBodyReadOnly rigidBody = controllerInput.getInput().findRigidBody(definition.getRigidBody().getName());
      List<SimIMUSensor> sensorList = rigidBody.getParentJoint().getAuxialiryData().getIMUSensors();
      try
      {
         simIMUSensors.add(sensorList.stream().filter(candidate -> candidate.getName().equals(definition.getName())).findFirst().get());
         if (usePerfectSensor)
            controllerIMUSensors.add(new IMUSensor(definition, null));
         stateEstimatorSensorDefinitions.addIMUSensorDefinition(definition);
      }
      catch (Exception e)
      {
         throw new RuntimeException("Troublesome sensor: " + definition.getName() + ", sensor list: " + sensorList, e);
      }
   }

   private void addWrenchSensors(ForceSensorDataHolder forceSensorDataHolderToUpdate)
   {
      for (ForceSensorDefinition definition : forceSensorDataHolderToUpdate.getForceSensorDefinitions())
      {
         addWrenchSensor(definition, forceSensorDataHolderToUpdate.get(definition));
      }
      forceSensorOutputs = forceSensorDataHolderToUpdate;
   }

   private void addWrenchSensor(ForceSensorDefinition definition, ForceSensorData sensorDataToUpdate)
   {
      simWrenchSensors.add(controllerInput.getInput().findRigidBody(definition.getRigidBody().getName()).getParentJoint().getAuxialiryData().getWrenchSensors()
                                          .stream().filter(candidate -> candidate.getName().equals(definition.getSensorName())).findFirst().get());
      if (usePerfectSensor)
         controllerWrenchSensors.add(sensorDataToUpdate);
      stateEstimatorSensorDefinitions.addForceSensorDefinition(definition);
   }

   private final DMatrixRMaj wrenchMatrix = new DMatrixRMaj(6, 1);

   @Override
   public void initialize()
   {
      sensorProcessing.initialize();
   }

   @Override
   public long read(SensorDataContext sensorDataContext)
   {
      long timestamp = Conversions.secondsToNanoseconds(controllerInput.getTime());

      if (usePerfectSensor)
      {
         this.timestamp.set(timestamp);
         controllerTimestamp.set(timestamp);
         sensorHeadPPSTimetamp.set(timestamp);
      }

      MultiBodySystemTools.copyJointsState(simJointList, controllerJointList, JointStateType.CONFIGURATION);
      MultiBodySystemTools.copyJointsState(simJointList, controllerJointList, JointStateType.VELOCITY);
      rootBody.updateFramesRecursively();

      if (usePerfectSensor)
      {

         for (int i = 0; i < imuSensorDefinitions.size(); i++)
         {
            IMUSensor controllerSensor = controllerIMUSensors.get(i);
            SimIMUSensor simSensor = simIMUSensors.get(i);
            controllerSensor.setOrientationMeasurement(simSensor.getOrientation());
            controllerSensor.setAngularVelocityMeasurement(simSensor.getAngularVelocity());
            controllerSensor.setLinearAccelerationMeasurement(simSensor.getLinearAcceleration());
         }

         for (int i = 0; i < wrenchSensorDefinitions.size(); i++)
         {
            ForceSensorData controllerData = controllerWrenchSensors.get(i);
            SimWrenchSensor simWrench = simWrenchSensors.get(i);
            controllerData.setWrench(simWrench.getWrench().getAngularPart(), simWrench.getWrench().getLinearPart());
         }
      }
      else
      {
         for (int i = 0; i < sensorProcessing.getJointSensorDefinitions().size(); i++)
         {
            OneDoFJointBasics joint = sensorProcessing.getJointSensorDefinitions().get(i);
            sensorProcessing.setJointPositionSensorValue(joint, joint.getQ());
            sensorProcessing.setJointVelocitySensorValue(joint, joint.getQd());
            sensorProcessing.setJointTauSensorValue(joint, joint.getTau());
         }

         for (int i = 0; i < imuSensorDefinitions.size(); i++)
         {
            IMUDefinition definition = imuSensorDefinitions.get(i);
            SimIMUSensor simSensor = simIMUSensors.get(i);
            sensorProcessing.setOrientationSensorValue(definition, simSensor.getOrientation());
            sensorProcessing.setAngularVelocitySensorValue(definition, simSensor.getAngularVelocity());
            sensorProcessing.setLinearAccelerationSensorValue(definition, simSensor.getLinearAcceleration());
         }

         for (int i = 0; i < wrenchSensorDefinitions.size(); i++)
         {
            ForceSensorDefinition definition = wrenchSensorDefinitions.get(i);
            SimWrenchSensor simWrench = simWrenchSensors.get(i);
            simWrench.getWrench().get(wrenchMatrix);
            sensorProcessing.setForceSensorValue(definition, wrenchMatrix);
         }

         sensorProcessing.startComputation(timestamp, timestamp, timestamp);
      }

      return timestamp;
   }

   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   @Override
   public SensorOutputMapReadOnly getProcessedSensorOutputMap()
   {
      return usePerfectSensor ? perfectSensorOutputMap : sensorProcessing;
   }

   @Override
   public SensorOutputMapReadOnly getRawSensorOutputMap()
   {
      return usePerfectSensor ? perfectSensorOutputMap : sensorProcessing.getRawSensorOutputMap();
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }
}
