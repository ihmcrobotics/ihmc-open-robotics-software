package us.ihmc.avatar.scs2;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

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
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class SCS2SensorReader implements SensorReader, SensorOutputMapReadOnly
{
   private final SimControllerInput controllerInput;

   private final PairList<IMUSensor, SimIMUSensor> imuSensors = new PairList<>();
   private final PairList<ForceSensorData, SimWrenchSensor> wrenchSensors = new PairList<>();

   private final List<OneDoFJointStateReadOnly> jointSensorOutputList = new ArrayList<>();
   private final Map<OneDoFJointBasics, OneDoFJointStateReadOnly> jointToSensorOutputMap = new HashMap<>();
   private final List<IMUSensor> imuOutputs = new ArrayList<>();
   private ForceSensorDataHolder forceSensorOutputs = null;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoLong timestamp = new YoLong("timestamp", registry);
   private final YoLong controllerTimestamp = new YoLong("controllerTimestamp", registry);
   private final YoLong sensorHeadPPSTimetamp = new YoLong("sensorHeadPPSTimetamp", registry);

   private final RigidBodyBasics rootBody;
   private final List<JointBasics> controllerJointList;
   private final List<JointReadOnly> simJointList;

   protected SensorProcessing sensorProcessing = null;

   public SCS2SensorReader(SimControllerInput controllerInput, FloatingJointBasics rootJoint, boolean usePerfectSensor)
   {
      this.controllerInput = controllerInput;

      rootBody = rootJoint.getPredecessor();
      simJointList = new ArrayList<>(controllerInput.getInput().getAllJoints());
      controllerJointList = new ArrayList<>(rootJoint.subtreeList());

      if (!usePerfectSensor)
      {
         simJointList.remove(0);
         controllerJointList.remove(0);

      }

      for (JointBasics joint : rootJoint.subtreeIterable())
      {
         if (joint instanceof OneDoFJointBasics)
         {
            OneDoFJointBasics oneDoFJoint = (OneDoFJointBasics) joint;
            OneDoFJointStateReadOnly jointSensorOutput = OneDoFJointStateReadOnly.createFromOneDoFJoint(oneDoFJoint, true);
            jointSensorOutputList.add(jointSensorOutput);
            jointToSensorOutputMap.put(oneDoFJoint, jointSensorOutput);
         }
      }
   }

   public void addIMUSensor(IMUDefinition definition)
   {
      SimIMUSensor simIMUSensor = controllerInput.getInput().findRigidBody(definition.getRigidBody().getName()).getParentJoint().getAuxialiryData()
                                                 .getIMUSensors().stream().filter(candidate -> candidate.getName().equals(definition.getName())).findFirst()
                                                 .get();
      IMUSensor imuSensor = new IMUSensor(definition, null);
      imuSensors.add(new ImmutablePair<>(imuSensor, simIMUSensor));
      imuOutputs.add(imuSensor);
   }

   public void addWrenchSensors(ForceSensorDataHolder forceSensorDataHolderToUpdate)
   {
      for (ForceSensorDefinition definition : forceSensorDataHolderToUpdate.getForceSensorDefinitions())
      {
         addWrenchSensor(definition, forceSensorDataHolderToUpdate.get(definition));
      }
      forceSensorOutputs = forceSensorDataHolderToUpdate;
   }

   private void addWrenchSensor(ForceSensorDefinition definition, ForceSensorData sensorDataToUpdate)
   {
      SimWrenchSensor simWrenchSensor = controllerInput.getInput().findRigidBody(definition.getRigidBody().getName()).getParentJoint().getAuxialiryData()
                                                       .getWrenchSensors().stream().filter(candidate -> candidate.getName().equals(definition.getSensorName()))
                                                       .findFirst().get();
      wrenchSensors.add(new ImmutablePair<>(sensorDataToUpdate, simWrenchSensor));
   }

   public void configureSensorProcessing(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions,
                                         SensorProcessingConfiguration sensorProcessingConfiguration)
   {
      sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, registry);
   }

   @Override
   public long read(SensorDataContext sensorDataContext)
   {
      long timestamp = Conversions.secondsToNanoseconds(controllerInput.getTime());
      this.timestamp.set(timestamp);
      controllerTimestamp.set(timestamp);
      sensorHeadPPSTimetamp.set(timestamp);

      MultiBodySystemTools.copyJointsState(simJointList, controllerJointList, JointStateType.CONFIGURATION);
      MultiBodySystemTools.copyJointsState(simJointList, controllerJointList, JointStateType.VELOCITY);
      rootBody.updateFramesRecursively();

      for (int i = 0; i < imuSensors.size(); i++)
      {
         IMUSensor controllerSensor = imuSensors.get(i).getLeft();
         SimIMUSensor simSensor = imuSensors.get(i).getRight();
         controllerSensor.setOrientationMeasurement(simSensor.getOrientation());
         controllerSensor.setAngularVelocityMeasurement(simSensor.getAngularVelocity());
         controllerSensor.setLinearAccelerationMeasurement(simSensor.getLinearAcceleration());
      }

      for (int i = 0; i < wrenchSensors.size(); i++)
      {
         ForceSensorData controllerData = wrenchSensors.get(i).getLeft();
         SimWrenchSensor simWrench = wrenchSensors.get(i).getRight();
         controllerData.setWrench(simWrench.getWrench().getAngularPart(), simWrench.getWrench().getLinearPart());
      }

      if (sensorProcessing != null)
      {
         for (int i = 0; i < sensorProcessing.getJointSensorDefinitions().size(); i++)
         {
            OneDoFJointBasics joint = sensorProcessing.getJointSensorDefinitions().get(i);
            sensorProcessing.setJointPositionSensorValue(joint, joint.getQ());
            sensorProcessing.setJointVelocitySensorValue(joint, joint.getQd());
            sensorProcessing.setJointTauSensorValue(joint, joint.getTau());
         }

         for (int i = 0; i < imuSensors.size(); i++)
         {
            sensorProcessing.s
         }

         for (int i = 0; i < wrenchSensors.size(); i++)
         {
            ForceSensorData controllerData = wrenchSensors.get(i).getLeft();
            SimWrenchSensor simWrench = wrenchSensors.get(i).getRight();
            controllerData.setWrench(simWrench.getWrench().getAngularPart(), simWrench.getWrench().getLinearPart());
         }
      }

      return getMonotonicTime();
   }

   @Override
   public SensorOutputMapReadOnly getProcessedSensorOutputMap()
   {
      return this;
   }

   @Override
   public SensorOutputMapReadOnly getRawSensorOutputMap()
   {
      return this;
   }

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
      return imuOutputs;
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorOutputs()
   {
      return forceSensorOutputs;
   }
}
