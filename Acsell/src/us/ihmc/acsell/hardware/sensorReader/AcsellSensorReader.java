package us.ihmc.acsell.hardware.sensorReader;

import java.io.IOException;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.acsell.hardware.AcsellJoint;
import us.ihmc.acsell.hardware.state.AcsellJointState;
import us.ihmc.acsell.hardware.state.AcsellState;
import us.ihmc.acsell.hardware.state.AcsellXSensState;
import us.ihmc.acsell.hardware.state.UDPAcsellStateReader;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.model.DesiredJointDataHolder;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

public class AcsellSensorReader<JOINT extends Enum<JOINT> & AcsellJoint> implements SensorReader
{

   private final AcsellState<?,JOINT> state;
   private final UDPAcsellStateReader reader;

   private final SensorProcessing sensorProcessing;
   private final RawJointSensorDataHolderMap rawJointSensorDataHolderMap;
   
   private final JOINT[] jointNames;
   private final EnumMap<JOINT, OneDoFJoint> acsellJoints;

   private final SideDependentList<ForceSensorDefinition> forceSensorDefinitions = new SideDependentList<>();

   private final IMUDefinition pelvisIMU;

   private final Quaternion quaternion = new Quaternion();
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D linearAcceleration = new Vector3D();

   private final LongYoVariable corruptedPackets;
   
   private final DesiredJointDataHolder estimatorDesiredJointDataHolder;

   public AcsellSensorReader(AcsellState<?, JOINT> state, JOINT[] jointNames, EnumMap<JOINT, OneDoFJoint> acsellJoints,
         StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, RawJointSensorDataHolderMap rawJointSensorDataHolderMap,
         DRCRobotSensorInformation sensorInformation, StateEstimatorParameters stateEstimatorParameters,
         DesiredJointDataHolder estimatorDesiredJointDataHolder, YoVariableRegistry sensorReaderRegistry)
   {
      this.state = state;
      reader = new UDPAcsellStateReader(state);

      sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, stateEstimatorParameters, sensorReaderRegistry);

      this.rawJointSensorDataHolderMap = rawJointSensorDataHolderMap;

      this.jointNames = jointNames;
      this.acsellJoints = acsellJoints;
      
      this.estimatorDesiredJointDataHolder = estimatorDesiredJointDataHolder;

      List<IMUDefinition> imuList = stateEstimatorSensorDefinitions.getIMUSensorDefinitions();
      IMUDefinition pelvisIMU = null;
      for (IMUDefinition imu : imuList)
      {
         if (imu.getName().equals(sensorInformation.getPrimaryBodyImu()))
         {
            pelvisIMU = imu;
            break;
         }
      }
      this.pelvisIMU = pelvisIMU;

      for (ForceSensorDefinition definition : stateEstimatorSensorDefinitions.getForceSensorDefinitions())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            if (definition.getSensorName().equals(sensorInformation.getFeetForceSensorNames().get(robotSide)))
            {
               forceSensorDefinitions.put(robotSide, definition);
            }

         }
      }
      
      this.corruptedPackets = new LongYoVariable("corruptedPackets", sensorReaderRegistry);
   }

   @Override
   public void read()
   {
      long timestamp;
      try
      {
         timestamp = reader.receive();
         for (JOINT jointId : jointNames)
         {
            AcsellJointState jointState = state.getJointState(jointId);
            OneDoFJoint joint = acsellJoints.get(jointId);

            RawJointSensorDataHolder rawJoint = rawJointSensorDataHolderMap.get(joint);

            state.updateRawSensorData(jointId, rawJoint);

            sensorProcessing.setJointPositionSensorValue(joint, jointState.getQ());
//            sensorProcessing.setJointTauSensorValue(joint, jointState.getTau());
            
            // TODO: Test me!!!!
            sensorProcessing.setJointTauSensorValue(joint, estimatorDesiredJointDataHolder.get(joint).getTauDesired());
            
            sensorProcessing.setJointVelocitySensorValue(joint, jointState.getQd());
         }

         AcsellXSensState xSensState = state.getXSensState();
         xSensState.getQuaternion(quaternion);
         xSensState.getGyro(angularVelocity);
         xSensState.getAccel(linearAcceleration);

         for (RobotSide robotSide : RobotSide.values)
         {
            sensorProcessing.setForceSensorValue(forceSensorDefinitions.get(robotSide), state.getFootWrench(robotSide));
         }

         sensorProcessing.setOrientationSensorValue(pelvisIMU, quaternion);
         sensorProcessing.setAngularVelocitySensorValue(pelvisIMU, angularVelocity);
         sensorProcessing.setLinearAccelerationSensorValue(pelvisIMU, linearAcceleration);

         sensorProcessing.startComputation(timestamp, timestamp, -1);
      }
      catch (IOException e)
      {
         corruptedPackets.increment();
      }
   }

   public void connect() throws IOException
   {
      reader.connect();
   }

   public void disconnect()
   {
      reader.disconnect();
   }

   @Override
   public SensorOutputMapReadOnly getSensorOutputMapReadOnly()
   {
      return sensorProcessing;
   }

   @Override
   public SensorRawOutputMapReadOnly getSensorRawOutputMapReadOnly()
   {
      return sensorProcessing;
   }

   @Override public AuxiliaryRobotData newAuxiliaryRobotDataInstance()
   {
      return null;
   }
}
