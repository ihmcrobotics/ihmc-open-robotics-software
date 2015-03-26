package us.ihmc.steppr.hardware.sensorReader;

import java.io.IOException;
import java.util.EnumMap;
import java.util.List;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.StepprUtil;
import us.ihmc.steppr.hardware.state.StepprJointState;
import us.ihmc.steppr.hardware.state.StepprState;
import us.ihmc.steppr.hardware.state.StepprXSensState;
import us.ihmc.steppr.hardware.state.UDPStepprStateReader;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;

public class StepprSensorReader implements SensorReader
{
   private final YoVariableRegistry registry = new YoVariableRegistry("StepprSensorReader");

   private final StepprState state;
   private final UDPStepprStateReader reader;

   private final SensorProcessing sensorProcessing;
   private final RawJointSensorDataHolderMap rawJointSensorDataHolderMap;
   private final EnumMap<StepprJoint, OneDoFJoint> stepprJoints;

   private final SideDependentList<ForceSensorDefinition> forceSensorDefinitions = new SideDependentList<>();
   private final ForceSensorDataHolder forceSensorDataHolderForEstimator;

   private final IMUDefinition pelvisIMU;

   private final Quat4d quaternion = new Quat4d();
   private final Vector3d angularVelocity = new Vector3d();
   private final Vector3d linearAcceleration = new Vector3d();

   private final LongYoVariable corruptedPackets = new LongYoVariable("corruptedPackets", registry);

   public StepprSensorReader(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, ForceSensorDataHolder forceSensorDataHolderForEstimator,
         RawJointSensorDataHolderMap rawJointSensorDataHolderMap, DRCRobotSensorInformation sensorInformation,
         StateEstimatorParameters stateEstimatorParameters, YoVariableRegistry parentRegistry)
   {
      state = new StepprState(stateEstimatorParameters.getEstimatorDT(), registry);
      reader = new UDPStepprStateReader(state);

      sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, stateEstimatorParameters, registry);

      this.rawJointSensorDataHolderMap = rawJointSensorDataHolderMap;
      this.forceSensorDataHolderForEstimator = forceSensorDataHolderForEstimator;

      List<OneDoFJoint> jointList = stateEstimatorSensorDefinitions.getJointSensorDefinitions();
      stepprJoints = StepprUtil.createJointMap(jointList);

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

      for (ForceSensorDefinition definition : forceSensorDataHolderForEstimator.getForceSensorDefinitions())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            if (definition.getSensorName().equals(sensorInformation.getFeetForceSensorNames().get(robotSide)))
            {
               forceSensorDefinitions.put(robotSide, definition);
            }

         }
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void read()
   {
      long timestamp;
      try
      {
         timestamp = reader.receive();
         for (StepprJoint jointId : StepprJoint.values)
         {
            StepprJointState jointState = state.getJointState(jointId);
            OneDoFJoint joint = stepprJoints.get(jointId);

            RawJointSensorDataHolder rawJoint = rawJointSensorDataHolderMap.get(joint);

            state.updateRawSensorData(jointId, rawJoint);

            sensorProcessing.setJointPositionSensorValue(joint, jointState.getQ());
            sensorProcessing.setJointTauSensorValue(joint, jointState.getTau());
            sensorProcessing.setJointVelocitySensorValue(joint, jointState.getQd());
         }

         StepprXSensState xSensState = state.getXSensState();
         xSensState.getQuaternion(quaternion);
         xSensState.getGyro(angularVelocity);
         xSensState.getAccel(linearAcceleration);

         for (RobotSide robotSide : RobotSide.values)
         {
            forceSensorDataHolderForEstimator.setForceSensorValue(forceSensorDefinitions.get(robotSide), state.getFootWrench(robotSide));

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
}
