package us.ihmc.sensorProcessing.simulatedSensors;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.utilities.maps.ObjectObjectMap;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.time.TimeTools;

public class SimulatedSensorHolderAndReader implements SensorReader
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCPerfectSensorReader");
   private final IntegerYoVariable step = new IntegerYoVariable("step", registry);

   private final DoubleYoVariable yoTime;

   private final ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointPositionSensor> jointPositionSensors = new ObjectObjectMap<>();
   private final ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointVelocitySensor> jointVelocitySensors = new ObjectObjectMap<>();
   private final ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointTorqueSensor> jointTorqueSensors = new ObjectObjectMap<>();

   private final ObjectObjectMap<IMUDefinition, SimulatedOrientationSensorFromRobot> orientationSensors = new ObjectObjectMap<>();

   private final ObjectObjectMap<IMUDefinition, SimulatedAngularVelocitySensorFromRobot> angularVelocitySensors = new ObjectObjectMap<>();
   private final ObjectObjectMap<IMUDefinition, SimulatedLinearAccelerationSensorFromRobot> linearAccelerationSensors = new ObjectObjectMap<>();

   private final ObjectObjectMap<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensors = new ObjectObjectMap<>();

   private final SensorProcessing sensorProcessing;

   public SimulatedSensorHolderAndReader(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions,
         SensorProcessingConfiguration sensorProcessingConfiguration, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      this.sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, registry);
      this.yoTime = yoTime;
      step.set(29831);

      parentRegistry.addChild(registry);
   }

   public void addJointPositionSensorPort(OneDoFJoint oneDoFJoint, SimulatedOneDoFJointPositionSensor jointPositionSensor)
   {
      jointPositionSensors.add(oneDoFJoint, jointPositionSensor);
   }

   public void addJointTorqueSensorPort(OneDoFJoint oneDoFJoint, SimulatedOneDoFJointTorqueSensor jointTorqueSensor)
   {
      jointTorqueSensors.add(oneDoFJoint, jointTorqueSensor);
   }

   public void addJointVelocitySensorPort(OneDoFJoint oneDoFJoint, SimulatedOneDoFJointVelocitySensor jointVelocitySensor)
   {
      jointVelocitySensors.add(oneDoFJoint, jointVelocitySensor);
   }

   public void addOrientationSensorPort(IMUDefinition imuDefinition, SimulatedOrientationSensorFromRobot orientationSensor)
   {
      orientationSensors.add(imuDefinition, orientationSensor);
   }

   public void addAngularVelocitySensorPort(IMUDefinition imuDefinition, SimulatedAngularVelocitySensorFromRobot angularVelocitySensor)
   {
      angularVelocitySensors.add(imuDefinition, angularVelocitySensor);
   }

   public void addLinearAccelerationSensorPort(IMUDefinition imuDefinition, SimulatedLinearAccelerationSensorFromRobot linearAccelerationSensor)
   {
      linearAccelerationSensors.add(imuDefinition, linearAccelerationSensor);
   }

   public void addForceTorqueSensorPort(ForceSensorDefinition forceSensorDefinition, WrenchCalculatorInterface groundContactPointBasedWrenchCalculator)
   {
      forceTorqueSensors.add(forceSensorDefinition, groundContactPointBasedWrenchCalculator);
   }

   public SensorOutputMapReadOnly getSensorOutputMapReadOnly()
   {
      return sensorProcessing;
   }

   @Override
   public SensorRawOutputMapReadOnly getSensorRawOutputMapReadOnly()
   {
      return sensorProcessing;
   }

   public void read()
   {
      for (int i = 0; i < jointPositionSensors.getLength(); i++)
      {
         OneDoFJoint oneDoFJoint = jointPositionSensors.getFirst(i);
         SimulatedOneDoFJointPositionSensor simulatedOneDoFJointPositionSensor = jointPositionSensors.getSecond(i);
         simulatedOneDoFJointPositionSensor.startComputation();
         simulatedOneDoFJointPositionSensor.waitUntilComputationIsDone();
         double q = simulatedOneDoFJointPositionSensor.getJointPositionOutputPort().getData().doubleValue();
         sensorProcessing.setJointPositionSensorValue(oneDoFJoint, q);
      }

      for (int i = 0; i < jointTorqueSensors.getLength(); i++)
      {
         OneDoFJoint oneDoFJoint = jointTorqueSensors.getFirst(i);
         SimulatedOneDoFJointTorqueSensor simulatedOneDoFJointTorqueSensor = jointTorqueSensors.getSecond(i);
         simulatedOneDoFJointTorqueSensor.startComputation();
         simulatedOneDoFJointTorqueSensor.waitUntilComputationIsDone();
         double tau = simulatedOneDoFJointTorqueSensor.getJointTorqueOutputPort().getData().doubleValue();
         sensorProcessing.setJointTauSensorValue(oneDoFJoint, tau);
      }

      for (int i = 0; i < jointVelocitySensors.getLength(); i++)
      {
         SimulatedOneDoFJointVelocitySensor simulatedOneDoFJointVelocitySensor = jointVelocitySensors.getSecond(i);
         simulatedOneDoFJointVelocitySensor.startComputation();
         simulatedOneDoFJointVelocitySensor.waitUntilComputationIsDone();
         double value = simulatedOneDoFJointVelocitySensor.getJointVelocityOutputPort().getData().doubleValue();
         sensorProcessing.setJointVelocitySensorValue(jointVelocitySensors.getFirst(i), value);
      }

      for (int i = 0; i < orientationSensors.getLength(); i++)
      {
         SimulatedOrientationSensorFromRobot orientationSensor = orientationSensors.getSecond(i);
         orientationSensor.startComputation();
         orientationSensor.waitUntilComputationIsDone();
         Matrix3d value = orientationSensor.getOrientationOutputPort().getData();
         sensorProcessing.setOrientationSensorValue(orientationSensors.getFirst(i), value);
      }

      for (int i = 0; i < angularVelocitySensors.getLength(); i++)
      {
         SimulatedAngularVelocitySensorFromRobot angularVelocitySensor = angularVelocitySensors.getSecond(i);
         angularVelocitySensor.startComputation();
         angularVelocitySensor.waitUntilComputationIsDone();
         Vector3d value = angularVelocitySensor.getAngularVelocityOutputPort().getData();
         sensorProcessing.setAngularVelocitySensorValue(angularVelocitySensors.getFirst(i), value);
      }

      for (int i = 0; i < linearAccelerationSensors.getLength(); i++)
      {

         SimulatedLinearAccelerationSensorFromRobot linearAccelerationSensor = linearAccelerationSensors.getSecond(i);
         linearAccelerationSensor.startComputation();
         linearAccelerationSensor.waitUntilComputationIsDone();
         Vector3d value = linearAccelerationSensor.getLinearAccelerationOutputPort().getData();
         sensorProcessing.setLinearAccelerationSensorValue(linearAccelerationSensors.getFirst(i), value);
      }

      for (int i = 0; i < forceTorqueSensors.getLength(); i++)
      {
         final WrenchCalculatorInterface forceTorqueSensor = forceTorqueSensors.getSecond(i);
         forceTorqueSensor.calculate();
         sensorProcessing.setForceSensorValue(forceTorqueSensors.getFirst(i), forceTorqueSensor.getWrench());
      }

      long timestamp = TimeTools.secondsToNanoSeconds(yoTime.getDoubleValue());
      sensorProcessing.startComputation(timestamp, timestamp, -1);

      step.increment();
   }

   @Override public AuxiliaryRobotData newAuxiliaryRobotDataInstance()
   {
      return null;
   }
}
