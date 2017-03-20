package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;

public class SimulatedSensorHolderAndReader implements SensorReader
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCPerfectSensorReader");
   protected final IntegerYoVariable step = new IntegerYoVariable("step", registry);

   protected final DoubleYoVariable yoTime;

   protected final List<Pair<OneDoFJoint, SimulatedOneDoFJointPositionSensor>> jointPositionSensors = new ArrayList<>();
   protected final List<Pair<OneDoFJoint, SimulatedOneDoFJointVelocitySensor>> jointVelocitySensors = new ArrayList<>();
   protected final List<Pair<OneDoFJoint, SimulatedOneDoFJointTorqueSensor>> jointTorqueSensors = new ArrayList<>();
   protected final List<Pair<IMUDefinition, SimulatedOrientationSensorFromRobot>> orientationSensors = new ArrayList<>();
   protected final List<Pair<IMUDefinition, SimulatedAngularVelocitySensorFromRobot>> angularVelocitySensors = new ArrayList<>();
   protected final List<Pair<IMUDefinition, SimulatedLinearAccelerationSensorFromRobot>> linearAccelerationSensors = new ArrayList<>();
   protected final List<Pair<ForceSensorDefinition, WrenchCalculatorInterface>> forceTorqueSensors = new ArrayList<>();

   protected final SensorProcessing sensorProcessing;

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
      jointPositionSensors.add(Pair.of(oneDoFJoint, jointPositionSensor));
   }

   public void addJointTorqueSensorPort(OneDoFJoint oneDoFJoint, SimulatedOneDoFJointTorqueSensor jointTorqueSensor)
   {
      jointTorqueSensors.add(Pair.of(oneDoFJoint, jointTorqueSensor));
   }

   public void addJointVelocitySensorPort(OneDoFJoint oneDoFJoint, SimulatedOneDoFJointVelocitySensor jointVelocitySensor)
   {
      jointVelocitySensors.add(Pair.of(oneDoFJoint, jointVelocitySensor));
   }

   public void addOrientationSensorPort(IMUDefinition imuDefinition, SimulatedOrientationSensorFromRobot orientationSensor)
   {
      orientationSensors.add(Pair.of(imuDefinition, orientationSensor));
   }

   public void addAngularVelocitySensorPort(IMUDefinition imuDefinition, SimulatedAngularVelocitySensorFromRobot angularVelocitySensor)
   {
      angularVelocitySensors.add(Pair.of(imuDefinition, angularVelocitySensor));
   }

   public void addLinearAccelerationSensorPort(IMUDefinition imuDefinition, SimulatedLinearAccelerationSensorFromRobot linearAccelerationSensor)
   {
      linearAccelerationSensors.add(Pair.of(imuDefinition, linearAccelerationSensor));
   }

   public void addForceTorqueSensorPort(ForceSensorDefinition forceSensorDefinition, WrenchCalculatorInterface groundContactPointBasedWrenchCalculator)
   {
      forceTorqueSensors.add(Pair.of(forceSensorDefinition, groundContactPointBasedWrenchCalculator));
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
      for (int i = 0; i < jointPositionSensors.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointPositionSensors.get(i).getLeft();
         SimulatedOneDoFJointPositionSensor simulatedOneDoFJointPositionSensor = jointPositionSensors.get(i).getRight();
         simulatedOneDoFJointPositionSensor.startComputation();
         simulatedOneDoFJointPositionSensor.waitUntilComputationIsDone();
         double q = simulatedOneDoFJointPositionSensor.getJointPositionOutputPort().getData().doubleValue();
         sensorProcessing.setJointPositionSensorValue(oneDoFJoint, q);
      }

      for (int i = 0; i < jointTorqueSensors.size(); i++)
      {
         OneDoFJoint oneDoFJoint = jointTorqueSensors.get(i).getLeft();
         SimulatedOneDoFJointTorqueSensor simulatedOneDoFJointTorqueSensor = jointTorqueSensors.get(i).getRight();
         simulatedOneDoFJointTorqueSensor.startComputation();
         simulatedOneDoFJointTorqueSensor.waitUntilComputationIsDone();
         double tau = simulatedOneDoFJointTorqueSensor.getJointTorqueOutputPort().getData().doubleValue();
         sensorProcessing.setJointTauSensorValue(oneDoFJoint, tau);
      }

      for (int i = 0; i < jointVelocitySensors.size(); i++)
      {
         SimulatedOneDoFJointVelocitySensor simulatedOneDoFJointVelocitySensor = jointVelocitySensors.get(i).getRight();
         simulatedOneDoFJointVelocitySensor.startComputation();
         simulatedOneDoFJointVelocitySensor.waitUntilComputationIsDone();
         double value = simulatedOneDoFJointVelocitySensor.getJointVelocityOutputPort().getData().doubleValue();
         sensorProcessing.setJointVelocitySensorValue(jointVelocitySensors.get(i).getLeft(), value);
      }

      for (int i = 0; i < orientationSensors.size(); i++)
      {
         SimulatedOrientationSensorFromRobot orientationSensor = orientationSensors.get(i).getRight();
         orientationSensor.startComputation();
         orientationSensor.waitUntilComputationIsDone();
         RotationMatrix value = orientationSensor.getOrientationOutputPort().getData();
         sensorProcessing.setOrientationSensorValue(orientationSensors.get(i).getLeft(), value);
      }

      for (int i = 0; i < angularVelocitySensors.size(); i++)
      {
         SimulatedAngularVelocitySensorFromRobot angularVelocitySensor = angularVelocitySensors.get(i).getRight();
         angularVelocitySensor.startComputation();
         angularVelocitySensor.waitUntilComputationIsDone();
         Vector3D value = angularVelocitySensor.getAngularVelocityOutputPort().getData();
         sensorProcessing.setAngularVelocitySensorValue(angularVelocitySensors.get(i).getLeft(), value);
      }

      for (int i = 0; i < linearAccelerationSensors.size(); i++)
      {

         SimulatedLinearAccelerationSensorFromRobot linearAccelerationSensor = linearAccelerationSensors.get(i).getRight();
         linearAccelerationSensor.startComputation();
         linearAccelerationSensor.waitUntilComputationIsDone();
         Vector3D value = linearAccelerationSensor.getLinearAccelerationOutputPort().getData();
         sensorProcessing.setLinearAccelerationSensorValue(linearAccelerationSensors.get(i).getLeft(), value);
      }

      for (int i = 0; i < forceTorqueSensors.size(); i++)
      {
         final WrenchCalculatorInterface forceTorqueSensor = forceTorqueSensors.get(i).getRight();
         forceTorqueSensor.calculate();
         sensorProcessing.setForceSensorValue(forceTorqueSensors.get(i).getLeft(), forceTorqueSensor.getWrench());
      }

      long timestamp = Conversions.secondsToNanoseconds(yoTime.getDoubleValue());
      sensorProcessing.startComputation(timestamp, timestamp, -1);

      step.increment();
   }

   @Override public AuxiliaryRobotData newAuxiliaryRobotDataInstance()
   {
      return null;
   }
}
