package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

public class SimulatedSensorHolderAndReaderWithYawDriftCorruptor extends SimulatedSensorHolderAndReader
{
   public SimulatedSensorHolderAndReaderWithYawDriftCorruptor(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions,
         SensorProcessingConfiguration sensorProcessingConfiguration, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, yoTime, parentRegistry);
   }

   @Override
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
         Matrix3d value = orientationSensor.getOrientationOutputPort().getData();
         sensorProcessing.setOrientationSensorValue(orientationSensors.get(i).getLeft(), value);
      }

      for (int i = 0; i < angularVelocitySensors.size(); i++)
      {
         SimulatedAngularVelocitySensorFromRobot angularVelocitySensor = angularVelocitySensors.get(i).getRight();
         angularVelocitySensor.startComputation();
         angularVelocitySensor.waitUntilComputationIsDone();
         Vector3d value = angularVelocitySensor.getAngularVelocityOutputPort().getData();
         sensorProcessing.setAngularVelocitySensorValue(angularVelocitySensors.get(i).getLeft(), value);
      }

      for (int i = 0; i < linearAccelerationSensors.size(); i++)
      {

         SimulatedLinearAccelerationSensorFromRobot linearAccelerationSensor = linearAccelerationSensors.get(i).getRight();
         linearAccelerationSensor.startComputation();
         linearAccelerationSensor.waitUntilComputationIsDone();
         Vector3d value = linearAccelerationSensor.getLinearAccelerationOutputPort().getData();
         sensorProcessing.setLinearAccelerationSensorValue(linearAccelerationSensors.get(i).getLeft(), value);
      }

      for (int i = 0; i < forceTorqueSensors.size(); i++)
      {
         final WrenchCalculatorInterface forceTorqueSensor = forceTorqueSensors.get(i).getRight();
         forceTorqueSensor.calculate();
         sensorProcessing.setForceSensorValue(forceTorqueSensors.get(i).getLeft(), forceTorqueSensor.getWrench());
      }

      long timestamp = TimeTools.secondsToNanoSeconds(yoTime.getDoubleValue());
      sensorProcessing.startComputation(timestamp, timestamp, -1);

      step.increment();
   }
}
