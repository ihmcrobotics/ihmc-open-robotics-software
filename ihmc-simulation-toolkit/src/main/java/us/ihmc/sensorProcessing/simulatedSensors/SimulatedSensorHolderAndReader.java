package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class SimulatedSensorHolderAndReader implements SensorReader
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCPerfectSensorReader");
   protected final YoInteger step = new YoInteger("step", registry);

   protected final YoDouble yoTime;

   protected final List<Pair<OneDoFJointBasics, DoubleProvider>> jointPositionSensors = new ArrayList<>();
   protected final List<Pair<OneDoFJointBasics, DoubleProvider>> jointVelocitySensors = new ArrayList<>();
   protected final List<Pair<OneDoFJointBasics, DoubleProvider>> jointTorqueSensors = new ArrayList<>();
   protected final List<Pair<IMUDefinition, QuaternionProvider>> orientationSensors = new ArrayList<>();
   protected final List<Pair<IMUDefinition, Vector3DProvider>> angularVelocitySensors = new ArrayList<>();
   protected final List<Pair<IMUDefinition, Vector3DProvider>> linearAccelerationSensors = new ArrayList<>();
   protected final List<Pair<ForceSensorDefinition, WrenchCalculatorInterface>> forceTorqueSensors = new ArrayList<>();

   protected final SensorProcessing sensorProcessing;

   public SimulatedSensorHolderAndReader(StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions,
         SensorProcessingConfiguration sensorProcessingConfiguration, YoDouble yoTime, YoVariableRegistry parentRegistry)
   {
      this.sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, registry);
      this.yoTime = yoTime;
      step.set(29831);

      parentRegistry.addChild(registry);
   }

   public void addJointPositionSensorPort(OneDoFJointBasics oneDoFJoint, DoubleProvider jointPositionSensor)
   {
      jointPositionSensors.add(Pair.of(oneDoFJoint, jointPositionSensor));
   }

   public void addJointTorqueSensorPort(OneDoFJointBasics oneDoFJoint, DoubleProvider jointTorqueSensor)
   {
      jointTorqueSensors.add(Pair.of(oneDoFJoint, jointTorqueSensor));
   }

   public void addJointVelocitySensorPort(OneDoFJointBasics oneDoFJoint, DoubleProvider jointVelocitySensor)
   {
      jointVelocitySensors.add(Pair.of(oneDoFJoint, jointVelocitySensor));
   }

   public void addOrientationSensorPort(IMUDefinition imuDefinition, QuaternionProvider orientationSensor)
   {
      orientationSensors.add(Pair.of(imuDefinition, orientationSensor));
   }

   public void addAngularVelocitySensorPort(IMUDefinition imuDefinition, Vector3DProvider angularVelocitySensor)
   {
      angularVelocitySensors.add(Pair.of(imuDefinition, angularVelocitySensor));
   }

   public void addLinearAccelerationSensorPort(IMUDefinition imuDefinition, Vector3DProvider linearAccelerationSensor)
   {
      linearAccelerationSensors.add(Pair.of(imuDefinition, linearAccelerationSensor));
   }

   public void addForceTorqueSensorPort(ForceSensorDefinition forceSensorDefinition, WrenchCalculatorInterface groundContactPointBasedWrenchCalculator)
   {
      forceTorqueSensors.add(Pair.of(forceSensorDefinition, groundContactPointBasedWrenchCalculator));
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

   @Override
   public long read(SensorDataContext sensorDataContextToSet)
   {
      for (int i = 0; i < jointPositionSensors.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointPositionSensors.get(i).getLeft();
         double newValue = jointPositionSensors.get(i).getRight().getValue();
         sensorProcessing.setJointPositionSensorValue(oneDoFJoint, newValue);
      }

      for (int i = 0; i < jointTorqueSensors.size(); i++)
      {
         OneDoFJointBasics oneDoFJoint = jointTorqueSensors.get(i).getLeft();
         double newValue = jointTorqueSensors.get(i).getRight().getValue();
         sensorProcessing.setJointTauSensorValue(oneDoFJoint, newValue);
      }

      for (int i = 0; i < jointVelocitySensors.size(); i++)
      {
         double newValue = jointVelocitySensors.get(i).getRight().getValue();
         sensorProcessing.setJointVelocitySensorValue(jointVelocitySensors.get(i).getLeft(), newValue);
      }

      for (int i = 0; i < orientationSensors.size(); i++)
      {
         QuaternionReadOnly newValue = orientationSensors.get(i).getRight().getValue();
         sensorProcessing.setOrientationSensorValue(orientationSensors.get(i).getLeft(), newValue);
      }

      for (int i = 0; i < angularVelocitySensors.size(); i++)
      {
         Vector3DReadOnly newValue = angularVelocitySensors.get(i).getRight().getValue();
         sensorProcessing.setAngularVelocitySensorValue(angularVelocitySensors.get(i).getLeft(), newValue);
      }

      for (int i = 0; i < linearAccelerationSensors.size(); i++)
      {
         Vector3DReadOnly newValue = linearAccelerationSensors.get(i).getRight().getValue();
         sensorProcessing.setLinearAccelerationSensorValue(linearAccelerationSensors.get(i).getLeft(), newValue);
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
      return timestamp;
   }
}
