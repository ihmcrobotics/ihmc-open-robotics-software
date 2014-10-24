package us.ihmc.sensorProcessing.simulatedSensors;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.maps.ObjectObjectMap;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.BacklashCompensatingVelocityYoVariable;

import com.yobotics.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;

public class SimulatedSensorHolderAndReader implements SensorReader
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCPerfectSensorReader");
   private final IntegerYoVariable step = new IntegerYoVariable("step", registry);
  
   private final double estimatorDT;
   
   private final DoubleYoVariable yoTime;

   private final ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointPositionSensor> jointPositionSensors = new ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointPositionSensor>();
   private final ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointVelocitySensor> jointVelocitySensors = new ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointVelocitySensor>();
   private final ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointTorqueSensor> jointTorqueSensors = new ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointTorqueSensor>();

   private final ObjectObjectMap<IMUDefinition, SimulatedOrientationSensorFromRobot> orientationSensors = new ObjectObjectMap<IMUDefinition, SimulatedOrientationSensorFromRobot>();

   private final ObjectObjectMap<IMUDefinition, SimulatedAngularVelocitySensorFromRobot> angularVelocitySensors = new ObjectObjectMap<IMUDefinition, SimulatedAngularVelocitySensorFromRobot>();
   private final ObjectObjectMap<IMUDefinition, SimulatedLinearAccelerationSensorFromRobot> linearAccelerationSensors = new ObjectObjectMap<IMUDefinition, SimulatedLinearAccelerationSensorFromRobot>();

   private final ObjectObjectMap<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensors = new ObjectObjectMap<ForceSensorDefinition, WrenchCalculatorInterface>();

   private final SensorProcessing sensorProcessing;

   private final ForceSensorDataHolder forceSensorDataHolder;

   private final BooleanYoVariable useFiniteDifferencesForVelocities;
   private final DoubleYoVariable alphaFiniteDifferences;
   private final DoubleYoVariable slopTimeFiniteDifferences;

   private ObjectObjectMap<OneDoFJoint, BacklashCompensatingVelocityYoVariable> finiteDifferenceVelocities;

   public SimulatedSensorHolderAndReader(SensorFilterParameters sensorFilterParameters, StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, ForceSensorDataHolder forceSensorDataHolderForEstimator, SensorNoiseParameters sensorNoiseParameters, DoubleYoVariable yoTime, YoVariableRegistry sensorReaderFactoryRegistry)
   {
      this.sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, sensorFilterParameters, sensorNoiseParameters, registry);
      this.forceSensorDataHolder = forceSensorDataHolderForEstimator;
      this.estimatorDT = sensorFilterParameters.getEstimatorDT();
      this.yoTime = yoTime;
      step.set(29831);

      useFiniteDifferencesForVelocities = new BooleanYoVariable("useFiniteDifferencesForVelocities", registry);
      alphaFiniteDifferences = new DoubleYoVariable("alphaFiniteDifferences", registry);
      slopTimeFiniteDifferences = new DoubleYoVariable("slopTimeFiniteDifferences", registry);

      alphaFiniteDifferences.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(
            sensorFilterParameters.getJointVelocityFilterFrequencyInHertz(), estimatorDT));
      slopTimeFiniteDifferences.set(sensorFilterParameters.getJointVelocitySlopTimeForBacklashCompensation());
      useFiniteDifferencesForVelocities.set(false);

      if (sensorReaderFactoryRegistry != null)
      {
         sensorReaderFactoryRegistry.addChild(registry);
      }
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

      if (finiteDifferenceVelocities == null)
         finiteDifferenceVelocities = new ObjectObjectMap<OneDoFJoint, BacklashCompensatingVelocityYoVariable>();

      BacklashCompensatingVelocityYoVariable finiteDifferenceVelocity = new BacklashCompensatingVelocityYoVariable("fd_qd_" + oneDoFJoint.getName(), "",
            alphaFiniteDifferences, estimatorDT, slopTimeFiniteDifferences, registry);
      finiteDifferenceVelocities.add(oneDoFJoint, finiteDifferenceVelocity);
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

   public void read()
   {
      for (int i = 0; i < jointPositionSensors.getLength(); i++)
      {
         OneDoFJoint oneDoFJoint = jointPositionSensors.getFirst(i);
         SimulatedOneDoFJointPositionSensor simulatedOneDoFJointPositionSensor = jointPositionSensors.getSecond(i);
         simulatedOneDoFJointPositionSensor.startComputation();
         simulatedOneDoFJointPositionSensor.waitUntilComputationIsDone();
         Double q = simulatedOneDoFJointPositionSensor.getJointPositionOutputPort().getData();
         sensorProcessing.setJointPositionSensorValue(oneDoFJoint, q);

         BacklashCompensatingVelocityYoVariable finiteDifferenceVelocity = finiteDifferenceVelocities.getSecond(i);
         finiteDifferenceVelocity.update(q);

         if (useFiniteDifferencesForVelocities.getBooleanValue())
         {
            sensorProcessing.setJointVelocitySensorValue(jointVelocitySensors.getFirst(i), finiteDifferenceVelocity.getDoubleValue());
         }
      }

      for (int i = 0; i < jointTorqueSensors.getLength(); i++)
      {
         OneDoFJoint oneDoFJoint = jointTorqueSensors.getFirst(i);
         SimulatedOneDoFJointTorqueSensor simulatedOneDoFJointTorqueSensor = jointTorqueSensors.getSecond(i);
         simulatedOneDoFJointTorqueSensor.startComputation();
         simulatedOneDoFJointTorqueSensor.waitUntilComputationIsDone();
         Double tau = simulatedOneDoFJointTorqueSensor.getJointTorqueOutputPort().getData();
         sensorProcessing.setJointTauSensorValue(oneDoFJoint, tau);
      }

      if (!useFiniteDifferencesForVelocities.getBooleanValue())
      {
         for (int i = 0; i < jointVelocitySensors.getLength(); i++)
         {
            SimulatedOneDoFJointVelocitySensor simulatedOneDoFJointVelocitySensor = jointVelocitySensors.getSecond(i);
            simulatedOneDoFJointVelocitySensor.startComputation();
            simulatedOneDoFJointVelocitySensor.waitUntilComputationIsDone();
            Double value = simulatedOneDoFJointVelocitySensor.getJointVelocityOutputPort().getData();
            sensorProcessing.setJointVelocitySensorValue(jointVelocitySensors.getFirst(i), value);
         }
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

      if (forceSensorDataHolder != null)
      {
         for (int i = 0; i < forceTorqueSensors.getLength(); i++)
         {
            final WrenchCalculatorInterface forceTorqueSensor = forceTorqueSensors.getSecond(i);
            forceTorqueSensor.calculate();
            forceSensorDataHolder.setForceSensorValue(forceTorqueSensors.getFirst(i), forceTorqueSensor.getWrench());
         }
      }

      long timestamp = TimeTools.secondsToNanoSeconds(yoTime.getDoubleValue());
      sensorProcessing.startComputation(timestamp, timestamp);

      step.increment();
      
   }
}
