package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.LinkedHashMap;
import java.util.Map.Entry;
import java.util.Set;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.sensorProcessing.sensorData.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.stateEstimation.JointAndIMUSensorDataSource;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class SimulatedSensorHolderAndReader implements Runnable
{
   
   private ControlFlowElement controllerDispatcher;
   
   
   private final LinkedHashMap<OneDoFJoint, SimulatedOneDoFJointPositionSensor> jointPositionSensors = new LinkedHashMap<OneDoFJoint,
                                                                                                          SimulatedOneDoFJointPositionSensor>();
   private final LinkedHashMap<OneDoFJoint, SimulatedOneDoFJointVelocitySensor> jointVelocitySensors = new LinkedHashMap<OneDoFJoint,
                                                                                                          SimulatedOneDoFJointVelocitySensor>();

   private final LinkedHashMap<IMUDefinition, SimulatedOrientationSensorFromRobot> orientationSensors = new LinkedHashMap<IMUDefinition,
                                                                                                           SimulatedOrientationSensorFromRobot>();

   private final LinkedHashMap<IMUDefinition, SimulatedAngularVelocitySensorFromRobot> angularVelocitySensors = new LinkedHashMap<IMUDefinition,
                                                                                                                   SimulatedAngularVelocitySensorFromRobot>();
   private final LinkedHashMap<IMUDefinition, SimulatedLinearAccelerationSensorFromRobot> linearAccelerationSensors =
      new LinkedHashMap<IMUDefinition, SimulatedLinearAccelerationSensorFromRobot>();

   private final LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensors = new LinkedHashMap<ForceSensorDefinition, WrenchCalculatorInterface>();
   

   private JointAndIMUSensorDataSource jointAndIMUSensorDataSource;
   
   
   private ForceSensorDataHolder forceSensorDataHolder;

   
   public SimulatedSensorHolderAndReader()
   {
   }

   public void addJointPositionSensorPort(OneDoFJoint oneDoFJoint, SimulatedOneDoFJointPositionSensor jointPositionSensor)
   {
      jointPositionSensors.put(oneDoFJoint, jointPositionSensor);
   }

   public void addJointVelocitySensorPort(OneDoFJoint oneDoFJoint, SimulatedOneDoFJointVelocitySensor jointVelocitySensor)
   {
      jointVelocitySensors.put(oneDoFJoint, jointVelocitySensor);
   }

   public void addOrientationSensorPort(IMUDefinition imuDefinition, SimulatedOrientationSensorFromRobot orientationSensor)
   {
      orientationSensors.put(imuDefinition, orientationSensor);
   }

   public void addAngularVelocitySensorPort(IMUDefinition imuDefinition, SimulatedAngularVelocitySensorFromRobot angularVelocitySensor)
   {
      angularVelocitySensors.put(imuDefinition, angularVelocitySensor);
   }

   public void addLinearAccelerationSensorPort(IMUDefinition imuDefinition, SimulatedLinearAccelerationSensorFromRobot linearAccelerationSensor)
   {
      linearAccelerationSensors.put(imuDefinition, linearAccelerationSensor);
   }
   
   public void addForceTorqueSensorPort(ForceSensorDefinition forceSensorDefinition, WrenchCalculatorInterface groundContactPointBasedWrenchCalculator)
   {
      forceTorqueSensors.put(forceSensorDefinition, groundContactPointBasedWrenchCalculator);
   }


   public void setJointAndIMUSensorDataSource(JointAndIMUSensorDataSource jointAndIMUSensorDataSource)
   {
      this.jointAndIMUSensorDataSource = jointAndIMUSensorDataSource;
   }

   public void run()
   {
      if(controllerDispatcher != null)
      {
         controllerDispatcher.waitUntilComputationIsDone();
      }
      
      
      Set<OneDoFJoint> jointsForPositionSensors = jointPositionSensors.keySet();
      for (OneDoFJoint oneDoFJoint : jointsForPositionSensors)
      {
         SimulatedOneDoFJointPositionSensor simulatedOneDoFJointPositionSensor = jointPositionSensors.get(oneDoFJoint);
         simulatedOneDoFJointPositionSensor.startComputation();
         simulatedOneDoFJointPositionSensor.waitUntilComputationIsDone();
         Double value = simulatedOneDoFJointPositionSensor.getJointPositionOutputPort().getData();
         jointAndIMUSensorDataSource.setJointPositionSensorValue(oneDoFJoint, value);
      }

      Set<OneDoFJoint> jointsForVelocitySensors = jointVelocitySensors.keySet();
      for (OneDoFJoint oneDoFJoint : jointsForVelocitySensors)
      {
         SimulatedOneDoFJointVelocitySensor simulatedOneDoFJointVelocitySensor = jointVelocitySensors.get(oneDoFJoint);
         simulatedOneDoFJointVelocitySensor.startComputation();
         simulatedOneDoFJointVelocitySensor.waitUntilComputationIsDone();
         Double value = simulatedOneDoFJointVelocitySensor.getJointVelocityOutputPort().getData();
         jointAndIMUSensorDataSource.setJointVelocitySensorValue(oneDoFJoint, value);
      }

      Set<IMUDefinition> orientationSensorDefinitions = orientationSensors.keySet();
      for (IMUDefinition imuDefinition : orientationSensorDefinitions)
      {
         SimulatedOrientationSensorFromRobot orientationSensor = orientationSensors.get(imuDefinition);
         orientationSensor.startComputation();
         orientationSensor.waitUntilComputationIsDone();
         Matrix3d value = orientationSensor.getOrientationOutputPort().getData();
         jointAndIMUSensorDataSource.setOrientationSensorValue(imuDefinition, value);
      }

      Set<IMUDefinition> angularVelocitySensorDefinitions = angularVelocitySensors.keySet();
      for (IMUDefinition imuDefinition : angularVelocitySensorDefinitions)
      {
         SimulatedAngularVelocitySensorFromRobot angularVelocitySensor = angularVelocitySensors.get(imuDefinition);
         angularVelocitySensor.startComputation();
         angularVelocitySensor.waitUntilComputationIsDone();
         Vector3d value = angularVelocitySensor.getAngularVelocityOutputPort().getData();
         jointAndIMUSensorDataSource.setAngularVelocitySensorValue(imuDefinition, value);
      }

      Set<IMUDefinition> linearAccelerationSensorDefinitions = linearAccelerationSensors.keySet();
      for (IMUDefinition imuDefinition : linearAccelerationSensorDefinitions)
      {
         SimulatedLinearAccelerationSensorFromRobot linearAccelerationSensor = linearAccelerationSensors.get(imuDefinition);
         linearAccelerationSensor.startComputation();
         linearAccelerationSensor.waitUntilComputationIsDone();
         Vector3d value = linearAccelerationSensor.getLinearAccelerationOutputPort().getData();
         jointAndIMUSensorDataSource.setLinearAccelerationSensorValue(imuDefinition, value);
      }
      
      
      if(forceSensorDataHolder != null)
      {
         for(Entry<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensorEntry : forceTorqueSensors.entrySet())
         {
            final WrenchCalculatorInterface forceTorqueSensor = forceTorqueSensorEntry.getValue();
            forceTorqueSensor.startComputation();
            forceTorqueSensor.waitUntilComputationIsDone();  
            forceSensorDataHolder.setForceSensorValue(forceTorqueSensorEntry.getKey(), forceTorqueSensor.getForceSensorOutputPort().getData());
         }
      }
      
      
      
      if(controllerDispatcher != null)
      {
         controllerDispatcher.startComputation();
      }
   }

   public void setControllerDispatcher(ControlFlowElement controllerDispatcher)
   {
      this.controllerDispatcher = controllerDispatcher;
   }

   
   public void setForceSensorDataHolder(ForceSensorDataHolder forceSensorDataHolder)
   {
      this.forceSensorDataHolder = forceSensorDataHolder;
   }
}
