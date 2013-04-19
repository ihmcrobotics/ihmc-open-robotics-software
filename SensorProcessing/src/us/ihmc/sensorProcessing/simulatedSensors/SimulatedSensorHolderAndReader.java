package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.LinkedHashMap;
import java.util.Set;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.stateEstimation.JointAndIMUSensorDataSource;
import us.ihmc.sensorProcessing.stateEstimation.PointVelocitySensorDataSource;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class SimulatedSensorHolderAndReader implements Runnable
{
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

   private final LinkedHashMap<PointVelocitySensorDefinition, SimulatedPointVelocitySensorFromRobot> pointVelocitySensors =
      new LinkedHashMap<PointVelocitySensorDefinition, SimulatedPointVelocitySensorFromRobot>();

   private JointAndIMUSensorDataSource jointAndIMUSensorDataSource;
   private PointVelocitySensorDataSource pointVelocitySensorDataSource;

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

   public void addPointVelocitySensorPort(PointVelocitySensorDefinition pointVelocitySensorDefinition,
           SimulatedPointVelocitySensorFromRobot pointVelocitySensor)
   {
      pointVelocitySensors.put(pointVelocitySensorDefinition, pointVelocitySensor);
   }


   public void setJointAndIMUSensorDataSource(JointAndIMUSensorDataSource jointAndIMUSensorDataSource)
   {
      this.jointAndIMUSensorDataSource = jointAndIMUSensorDataSource;
   }

   public void setPointVelocitySensorDataSource(PointVelocitySensorDataSource pointVelocitySensorDataSource)
   {
      this.pointVelocitySensorDataSource = pointVelocitySensorDataSource;
   }

   public void run()
   {
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

      if (pointVelocitySensorDataSource != null)
      {
         Set<PointVelocitySensorDefinition> pointVelocitySensorDefinitions = pointVelocitySensors.keySet();
         for (PointVelocitySensorDefinition pointVelocitySensorDefinition : pointVelocitySensorDefinitions)
         {
            SimulatedPointVelocitySensorFromRobot pointVelocitySensor = pointVelocitySensors.get(pointVelocitySensorDefinition);
            pointVelocitySensor.startComputation();
            pointVelocitySensor.waitUntilComputationIsDone();
            Vector3d value = pointVelocitySensor.getPointVelocityOutputPort().getData();
            pointVelocitySensorDataSource.setPointVelocitySensorValue(pointVelocitySensorDefinition, value);
         }
      }

   }
}
