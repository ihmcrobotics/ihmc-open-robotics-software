package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.LinkedHashMap;
import java.util.Set;

import us.ihmc.sensorProcessing.stateEstimation.JointSensorDataSource;
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

   private JointSensorDataSource jointSensorDataSource;

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


   public void setJointSensorDataSource(JointSensorDataSource jointSensorDataSource)
   {
      this.jointSensorDataSource = jointSensorDataSource;
   }

   public void run()
   {
      Set<OneDoFJoint> jointsForPositionSensors = jointPositionSensors.keySet();
      for (OneDoFJoint oneDoFJoint : jointsForPositionSensors)
      {
         SimulatedOneDoFJointPositionSensor simulatedOneDoFJointPositionSensor = jointPositionSensors.get(jointsForPositionSensors);
         simulatedOneDoFJointPositionSensor.startComputation();
         simulatedOneDoFJointPositionSensor.waitUntilComputationIsDone();
         Double value = simulatedOneDoFJointPositionSensor.getJointPositionOutputPort().getData();
         jointSensorDataSource.setJointPositionSensorValue(oneDoFJoint, value);
      }


   }
}
