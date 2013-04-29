package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class StateEstimatorSensorDefinitions
{
   private final ArrayList<OneDoFJoint> oneDoFJointsForPositionSensors = new ArrayList<OneDoFJoint>();
   private final ArrayList<OneDoFJoint> oneDoFJointsForVelocitySensors = new ArrayList<OneDoFJoint>();

   private final ArrayList<IMUDefinition> imuDefinitionsForOrientationSensors = new ArrayList<IMUDefinition>();
   private final ArrayList<IMUDefinition> imuDefinitionsForAngularVelocitySensors = new ArrayList<IMUDefinition>();
   private final ArrayList<IMUDefinition> imuDefinitionsForLinearAccelerationSensors = new ArrayList<IMUDefinition>();

   public StateEstimatorSensorDefinitions()
   {
   }

   public List<OneDoFJoint> getJointPositionSensorDefinitions()
   {
      return oneDoFJointsForPositionSensors;
   }

   public List<OneDoFJoint> getJointVelocitySensorDefinitions()
   {
      return oneDoFJointsForVelocitySensors;
   }

   public List<IMUDefinition> getOrientationSensorDefinitions()
   {
      return imuDefinitionsForOrientationSensors;
   }

   public List<IMUDefinition> getAngularVelocitySensorDefinitions()
   {
      return imuDefinitionsForAngularVelocitySensors;
   }

   public List<IMUDefinition> getLinearAccelerationSensorDefinitions()
   {
      return imuDefinitionsForLinearAccelerationSensors;
   }

   public void addJointPositionSensorDefinition(OneDoFJoint oneDoFJoint)
   {
      oneDoFJointsForPositionSensors.add(oneDoFJoint);
   }

   public void addJointVelocitySensorDefinition(OneDoFJoint oneDoFJoint)
   {
      oneDoFJointsForVelocitySensors.add(oneDoFJoint);
   }

   public void addOrientationSensorDefinition(IMUDefinition imuDefinition)
   {
      imuDefinitionsForOrientationSensors.add(imuDefinition);
   }

   public void addAngularVelocitySensorDefinition(IMUDefinition imuDefinition)
   {
      imuDefinitionsForAngularVelocitySensors.add(imuDefinition);
   }

   public void addLinearAccelerationSensorDefinition(IMUDefinition imuDefinition)
   {
      imuDefinitionsForLinearAccelerationSensors.add(imuDefinition);
   }
}
