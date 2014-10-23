package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;

import us.ihmc.utilities.ForceSensorDefinition;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class StateEstimatorSensorDefinitions
{
   private final ArrayList<OneDoFJoint> oneDoFJointsForPositionSensors = new ArrayList<OneDoFJoint>();
   private final ArrayList<OneDoFJoint> oneDoFJointsForVelocitySensors = new ArrayList<OneDoFJoint>();

   private final ArrayList<IMUDefinition> imuDefinitionsForOrientationSensors = new ArrayList<IMUDefinition>();
   private final ArrayList<IMUDefinition> imuDefinitionsForAngularVelocitySensors = new ArrayList<IMUDefinition>();
   private final ArrayList<IMUDefinition> imuDefinitionsForLinearAccelerationSensors = new ArrayList<IMUDefinition>();

   private final SideDependentList<ForceSensorDefinition> footForceSensors = new SideDependentList<ForceSensorDefinition>();
   private final ArrayList<ForceSensorDefinition> forceSensorDefinitions = new ArrayList<ForceSensorDefinition>();

   public ArrayList<ForceSensorDefinition> getForceSensorDefinitions()
   {
      return forceSensorDefinitions;
   }

   public StateEstimatorSensorDefinitions()
   {
   }

   public ArrayList<OneDoFJoint> getJointPositionSensorDefinitions()
   {
      return oneDoFJointsForPositionSensors;
   }

   public ArrayList<OneDoFJoint> getJointVelocitySensorDefinitions()
   {
      return oneDoFJointsForVelocitySensors;
   }

   public ArrayList<IMUDefinition> getOrientationSensorDefinitions()
   {
      return imuDefinitionsForOrientationSensors;
   }

   public ArrayList<IMUDefinition> getAngularVelocitySensorDefinitions()
   {
      return imuDefinitionsForAngularVelocitySensors;
   }

   public ArrayList<IMUDefinition> getLinearAccelerationSensorDefinitions()
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
   public void addForceSensorDefinition(ForceSensorDefinition forceSensorDefinition)
   {
      forceSensorDefinitions.add(forceSensorDefinition);
   }

}
