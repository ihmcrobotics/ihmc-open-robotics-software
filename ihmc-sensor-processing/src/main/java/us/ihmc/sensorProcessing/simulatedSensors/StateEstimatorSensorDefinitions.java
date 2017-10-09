package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

public class StateEstimatorSensorDefinitions
{
   private final ArrayList<OneDoFJoint> oneDoFJointSensorDefinitions = new ArrayList<OneDoFJoint>();
   private final ArrayList<IMUDefinition> imuSensorDefinitions = new ArrayList<IMUDefinition>();
   private final ArrayList<ForceSensorDefinition> forceSensorDefinitions = new ArrayList<ForceSensorDefinition>();

   public StateEstimatorSensorDefinitions()
   {
   }

   public ArrayList<OneDoFJoint> getJointSensorDefinitions()
   {
      return oneDoFJointSensorDefinitions;
   }

   public ArrayList<IMUDefinition> getIMUSensorDefinitions()
   {
      return imuSensorDefinitions;
   }

   public ArrayList<ForceSensorDefinition> getForceSensorDefinitions()
   {
      return forceSensorDefinitions;
   }

   public void addJointSensorDefinition(OneDoFJoint oneDoFJoint)
   {
      oneDoFJointSensorDefinitions.add(oneDoFJoint);
   }

   public void addIMUSensorDefinition(IMUDefinition imuDefinition)
   {
      imuSensorDefinitions.add(imuDefinition);
   }

   public void addIMUSensorDefinition(IMUDefinition[] imuDefinitions)
   {
      for (IMUDefinition imuDefinition : imuDefinitions)
         imuSensorDefinitions.add(imuDefinition);
   }

   public void addForceSensorDefinition(ForceSensorDefinition forceSensorDefinition)
   {
      forceSensorDefinitions.add(forceSensorDefinition);
   }
}
