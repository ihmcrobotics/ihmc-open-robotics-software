package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;

import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class StateEstimatorSensorDefinitions
{
   private final ArrayList<OneDoFJoint> oneDoFJointSensorDefinitions = new ArrayList<OneDoFJoint>();
   private final ArrayList<IMUDefinition> imuSensorDefinitions = new ArrayList<IMUDefinition>();
   private final ArrayList<ForceSensorDefinition> forceSensorDefinitions = new ArrayList<ForceSensorDefinition>();

   public ArrayList<ForceSensorDefinition> getForceSensorDefinitions()
   {
      return forceSensorDefinitions;
   }

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

   public void addJointSensorDefinition(OneDoFJoint oneDoFJoint)
   {
      oneDoFJointSensorDefinitions.add(oneDoFJoint);
   }

   public void addIMUSensorDefinition(IMUDefinition imuDefinition)
   {
      imuSensorDefinitions.add(imuDefinition);
   }

   public void addForceSensorDefinition(ForceSensorDefinition forceSensorDefinition)
   {
      forceSensorDefinitions.add(forceSensorDefinition);
   }
}
