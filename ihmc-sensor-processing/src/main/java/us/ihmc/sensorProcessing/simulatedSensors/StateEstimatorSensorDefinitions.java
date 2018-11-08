package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;

public class StateEstimatorSensorDefinitions
{
   private final ArrayList<OneDoFJointBasics> oneDoFJointSensorDefinitions = new ArrayList<OneDoFJointBasics>();
   private final ArrayList<IMUDefinition> imuSensorDefinitions = new ArrayList<IMUDefinition>();
   private final ArrayList<ForceSensorDefinition> forceSensorDefinitions = new ArrayList<ForceSensorDefinition>();

   public StateEstimatorSensorDefinitions()
   {
   }

   public ArrayList<OneDoFJointBasics> getJointSensorDefinitions()
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

   public void addJointSensorDefinitions(OneDoFJointBasics[] oneDoFJoints)
   {
      for (OneDoFJointBasics oneDoFJoint : oneDoFJoints)
         addJointSensorDefinition(oneDoFJoint);
   }
   public void addJointSensorDefinition(OneDoFJointBasics oneDoFJoint)
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
         addIMUSensorDefinition(imuDefinition);
   }

   public void addForceSensorDefinition(ForceSensorDefinition forceSensorDefinition)
   {
      forceSensorDefinitions.add(forceSensorDefinition);
   }
}
