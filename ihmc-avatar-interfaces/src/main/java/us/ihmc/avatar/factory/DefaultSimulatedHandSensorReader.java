package us.ihmc.avatar.factory;

import java.util.List;

import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJointHolder;

public class DefaultSimulatedHandSensorReader implements SimulatedHandSensorReader
{
   private final OneDegreeOfFreedomJointHolder robot;
   private final List<String> fingerJointNames;

   public DefaultSimulatedHandSensorReader(OneDegreeOfFreedomJointHolder robot, List<String> fingerJointNames)
   {
      this.robot = robot;
      this.fingerJointNames = fingerJointNames;
   }

   @Override
   public void read(SensorDataContext sensorDataContext)
   {
      for (int i = 0; i < fingerJointNames.size(); i++)
      {
         String jointName = fingerJointNames.get(i);
         double q = robot.getOneDegreeOfFreedomJoint(jointName).getQ();
         sensorDataContext.getMeasuredJointState(jointName).setPosition(q);
      }
   }
}
