package us.ihmc.exampleSimulations.simpleArm;

import us.ihmc.exampleSimulations.simpleArm.SimpleArmRobot.ArmJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class SimpleRobotInputOutputMap
{
   private final SimpleArmRobot robot;

   public SimpleRobotInputOutputMap(SimpleArmRobot robot)
   {
      this.robot = robot;
   }

   public void readFromSimulation()
   {
      robot.updateInverseDynamicsStructureFromSimulation();
   }

   public void writeToSimulation()
   {
      robot.updateSimulationFromInverseDynamicsTorques();
   }

   public double getYaw()
   {
      return robot.getJoint(ArmJoint.YAW).getQ();
   }

   public double getPitch1()
   {
      return robot.getJoint(ArmJoint.PITCH_1).getQ();
   }

   public double getPitch2()
   {
      return robot.getJoint(ArmJoint.PITCH_2).getQ();
   }

   public void addYawTorque(double yawTorque)
   {
      OneDoFJoint joint = robot.getJoint(ArmJoint.YAW);
      joint.setTau(joint.getTau() + yawTorque);
   }

   public void addPitch1Torque(double pitch1Torque)
   {
      OneDoFJoint joint = robot.getJoint(ArmJoint.PITCH_1);
      joint.setTau(joint.getTau() + pitch1Torque);
   }

   public void addPitch2Torque(double pitch2Torque)
   {
      OneDoFJoint joint = robot.getJoint(ArmJoint.PITCH_2);
      joint.setTau(joint.getTau() + pitch2Torque);
   }
}
