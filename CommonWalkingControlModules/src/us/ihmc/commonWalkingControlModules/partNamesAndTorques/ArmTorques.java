package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import us.ihmc.tools.ArrayTools;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmTorques
{
   private double[] torques = new double[ArmJointName.values().length];
   private final RobotSide robotSide;

   public ArmTorques(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   private ArmTorques(ArmTorques armTorques)
   {
      this.torques = armTorques.getTorquesCopy();
      this.robotSide = armTorques.getRobotSide();
   }

   public RobotSide getRobotSide()
   {
      return this.robotSide;
   }

   public double getTorque(ArmJointName armJointName)
   {
      return torques[armJointName.ordinal()];
   }


   public double[] getTorquesCopy()
   {
      return torques.clone();
   }

   public ArmTorques getArmTorquesCopy()
   {
      return new ArmTorques(this);
   }

   public void setTorque(ArmJointName armJointName, double torqueValue)
   {
      torques[armJointName.ordinal()] = torqueValue;
   }

   public void setArmTorquesToDoubleArray(double[] torques)
   {
      if (torques.length != this.torques.length)
         throw new RuntimeException("torques length must match this.torque length, torques.length=" + torques.length + ", expected length="
                                    + this.torques.length);

      for (int i = 0; i < torques.length; i++)
      {
         this.torques[i] = torques[i];
      }
   }

   public void setTorquesToZero()
   {
      for (int index = 0; index < torques.length; index++)
      {
         torques[index] = 0.0;
      }
   }

   public void printValues()
   {
      String ret = "\n\n" + this.robotSide + "\n";
      System.out.println(ret);
      ArrayTools.printArray(this.torques, System.out);
   }
}
