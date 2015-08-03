package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import us.ihmc.robotics.humanoidRobot.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmJointPositions
{
   private double[] jointPositions = new double[ArmJointName.values().length];
   private final RobotSide robotSide;

   public static void validateArmJointpositionsArray(ArmJointPositions[] armJointPositionsArray)
   {
      if (armJointPositionsArray.length != RobotSide.values.length)
         throw new RuntimeException("ArmJointPositions lengths do not match.");
      if (armJointPositionsArray[RobotSide.LEFT.ordinal()].getRobotSide() != RobotSide.LEFT)
         throw new RuntimeException("ArmJointPositions sides are incorrect.");
      if (armJointPositionsArray[RobotSide.RIGHT.ordinal()].getRobotSide() != RobotSide.RIGHT)
         throw new RuntimeException("ArmJointPositions sides are incorrect.");
   }

   public ArmJointPositions(RobotSide robotSide)
   {
      this.robotSide = robotSide;
      setJointsToNAN();
   }

   private ArmJointPositions(ArmJointPositions armJointPositions)
   {
      this.jointPositions = armJointPositions.getJointPositionsCopy();
      this.robotSide = armJointPositions.getRobotSide();
   }

   public RobotSide getRobotSide()
   {
      return this.robotSide;
   }

   public double getJointPosition(ArmJointName armJointName)
   {
      return jointPositions[armJointName.ordinal()];
   }


   public double[] getJointPositionsCopy()
   {
      return jointPositions.clone();
   }

   public ArmJointPositions getCopy()
   {
      return new ArmJointPositions(this);
   }

   public void setJointposition(ArmJointName armJointName, double jointPosition)
   {
      jointPositions[armJointName.ordinal()] = jointPosition;
   }

   public void setArmJointPositionsToDoubleArray(double[] jointPositions)
   {
      if (jointPositions.length != this.jointPositions.length)
         throw new RuntimeException("joint positions length must match this.jointpositions length, torques.length=" + jointPositions.length
                                    + ", expected length=" + this.jointPositions.length);

      for (int i = 0; i < jointPositions.length; i++)
      {
         this.jointPositions[i] = jointPositions[i];
      }
   }


   public void setJointsToNAN()
   {
      for (int index = 0; index < jointPositions.length; index++)
      {
         jointPositions[index] = Double.NaN;
      }
   }

   public String toString()
   {
      String ret = "The " + robotSide.toString() + "ArmJointPositions:\n";

      for (ArmJointName armJointName : ArmJointName.values())
      {
         ret += armJointName.getCamelCaseNameForMiddleOfExpression() + " = " + jointPositions[armJointName.ordinal()] + "\n";
      }

      return ret;
   }
}
