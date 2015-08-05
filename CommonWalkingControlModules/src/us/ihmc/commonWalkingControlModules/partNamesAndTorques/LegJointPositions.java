package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class LegJointPositions
{
   private double[] jointPositions = new double[LegJointName.values().length];
   private final RobotSide robotSide;

   public static void validateLegJointpositionsArray(LegJointPositions[] legJointPositionsArray)
   {
      if (legJointPositionsArray.length != RobotSide.values.length)
         throw new RuntimeException("LegJointPositions lengths do not match.");
      if (legJointPositionsArray[RobotSide.LEFT.ordinal()].getRobotSide() != RobotSide.LEFT)
         throw new RuntimeException("LegJointPositions sides are incorrect.");
      if (legJointPositionsArray[RobotSide.RIGHT.ordinal()].getRobotSide() != RobotSide.RIGHT)
         throw new RuntimeException("LegJointPositions sides are incorrect.");
   }

   public LegJointPositions(RobotSide robotSide)
   {
      this.robotSide = robotSide;
      setJointsToNAN();
   }

   private LegJointPositions(LegJointPositions legJointPositions)
   {
      this.jointPositions = legJointPositions.getJointPositionsCopy();
      this.robotSide = legJointPositions.getRobotSide();
   }

   public RobotSide getRobotSide()
   {
      return this.robotSide;
   }

   public double getJointPosition(LegJointName legJointName)
   {
      return jointPositions[legJointName.ordinal()];
   }


   public double[] getJointPositionsCopy()
   {
      double[] ret = new double[jointPositions.length];
      for (int i=0; i<jointPositions.length; i++)
      {
         ret[i] = jointPositions[i];
      }

      return ret;
   }

   public LegJointPositions getCopy()
   {
      return new LegJointPositions(this);
   }

   public void setJointPosition(LegJointName legJointName, double jointPosition)
   {
      jointPositions[legJointName.ordinal()] = jointPosition;
   }

   public void setLegJointPositionsToDoubleArray(double[] jointPositions)
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
      String ret = "The " + robotSide.toString() + "LegJointPositions:\n";

      for (LegJointName legJointName : LegJointName.values())
      {
         ret += legJointName.getCamelCaseNameForMiddleOfExpression() + " = " + jointPositions[legJointName.ordinal()] + "\n";
      }

      return ret;
   }

   public boolean epsilonEquals(LegJointPositions legJointPositions, double epsilon)
   {
      if (this.robotSide != legJointPositions.robotSide)
         return false;

      for (LegJointName legJointName : LegJointName.values())
      {
         double thisPosition = this.getJointPosition(legJointName);
         double thatPosition = legJointPositions.getJointPosition(legJointName);

         if (Double.isNaN(thisPosition) || Double.isNaN(thatPosition))
         {
            if (!Double.isNaN(thisPosition))
               return false;
            if (!Double.isNaN(thatPosition))
               return false;
         }

         if ((Math.abs(thisPosition - thatPosition) > epsilon))
            return false;
      }

      return true;
   }

   public void set(LegJointPositions legJointPositions)
   {
      
      if(this.robotSide != legJointPositions.robotSide)
         throw new RuntimeException("Wrong side!");
      
      for (LegJointName legJointName : LegJointName.values())
      {
         setJointPosition(legJointName, legJointPositions.getJointPosition(legJointName));
      }
   }
   
}
