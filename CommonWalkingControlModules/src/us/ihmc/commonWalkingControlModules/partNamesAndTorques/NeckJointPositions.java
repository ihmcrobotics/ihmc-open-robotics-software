package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import us.ihmc.humanoidRobotics.partNames.NeckJointName;



public class NeckJointPositions
{
   private double[] jointPositions = new double[NeckJointName.values().length];

   public static void validateHeadJointPositions(NeckJointPositions[] neckJointAnglesArray)
   {
      if (neckJointAnglesArray.length != NeckJointName.values().length)
         throw new RuntimeException("NeckJointPositions wrong length.");
   }

   public NeckJointPositions()
   {
      setJointsToNAN();
   }

   private NeckJointPositions(NeckJointPositions HeadJointPositions)
   {
      this.jointPositions = HeadJointPositions.getJointPositionsCopy();
   }

   public double getJointPosition(NeckJointName HeadJointName)
   {
      return jointPositions[HeadJointName.ordinal()];
   }


   public double[] getJointPositionsCopy()
   {
      return jointPositions.clone();
   }

   public NeckJointPositions getHeadPositionsCopy()
   {
      return new NeckJointPositions(this);
   }


   public void setJointPosition(NeckJointName HeadJointName, double jointPosition)
   {
      jointPositions[HeadJointName.ordinal()] = jointPosition;
   }

   public void setHeadJointPositionsToDoubleArray(double[] jointPositions)
   {
      if (jointPositions.length != this.jointPositions.length)
         throw new RuntimeException("Input is length =" + jointPositions.length + ", expected length=" + this.jointPositions.length);

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
      String ret = "The headJointPositions:\n";

      for (NeckJointName headJointName : NeckJointName.values())
      {
         ret += headJointName.getCamelCaseNameForMiddleOfExpression() + " = " + jointPositions[headJointName.ordinal()] + "\n";
      }

      return ret;
   }

}
