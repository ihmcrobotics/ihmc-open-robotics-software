package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import us.ihmc.SdfLoader.partNames.SpineJointName;


public class SpineJointPositions
{
   private double[] jointPositions = new double[SpineJointName.values.length];

   public static void validateSpineJointPositions(SpineJointPositions[] spineJointAnglesArray)
   {
      if (spineJointAnglesArray.length != SpineJointName.values.length)
         throw new RuntimeException("SpineJointPositions wrong length.");
   }

   public SpineJointPositions()
   {
      setJointsToNAN();
   }

   private SpineJointPositions(SpineJointPositions spineJointPositions)
   {
      this.jointPositions = spineJointPositions.getJointPositionsCopy();
   }

   public double getJointPosition(SpineJointName spineJointName)
   {
      return jointPositions[spineJointName.ordinal()];
   }


   public double[] getJointPositionsCopy()
   {
      return jointPositions.clone();
   }

   public SpineJointPositions getSpinePositionsCopy()
   {
      return new SpineJointPositions(this);
   }


   public void setJointPosition(SpineJointName spineJointName, double jointPosition)
   {
      jointPositions[spineJointName.ordinal()] = jointPosition;
   }

   public void setSpineJointPositionsToDoubleArray(double[] jointPositions)
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
      String ret = "The spineJointPositions:\n";

      for (SpineJointName spineJointName : SpineJointName.values)
      {
         ret += spineJointName.getCamelCaseNameForMiddleOfExpression() + " = " + jointPositions[spineJointName.ordinal()] + "\n";
      }

      return ret;
   }


}
