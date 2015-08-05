package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import us.ihmc.humanoidRobotics.partNames.SpineJointName;



public class SpineJointVelocities
{
   private double[] jointVelocities = new double[SpineJointName.values.length];

   public SpineJointVelocities()
   {
      setJointVelocitiesToNaN();
   }

   private SpineJointVelocities(SpineJointVelocities spineJointVelocities)
   {
      this.jointVelocities = spineJointVelocities.getJointVelocitiesCopy();
   }

   public double getJointVelocity(SpineJointName spineJointName)
   {
      return jointVelocities[spineJointName.ordinal()];
   }


   public double[] getJointVelocitiesCopy()
   {
      return jointVelocities.clone();
   }

   public SpineJointVelocities getCopy()
   {
      return new SpineJointVelocities(this);
   }

   public void setJointVelocity(SpineJointName spineJointName, double jointVelocity)
   {
      jointVelocities[spineJointName.ordinal()] = jointVelocity;
   }

   public void setSpineJointVelocitiesToDoubleArray(double[] jointVelocities)
   {
      if (jointVelocities.length != this.jointVelocities.length)
         throw new RuntimeException("joint velocities length must match this.jointVelocities length, torques.length=" + jointVelocities.length
                                    + ", expected length=" + this.jointVelocities.length);

      for (int i = 0; i < jointVelocities.length; i++)
      {
         this.jointVelocities[i] = jointVelocities[i];
      }
   }


   public void setJointVelocitiesToNaN()
   {
      setJointVelocitiesToValue(Double.NaN);
   }

   public void setJointVelocitiesToZero()
   {
      setJointVelocitiesToValue(0.0);
   }

   private void setJointVelocitiesToValue(double value)
   {
      for (int index = 0; index < jointVelocities.length; index++)
      {
         jointVelocities[index] = value;
      }
   }

   public String toString()
   {
      String ret = "The SpineJointVelocities:\n";

      for (SpineJointName spineJointName : SpineJointName.values)
      {
         ret += spineJointName.getCamelCaseNameForMiddleOfExpression() + " = " + jointVelocities[spineJointName.ordinal()] + "\n";
      }

      return ret;
   }
}
