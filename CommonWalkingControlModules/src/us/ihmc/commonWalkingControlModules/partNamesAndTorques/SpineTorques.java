package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import us.ihmc.SdfLoader.partNames.SpineJointName;


public class SpineTorques
{
   private double[] torques = new double[SpineJointName.values.length];

   public SpineTorques()
   {
   }

   private SpineTorques(SpineTorques spineTorques)
   {
      this.torques = spineTorques.getTorquesCopy();
   }

   public double getTorque(SpineJointName spineJointName)
   {
      return torques[spineJointName.ordinal()];
   }


   public double[] getTorquesCopy()
   {
      return torques.clone();
   }

   public SpineTorques getSpineTorquesCopy()
   {
      return new SpineTorques(this);
   }


   public void setTorque(SpineJointName spineJointName, double torqueValue)
   {
      torques[spineJointName.ordinal()] = torqueValue;
   }

   public void setTorques(double[] torques)
   {
      if (torques.length != this.torques.length)
         throw new RuntimeException("torques length must match this.torque length, torques.length=" + torques.length + ", expected length="
                                    + this.torques.length);

      this.torques = torques.clone();
   }

   public void setTorquesToZero()
   {
      for (int index = 0; index < torques.length; index++)
      {
         torques[index] = 0.0;
      }
   }
}
