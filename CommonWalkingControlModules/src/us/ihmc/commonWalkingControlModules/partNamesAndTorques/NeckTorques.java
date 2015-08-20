package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import us.ihmc.SdfLoader.partNames.NeckJointName;


public class NeckTorques
{
   private double[] torques = new double[NeckJointName.values().length];

   public NeckTorques()
   {
   }

   public NeckTorques(double[] torques)
   {
      this.torques = torques.clone();
   }

   private NeckTorques(NeckTorques neckTorques)
   {
      this.torques = neckTorques.getTorquesCopy();
   }

   public double getTorque(NeckJointName neckJointName)
   {
      return torques[neckJointName.ordinal()];
   }


   public double[] getTorquesCopy()
   {
      return torques.clone();
   }

   public NeckTorques getNeckTorquesCopy()
   {
      return new NeckTorques(this);
   }


   public void setTorque(NeckJointName neckJointName, double torqueValue)
   {
      torques[neckJointName.ordinal()] = torqueValue;
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
