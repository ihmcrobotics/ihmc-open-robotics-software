package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import java.util.EnumMap;

import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.humanoidRobotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.robotSide.RobotSide;

public class LegTorques implements LegTorquesInterface
{
   private final EnumMap<LegJointName, Double> torques = new EnumMap<LegJointName, Double>(LegJointName.class);
   private final RobotSide robotSide;
   private final RobotSpecificJointNames robotJointNames;
   private final LegJointName[] legJointNames;

   public static void validateLegTorquesArray(LegTorques[] legTorques)
   {
      if (legTorques.length != RobotSide.values.length)
         throw new RuntimeException("LegTorquesArray lengths do not match.");
      if (legTorques[RobotSide.LEFT.ordinal()].getRobotSide() != RobotSide.LEFT)
         throw new RuntimeException("LegTorquesArray sides are incorrect.");
      if (legTorques[RobotSide.RIGHT.ordinal()].getRobotSide() != RobotSide.RIGHT)
         throw new RuntimeException("LegTorquesArray sides are incorrect.");
   }
   
   public LegTorques(RobotSpecificJointNames robotJointNames, RobotSide robotSide)
   {
      this.robotJointNames = robotJointNames;
      this.legJointNames = robotJointNames.getLegJointNames();
      this.robotSide = robotSide;
      this.setTorquesToZero();
   }

   private LegTorques(LegTorques legTorques)
   {
      this(legTorques.robotJointNames, legTorques.robotSide);

      setLegTorques(legTorques);
   }

   public LegJointName[] getLegJointNames()
   {
      return legJointNames;
   }

   public void setLegTorques(LegTorques legTorques)
   {
      for (LegJointName legJointName : legJointNames)
      {
         this.torques.put(legJointName, legTorques.torques.get(legJointName));
      }
   }

   public RobotSide getRobotSide()
   {
      return this.robotSide;
   }

   public double getTorque(LegJointName legJointName)
   {
      return torques.get(legJointName);
   }


   public LegTorques getLegTorquesCopy()
   {
      return new LegTorques(this);
   }


   public void setTorque(LegJointName legJointName, double torqueValue)
   {
      torques.put(legJointName, torqueValue);
   }

   public void setLegTorquesToDoubleArray(double[] torques)
   {
      if (torques.length != this.legJointNames.length)
         throw new RuntimeException("torques length must match legJointNames length, torques.length=" + torques.length + ", expected length="
                                    + this.legJointNames.length);

      for (int i = 0; i < legJointNames.length; i++)
      {
         LegJointName legJointName = legJointNames[i];
         this.torques.put(legJointName, torques[i]);
      }
   }

   public void addTorque(LegJointName legJointName, double torqueAddValue)
   {
      torques.put(legJointName, torques.get(legJointName) + torqueAddValue);
   }

   public void setTorquesToZero()
   {
      for (int i = 0; i < legJointNames.length; i++)
      {
         LegJointName legJointName = legJointNames[i];
         this.torques.put(legJointName, 0.0);
      }
   }

   public void setKneeTorque(double kneeTorque)
   {
      this.setTorque(LegJointName.KNEE, kneeTorque);
   }

   public void addKneeTorque(double kneeTorque)
   {
      this.addTorque(LegJointName.KNEE, kneeTorque);
   }

   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("RobotSide: " + robotSide.getSideNameFirstLetter() + "\n");

      for (int i = 0; i < legJointNames.length - 1; i++)
      {
         LegJointName legJointName = legJointNames[i];
         builder.append(legJointName + ": " + this.torques.get(legJointName) + "\n");
      }

      LegJointName legJointName = legJointNames[legJointNames.length - 1];
      builder.append(legJointName + ": " + this.torques.get(legJointName));

      return builder.toString();
   }
}
