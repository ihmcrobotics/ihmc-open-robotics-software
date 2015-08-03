package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import us.ihmc.robotics.humanoidRobot.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class LegJointAccelerations
{
   private final LegJointName[] legJointNames;

   /*
    * Converting from Double to double and visa-versa creates new objects and is
    * slow
    */

   //   private final EnumMap<LegJointName, Double> jointAccelerations;
   private final double[] jointAccelerations = new double[LegJointName.values().length];
   private final RobotSide robotSide;

   public static void validateLegJointAccelerationsArray(LegJointAccelerations[] legJointAccelerationsArray)
   {
      if (legJointAccelerationsArray.length != RobotSide.values.length)
         throw new RuntimeException("LegJointAccelerations lengths do not match.");
      if (legJointAccelerationsArray[RobotSide.LEFT.ordinal()].getRobotSide() != RobotSide.LEFT)
         throw new RuntimeException("LegJointAccelerations sides are incorrect.");
      if (legJointAccelerationsArray[RobotSide.RIGHT.ordinal()].getRobotSide() != RobotSide.RIGHT)
         throw new RuntimeException("LegJointAccelerations sides are incorrect.");
   }

   public LegJointAccelerations(LegJointName[] legJointNames, RobotSide robotSide)
   {
      this.legJointNames = legJointNames;
      //      jointAccelerations = new EnumMap<LegJointName, Double>(LegJointName.class);

      this.robotSide = robotSide;
      setJointAccelerationsToNAN();
   }

   public RobotSide getRobotSide()
   {
      return this.robotSide;
   }

   public double getJointAcceleration(LegJointName legJointName)
   {
      //      return jointAccelerations.get(legJointName);
      return jointAccelerations[legJointName.ordinal()];
   }

   public void setJointAcceleration(LegJointName legJointName, double jointAcceleration)
   {
      //      jointAccelerations.put(legJointName, jointAcceleration);
      jointAccelerations[legJointName.ordinal()] = jointAcceleration;
   }

   public void setLegJointAccelerationsToDoubleArray(double[] jointAccelerations)
   {
      if (jointAccelerations.length != this.legJointNames.length)
         throw new RuntimeException("joint angles length must match this.jointAccelerations length, torques.length=" + jointAccelerations.length
               + ", expected length=" + this.legJointNames.length);

      for (int i = 0; i < legJointNames.length; i++)
      {
         //         this.jointAccelerations.put(legJointNames[i], jointAccelerations[i]);
         setJointAcceleration(legJointNames[i], jointAccelerations[i]);
      }
   }

   public void setJointAccelerationsToNAN()
   {
      for (LegJointName legJointName : legJointNames)
      {
         //         jointAccelerations.put(legJointName, Double.NaN);
         setJointAcceleration(legJointName, Double.NaN);
      }
   }

   public String toString()
   {
      String ret = "The " + robotSide.toString() + "LegJointAccelerations:\n";

      for (LegJointName legJointName : legJointNames)
      {
         ret += legJointName.getCamelCaseNameForMiddleOfExpression() + " = " + getJointAcceleration(legJointName) + "\n";
      }

      return ret;
   }

   public void set(LegJointAccelerations legJointAccelerations)
   {
      if (this.robotSide != legJointAccelerations.robotSide)
         throw new RuntimeException("Wrong side!");

      for (LegJointName legJointName : legJointNames)
      {
         setJointAcceleration(legJointName, legJointAccelerations.getJointAcceleration(legJointName));
      }
   }
}
