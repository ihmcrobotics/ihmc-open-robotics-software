package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.RobotSide;

import com.mathworks.jama.Matrix;

public class LegJointVelocities
{
   private final LegJointName[] legJointNames;
   private final EnumMap<LegJointName, Double> jointVelocities;
   private final RobotSide robotSide;

   public static void validateLegJointVelocitiesArray(LegJointVelocities[] legJointVelocitiesArray)
   {
      if (legJointVelocitiesArray.length != RobotSide.values().length)
         throw new RuntimeException("LegJointVelocities lengths do not match.");
      if (legJointVelocitiesArray[RobotSide.LEFT.ordinal()].getRobotSide() != RobotSide.LEFT)
         throw new RuntimeException("LegJointVelocities sides are incorrect.");
      if (legJointVelocitiesArray[RobotSide.RIGHT.ordinal()].getRobotSide() != RobotSide.RIGHT)
         throw new RuntimeException("LegJointVelocities sides are incorrect.");
   }

   public LegJointVelocities(LegJointName[] legJointNames, RobotSide robotSide)
   {
      this.legJointNames = legJointNames;
      jointVelocities = new EnumMap<LegJointName, Double>(LegJointName.class);

      this.robotSide = robotSide;
      setJointVelocitiesToNAN();
   }

   public RobotSide getRobotSide()
   {
      return this.robotSide;
   }

   public double getJointVelocity(LegJointName legJointName)
   {
      return jointVelocities.get(legJointName);
   }

   public void setJointVelocity(LegJointName legJointName, double jointVelocity)
   {
      jointVelocities.put(legJointName, jointVelocity);
   }

   public void setLegJointVelocitiesToDoubleArray(double[] jointVelocities)
   {
      if (jointVelocities.length != this.legJointNames.length)
         throw new RuntimeException("joint angles length must match this.jointVelocities length, torques.length=" + jointVelocities.length
                                    + ", expected length=" + this.legJointNames.length);

      for (int i = 0; i < legJointNames.length; i++)
      {
         this.jointVelocities.put(legJointNames[i], jointVelocities[i]);
      }
   }

   public void setJointVelocitiesToNAN()
   {
      for (LegJointName legJointName : legJointNames)
      {
         jointVelocities.put(legJointName, Double.NaN);
      }
   }


   public Matrix toMatrix()
   {
      int size = legJointNames.length;
      Matrix ret = new Matrix(size, 1);

      for (int i = 0; i < size; i++)
      {
         ret.set(i, 0, jointVelocities.get(legJointNames[i]));
      }

      return ret;
   }

   public String toString()
   {
      String ret = "The " + robotSide.toString() + "LegJointVelocities:\n";

      for (LegJointName legJointName : legJointNames)
      {
         ret += legJointName.getCamelCaseNameForMiddleOfExpression() + " = " + jointVelocities.get(legJointNames) + "\n";
      }

      return ret;
   }
}
