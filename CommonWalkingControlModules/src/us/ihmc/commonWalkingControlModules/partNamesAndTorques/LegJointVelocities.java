package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;

/**
 * Created by IntelliJ IDEA.
 * User: Administrator
 * Date: Apr 21, 2010
 * Time: 2:29:21 PM
 * To change this template use File | Settings | File Templates.
 */
public class LegJointVelocities
{
   private double[] jointVelocities = new double[LegJointName.values().length];
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

   public LegJointVelocities(RobotSide robotSide)
   {
      this.robotSide = robotSide;
      setJointVelocitiesToNaN();
   }

   private LegJointVelocities(LegJointVelocities legJointVelocities)
   {
      this.jointVelocities = legJointVelocities.getJointVelocitiesCopy();
      this.robotSide = legJointVelocities.getRobotSide();
   }

   public RobotSide getRobotSide()
   {
      return this.robotSide;
   }

   public double getJointVelocity(LegJointName legJointName)
   {
      return jointVelocities[legJointName.ordinal()];
   }


   public double[] getJointVelocitiesCopy()
   {
      return jointVelocities.clone();
   }

   public LegJointVelocities getCopy()
   {
      return new LegJointVelocities(this);
   }

   public void setJointVelocity(LegJointName legJointName, double jointVelocity)
   {
      jointVelocities[legJointName.ordinal()] = jointVelocity;
   }

   public void setLegJointVelocitiesToDoubleArray(double[] jointVelocities)
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
      String ret = "The " + robotSide.toString() + "LegJointVelocities:\n";

      for (LegJointName legJointName : LegJointName.values())
      {
         ret += legJointName.getCamelCaseNameForMiddleOfExpression() + " = " + jointVelocities[legJointName.ordinal()] + "\n";
      }

      return ret;
   }
}
