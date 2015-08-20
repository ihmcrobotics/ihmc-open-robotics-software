package us.ihmc.commonWalkingControlModules.partNamesAndTorques;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class LegJointVelocities
{
   private final LegJointName[] legJointNames;
   
   /* 
    * Converting from Double to double and visa-versa creates new objects and is slow 
    */
   
//   private final EnumMap<LegJointName, Double> jointVelocities;
   private final double[] jointVelocities = new double[LegJointName.values().length];
   private final RobotSide robotSide;

   public static void validateLegJointVelocitiesArray(LegJointVelocities[] legJointVelocitiesArray)
   {
      if (legJointVelocitiesArray.length != RobotSide.values.length)
         throw new RuntimeException("LegJointVelocities lengths do not match.");
      if (legJointVelocitiesArray[RobotSide.LEFT.ordinal()].getRobotSide() != RobotSide.LEFT)
         throw new RuntimeException("LegJointVelocities sides are incorrect.");
      if (legJointVelocitiesArray[RobotSide.RIGHT.ordinal()].getRobotSide() != RobotSide.RIGHT)
         throw new RuntimeException("LegJointVelocities sides are incorrect.");
   }

   public LegJointVelocities(LegJointName[] legJointNames, RobotSide robotSide)
   {
      this.legJointNames = legJointNames;
//      jointVelocities = new EnumMap<LegJointName, Double>(LegJointName.class);

      this.robotSide = robotSide;
      setJointVelocitiesToNAN();
   }

   public RobotSide getRobotSide()
   {
      return this.robotSide;
   }

   public double getJointVelocity(LegJointName legJointName)
   {
//      return jointVelocities.get(legJointName);
      return jointVelocities[legJointName.ordinal()];
   }

   public void setJointVelocity(LegJointName legJointName, double jointVelocity)
   {
//      jointVelocities.put(legJointName, jointVelocity);
      jointVelocities[legJointName.ordinal()] = jointVelocity;
   }

   public void setLegJointVelocitiesToDoubleArray(double[] jointVelocities)
   {
      if (jointVelocities.length != this.legJointNames.length)
         throw new RuntimeException("joint angles length must match this.jointVelocities length, torques.length=" + jointVelocities.length
                                    + ", expected length=" + this.legJointNames.length);

      for (int i = 0; i < legJointNames.length; i++)
      {
//         this.jointVelocities.put(legJointNames[i], jointVelocities[i]);
         setJointVelocity(legJointNames[i], jointVelocities[i]);
      }
   }

   public void setJointVelocitiesToNAN()
   {
      for (LegJointName legJointName : legJointNames)
      {
//         jointVelocities.put(legJointName, Double.NaN);
         setJointVelocity(legJointName, Double.NaN);
      }
   }
   
   public DenseMatrix64F toDenseMatrix()
   {
      int size = legJointNames.length;
      DenseMatrix64F ret = new DenseMatrix64F(size, 1);

      for (int i = 0; i < size; i++)
      {
         ret.set(i, 0, getJointVelocity(legJointNames[i])); //jointVelocities.get(legJointNames[i]));
      }

      return ret;
   }

   public String toString()
   {
      String ret = "The " + robotSide.toString() + "LegJointVelocities:\n";

      for (LegJointName legJointName : legJointNames)
      {
         ret += legJointName.getCamelCaseNameForMiddleOfExpression() + " = " + getJointVelocity(legJointName) + "\n";
      }

      return ret;
   }

   public void set(LegJointVelocities legJointVelocities)
   {
      if(this.robotSide != legJointVelocities.robotSide)
         throw new RuntimeException("Wrong side!");
      
      for (LegJointName legJointName : legJointNames)
      {
         setJointVelocity(legJointName, legJointVelocities.getJointVelocity(legJointName));
      }
      
   }

   public LegJointName[] getLegJointNames()
   {
      return legJointNames;
   }
}
