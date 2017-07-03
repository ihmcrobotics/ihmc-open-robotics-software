package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.security.InvalidParameterException;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TrajectoryMathTools
{
   private static double tempVal;
   private static YoVariableRegistry testRegistry = new YoVariableRegistry("DummyRegistryForTrajectoryMath");
   private static YoPolynomial tempPoly1 = new YoPolynomial("TempPoly1", 100, testRegistry);
   private static YoPolynomial tempPoly2 = new YoPolynomial("TempPoly2", 100, testRegistry);
   private static YoTrajectory tempTraj1 = new YoTrajectory("TempTraj1", 100, testRegistry);
   private static YoTrajectory tempTraj2 = new YoTrajectory("TempTraj2", 100, testRegistry);

   public static void scale(YoTrajectory trajToPack, YoTrajectory traj, double scalar)
   {
      for (int i = 0; i < traj.getNumberOfCoefficients(); i++)
         trajToPack.polynomial.setDirectlyFast(i, traj.getCoefficient(i) * scalar);
   }

   public static void add(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
   {
      validatePackingTrajectoryForLinearCombination(trajToPack, traj1, traj2);
      validateTrajectoryTimes(traj1, traj2);
      int numberOfCoeffsToSet = Math.max(traj1.getNumberOfCoefficients(), traj2.getNumberOfCoefficients());
      for (int i = 0; i < numberOfCoeffsToSet; i++)
      {
         tempVal = 0.0;
         if (i < traj1.getNumberOfCoefficients())
            tempVal += traj1.getCoefficient(i);
         if (i < traj2.getNumberOfCoefficients())
            tempVal += traj2.getCoefficient(i);
         trajToPack.polynomial.setDirectlyFast(i, tempVal);
      }
      trajToPack.polynomial.reshape(numberOfCoeffsToSet);
      trajToPack.setTime(traj1.getInitialTime(), traj2.getFinalTime());
   }

   public static void subtract(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
   {
      validatePackingTrajectoryForLinearCombination(trajToPack, traj1, traj2);
      validateTrajectoryTimes(traj1, traj2);
      int numberOfCoeffsToSet = Math.max(traj1.getNumberOfCoefficients(), traj2.getNumberOfCoefficients());
      for (int i = 0; i < numberOfCoeffsToSet; i++)
      {
         tempVal = 0.0;
         if (i < traj1.getNumberOfCoefficients())
            tempVal += traj1.getCoefficient(i);
         if (i < traj2.getNumberOfCoefficients())
            tempVal -= traj2.getCoefficient(i);
         trajToPack.polynomial.setDirectlyFast(i, tempVal);
      }
      trajToPack.polynomial.reshape(numberOfCoeffsToSet);
      trajToPack.setTime(traj1.getInitialTime(), traj2.getFinalTime());
   }

   public static void multiply(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
   {
      validatePackingTrajectoryForMultiplication(trajToPack, traj1, traj2);
      validateTrajectoryTimes(traj1, traj2);
      tempPoly1.set(traj1.polynomial);
      tempPoly2.set(traj2.polynomial);
      int numberOfCoeffsToSet = traj1.getNumberOfCoefficients() + traj2.getNumberOfCoefficients() - 1;
      for (int i = 0; i < numberOfCoeffsToSet; i++)
      {
         tempVal = 0.0;
         for (int j = i; j >= 0; j--)
         {
            if (tempPoly1.getNumberOfCoefficients() > j && tempPoly2.getNumberOfCoefficients() > i - j)
            {
               tempVal += tempPoly1.getCoefficient(j) * tempPoly2.getCoefficient(i - j);
            }
         }
         trajToPack.polynomial.setDirectlyFast(i, tempVal);
      }
      trajToPack.polynomial.reshape(numberOfCoeffsToSet);
      trajToPack.setTime(traj1.getInitialTime(), traj2.getFinalTime());
   }

   public static void scale(YoTrajectory3D trajToPack, YoTrajectory3D traj, double scalarX, double scalarY, double scalarZ)
   {
      scale(trajToPack.getYoTrajectoryX(), traj.getYoTrajectoryX(), scalarX);
      scale(trajToPack.getYoTrajectoryY(), traj.getYoTrajectoryY(), scalarY);
      scale(trajToPack.getYoTrajectoryZ(), traj.getYoTrajectoryZ(), scalarZ);
   }

   public static void scale(YoTrajectory3D trajToPack, YoTrajectory3D traj, double scalar)
   {
      scale(trajToPack, traj, scalar, scalar, scalar);
   }

   public static void add(YoTrajectory3D trajToPack, YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         add(trajToPack.getYoTrajectory(direction), traj1.getYoTrajectory(direction), traj2.getYoTrajectory(direction));
   }

   public static void subtract(YoTrajectory3D trajToPack, YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         subtract(trajToPack.getYoTrajectory(direction), traj1.getYoTrajectory(direction), traj2.getYoTrajectory(direction));
   }

   public static void dotProduct(YoTrajectory3D trajToPack, YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         multiply(trajToPack.getYoTrajectory(direction), traj1.getYoTrajectory(direction), traj2.getYoTrajectory(direction));
   }

   public static void dotProduct(YoTrajectory trajToPackX, YoTrajectory trajToPackY, YoTrajectory trajToPackZ, YoTrajectory traj1X, YoTrajectory traj1Y,
                                 YoTrajectory traj1Z, YoTrajectory traj2X, YoTrajectory traj2Y, YoTrajectory traj2Z)
   {
      multiply(trajToPackX, traj1X, traj2X);
      multiply(trajToPackY, traj1Y, traj2Y);
      multiply(trajToPackZ, traj1Z, traj2Z);
   }

   public static void crossProduct(YoTrajectory3D trajToPack, YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      crossProduct(trajToPack.getYoTrajectoryX(), trajToPack.getYoTrajectoryY(), trajToPack.getYoTrajectoryZ(), traj1.getYoTrajectoryX(),
                   traj1.getYoTrajectoryY(), traj1.getYoTrajectoryZ(), traj2.getYoTrajectoryX(), traj2.getYoTrajectoryY(), traj2.getYoTrajectoryZ());
   }

   public static void crossProduct(YoTrajectory trajToPackX, YoTrajectory trajToPackY, YoTrajectory trajToPackZ, YoTrajectory traj1X, YoTrajectory traj1Y,
                                   YoTrajectory traj1Z, YoTrajectory traj2X, YoTrajectory traj2Y, YoTrajectory traj2Z)
   {
      multiply(tempTraj1, traj1Y, traj2Z);
      multiply(tempTraj2, traj1Z, traj2Y);
      subtract(trajToPackX, tempTraj1, tempTraj2);

      multiply(tempTraj1, traj1X, traj2Z);
      multiply(tempTraj2, traj1Z, traj2X);
      subtract(trajToPackY, tempTraj2, tempTraj1);

      multiply(tempTraj1, traj1X, traj2Y);
      multiply(tempTraj2, traj1Y, traj2X);
      subtract(trajToPackZ, tempTraj1, tempTraj2);
   }

   public static void validateTrajectoryTimes(YoTrajectory traj1, YoTrajectory traj2)
   {
      if (!MathTools.epsilonCompare(traj1.getInitialTime(), traj2.getInitialTime(), Epsilons.ONE_THOUSANDTH)
            || !MathTools.epsilonCompare(traj1.getFinalTime(), traj2.getFinalTime(), Epsilons.ONE_THOUSANDTH))
      {
         PrintTools.warn("Time mismatch in trajectories being added");
         throw new InvalidParameterException();
      }
   }

   public static void validatePackingTrajectoryForLinearCombination(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
   {
      if (trajToPack.getMaximumNumberOfCoefficients() < traj1.getNumberOfCoefficients()
            || trajToPack.getMaximumNumberOfCoefficients() < traj2.getNumberOfCoefficients())
      {
         PrintTools.warn("Not enough coefficients to store result of trajectory addition");
         throw new InvalidParameterException();
      }
   }

   public static void validatePackingTrajectoryForMultiplication(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
   {
      if (trajToPack.getMaximumNumberOfCoefficients() < traj1.getNumberOfCoefficients() + traj2.getNumberOfCoefficients() - 1)
      {
         PrintTools.warn("Not enough coefficients to store result of trajectory multplication");
         throw new InvalidParameterException();
      }
   }
}
