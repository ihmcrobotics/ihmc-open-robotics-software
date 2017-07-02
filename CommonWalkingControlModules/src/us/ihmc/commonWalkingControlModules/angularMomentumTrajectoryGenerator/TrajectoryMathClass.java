package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.security.InvalidParameterException;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TrajectoryMathClass
{
   private static double tempVal;
   private static YoVariableRegistry testRegistry = new YoVariableRegistry("DummyRegistryForTrajectoryMath");
   private static YoPolynomial tempPoly1 = new YoPolynomial("TempPoly1", 100, testRegistry);
   private static YoPolynomial tempPoly2 = new YoPolynomial("TempPoly2", 100, testRegistry);
   
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
