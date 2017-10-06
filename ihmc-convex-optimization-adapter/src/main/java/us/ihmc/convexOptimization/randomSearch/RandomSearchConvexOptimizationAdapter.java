package us.ihmc.convexOptimization.randomSearch;

import java.util.Random;

import us.ihmc.convexOptimization.ConvexOptimizationAdapter;

public class RandomSearchConvexOptimizationAdapter implements ConvexOptimizationAdapter
{
   private static final boolean DEBUG = false;

   // Cost function to be minimized.
   private double[] linearCostFunctionFVector;

   private double[][] quadraticCostFunctionPMatrix;
   private double[] quadraticCostFunctionQVector;
   private double quadraticCostFunctionR;

   // Linear Equality Constraints.
   private double[][] linearEqualityConstraintsAMatrix;
   private double[] linearEqualityConstraintsBVector;

   // Linear Inequality Constraints.
   private double[][] linearInequalityConstraintCVectors;
   private double[] linearInequalityConstraintBs;



   public RandomSearchConvexOptimizationAdapter()
   {
   }

   public void setLinearCostFunctionVector(double[] linearCostFunctionFVector)
   {
      this.linearCostFunctionFVector = linearCostFunctionFVector;
   }

   public void setQuadraticCostFunction(double[][] quadraticCostFunctionPMatrix, double[] quadraticCostFunctionQVector, double quadraticCostFunctionR)
   {
      this.quadraticCostFunctionPMatrix = quadraticCostFunctionPMatrix;
      this.quadraticCostFunctionQVector = quadraticCostFunctionQVector;
      this.quadraticCostFunctionR = quadraticCostFunctionR;

      throw new RuntimeException("Not implemented yet!");
   }

   public void setLinearEqualityConstraintsAMatrix(double[][] linearEqualityConstraintsAMatrix)
   {
      this.linearEqualityConstraintsAMatrix = linearEqualityConstraintsAMatrix;
   }

   public void setLinearEqualityConstraintsBVector(double[] linearEqualityConstraintsBVector)
   {
      this.linearEqualityConstraintsBVector = linearEqualityConstraintsBVector;
   }

   public void setLinearInequalityConstraints(double[][] linearInequalityConstraintCVectors, double[] linearInequalityConstraintBs)
   {
      this.linearInequalityConstraintCVectors = linearInequalityConstraintCVectors;
      this.linearInequalityConstraintBs = linearInequalityConstraintBs;
   }


   public double[] solve()
   {
      Random random = new Random(1776L);
      double epsilonForLinearEqualityConstraints = 0.002;

      int numberOfVariables = linearCostFunctionFVector.length;

      double[] bestXGuess = new double[numberOfVariables];

      double bestCost = computeCost(linearCostFunctionFVector, bestXGuess);

      double[] bestLinearEqualityConstraintErrors = null;
      if (linearEqualityConstraintsAMatrix != null)
      {
         bestLinearEqualityConstraintErrors = new double[linearEqualityConstraintsAMatrix.length];
      }

      computeLinearEqualityConstraintErrors(bestXGuess, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, bestLinearEqualityConstraintErrors);
      boolean bestLinearEqualityConstraintsAreMet = areAllLinearEqualityConstraintsMet(bestLinearEqualityConstraintErrors, epsilonForLinearEqualityConstraints);

      double[] bestLinearInequalityConstraintSums = null;
      if (linearInequalityConstraintCVectors != null)
      {
         bestLinearInequalityConstraintSums = new double[linearInequalityConstraintCVectors.length];
      }

      computeLinearInequalityConstraintSums(bestXGuess, linearInequalityConstraintCVectors, linearInequalityConstraintBs, bestLinearInequalityConstraintSums);
      boolean bestLinearInequalityConstraintsAreMet = areAllLinearInequalityConstraintsMet(bestLinearInequalityConstraintSums);

      double minimumDelta = 0.001;
      double maximumDelta = 10.0;
      double deltaReductionFactor = 0.9;
      double deltaIncreaseFactor = 1.2;

      double delta = 0.001;

      int numberOfIterations = 100000;
      int iterationNumber = 0;

      while (iterationNumber++ < numberOfIterations)
      {
         double[] newXGuess = generateRandomGuess(random, bestXGuess, delta);
         double newCost = computeCost(linearCostFunctionFVector, newXGuess);
         double[] newLinearEqualityConstraintErrors = null;
         double[] newLinearInequalityConstraintSums = null;

         // Equality Constraints
         if (linearEqualityConstraintsAMatrix != null)
         {
            newLinearEqualityConstraintErrors = new double[linearEqualityConstraintsAMatrix.length];
         }

         computeLinearEqualityConstraintErrors(newXGuess, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector,
                 newLinearEqualityConstraintErrors);
         boolean newLinearEqualityConstraintsAreMet = areAllLinearEqualityConstraintsMet(newLinearEqualityConstraintErrors,
                                                         epsilonForLinearEqualityConstraints);

         if (bestLinearEqualityConstraintsAreMet &&!newLinearEqualityConstraintsAreMet)
         {
            delta = decreaseDelta(delta, deltaReductionFactor, minimumDelta);

            continue;
         }

         if (newLinearEqualityConstraintsAreMet &&!bestLinearEqualityConstraintsAreMet)
         {
            printIfDebug("Found a linear equality constraint satisfaction!");

            bestXGuess = newXGuess;
            bestCost = newCost;
            bestLinearEqualityConstraintErrors = newLinearEqualityConstraintErrors;
            bestLinearEqualityConstraintsAreMet = newLinearEqualityConstraintsAreMet;

            printOutBestGuess(bestXGuess);
            delta = increaseDelta(delta, deltaIncreaseFactor, maximumDelta);

            continue;
         }

         if (!newLinearEqualityConstraintsAreMet)
         {
            if (areAllEqualityConstraintsSameOrImproved(bestLinearEqualityConstraintErrors, newLinearEqualityConstraintErrors,
                    epsilonForLinearEqualityConstraints))
            {
               printIfDebug("Found reduced linear equality constraint violations!");

               bestXGuess = newXGuess;
               bestCost = newCost;
               bestLinearEqualityConstraintErrors = newLinearEqualityConstraintErrors;
               bestLinearEqualityConstraintsAreMet = newLinearEqualityConstraintsAreMet;

               printOutBestGuess(bestXGuess);
               delta = increaseDelta(delta, deltaIncreaseFactor, maximumDelta);
            }
            else
            {
               delta = decreaseDelta(delta, deltaReductionFactor, minimumDelta);
            }

            continue;
         }

         // Inequality Constraints
         if (linearInequalityConstraintCVectors != null)
         {
            newLinearInequalityConstraintSums = new double[linearInequalityConstraintCVectors.length];
         }

         computeLinearInequalityConstraintSums(newXGuess, linearInequalityConstraintCVectors, linearInequalityConstraintBs, newLinearInequalityConstraintSums);
         boolean newLinearInequalityConstraintsAreMet = areAllLinearInequalityConstraintsMet(newLinearInequalityConstraintSums);

         if (bestLinearInequalityConstraintsAreMet &&!newLinearInequalityConstraintsAreMet)
         {
            delta = decreaseDelta(delta, deltaReductionFactor, minimumDelta);

            continue;
         }

         if (newLinearInequalityConstraintsAreMet &&!bestLinearInequalityConstraintsAreMet)
         {
            printIfDebug("Found a linear inequality constraint satisfaction!");

            bestXGuess = newXGuess;
            bestCost = newCost;
            bestLinearInequalityConstraintSums = newLinearInequalityConstraintSums;
            bestLinearInequalityConstraintsAreMet = newLinearInequalityConstraintsAreMet;

            printOutBestGuess(bestXGuess);
            delta = increaseDelta(delta, deltaIncreaseFactor, maximumDelta);

            continue;
         }

         if (!newLinearInequalityConstraintsAreMet)
         {
            if (areAllInequalityConstraintsSameOrImproved(bestLinearInequalityConstraintSums, newLinearInequalityConstraintSums))
            {
               printIfDebug("Found reduced linear inequality constraint violations!");

               bestXGuess = newXGuess;
               bestCost = newCost;
               bestLinearInequalityConstraintSums = newLinearInequalityConstraintSums;
               bestLinearInequalityConstraintsAreMet = newLinearInequalityConstraintsAreMet;

               printOutBestGuess(bestXGuess);
               delta = increaseDelta(delta, deltaIncreaseFactor, maximumDelta);
            }
            else
            {
               delta = decreaseDelta(delta, deltaReductionFactor, minimumDelta);
            }

            continue;
         }


         if (newCost < bestCost)
         {
            printIfDebug("Found better cost! bestCost = " + bestCost);

            bestXGuess = newXGuess;
            bestCost = newCost;
            bestLinearInequalityConstraintSums = newLinearInequalityConstraintSums;
            bestLinearInequalityConstraintsAreMet = newLinearInequalityConstraintsAreMet;

            printOutBestGuess(bestXGuess);
            delta = increaseDelta(delta, deltaIncreaseFactor, maximumDelta);

            continue;
         }
         else
         {
            delta = decreaseDelta(delta, deltaReductionFactor, minimumDelta);
         }

      }

      if (bestLinearEqualityConstraintsAreMet && bestLinearInequalityConstraintsAreMet)
         return bestXGuess;

      return null;
   }


   private double decreaseDelta(double delta, double deltaReductionFactor, double minimumDelta)
   {
      double newDelta = delta * deltaReductionFactor;
      if (newDelta > minimumDelta)
         return newDelta;
      else
         return delta;
   }

   private double increaseDelta(double delta, double deltaIncreaseFactor, double maximumDelta)
   {
      double newDelta = delta * deltaIncreaseFactor;
      if (newDelta < maximumDelta)
         return newDelta;
      else
         return delta;
   }

   private void printOutBestGuess(double[] bestXGuess)
   {
      if (DEBUG)
      {
         System.out.print("best guess = (");

         for (int i = 0; i < bestXGuess.length; i++)
         {
            System.out.print(bestXGuess[i]);
            if (i < bestXGuess.length - 1)
               System.out.print(", ");
         }

         System.out.println(")");
      }

   }

   private double computeCost(double[] linearCostFunctionFVector, double[] xGuess)
   {
      double cost = 0.0;

      for (int i = 0; i < linearCostFunctionFVector.length; i++)
      {
         cost = cost + linearCostFunctionFVector[i] * xGuess[i];
      }

      return cost;
   }

   private boolean areAllEqualityConstraintsSameOrImproved(double[] previousLinearEqualityConstraintErrors, double[] newLinearEqualityConstraintErrors,
           double epsilonForLinearEqualityConstraints)
   {
      for (int i = 0; i < previousLinearEqualityConstraintErrors.length; i++)
      {
         boolean constraintNotSatisfied = Math.abs(newLinearEqualityConstraintErrors[i]) > epsilonForLinearEqualityConstraints;
         boolean errorIsWorse = Math.abs(newLinearEqualityConstraintErrors[i]) > Math.abs(previousLinearEqualityConstraintErrors[i]);
         if (constraintNotSatisfied && errorIsWorse)
            return false;
      }

      return true;
   }

   private boolean areAllInequalityConstraintsSameOrImproved(double[] previousInequalityConstraintSums, double[] newInequalityConstraintSums)
   {
      for (int i = 0; i < previousInequalityConstraintSums.length; i++)
      {
         if (newInequalityConstraintSums[i] > previousInequalityConstraintSums[i])
            return false;
      }

      return true;
   }

   private boolean areAllLinearInequalityConstraintsMet(double[] linearInequalityConstraintSums)
   {
      if (linearInequalityConstraintSums == null)
      {
         return true;
      }

      for (double sum : linearInequalityConstraintSums)
      {
         if (sum > 0.0)
            return false;
      }

      return true;
   }

   private boolean areAllLinearEqualityConstraintsMet(double[] linearEqualityConstraintErrors, double epsilon)
   {
      if (linearEqualityConstraintErrors == null)
      {
         return true;
      }

      for (double error : linearEqualityConstraintErrors)
      {
         if (Math.abs(error) > epsilon)
            return false;
      }

      return true;
   }

   private double[] generateRandomGuess(Random random, double[] bestXGuess, double delta)
   {
      double[] ret = new double[bestXGuess.length];

      for (int i = 0; i < bestXGuess.length; i++)
      {
         ret[i] = bestXGuess[i] + delta * (1.0 - 2.0 * random.nextDouble());
      }

      return ret;
   }

   private void computeLinearEqualityConstraintErrors(double[] xVector, double[][] linearEqualityConstraintsAMatrix, double[] linearEqualityConstraintsBVector,
           double[] linearEqualityConstraintErrorsToPack)
   {
      if (linearEqualityConstraintsAMatrix == null)
      {
         return;
      }

      for (int i = 0; i < linearEqualityConstraintsAMatrix.length; i++)
      {
         double linearEqualityConstraintSum = 0.0;
         for (int xIndex = 0; xIndex < linearEqualityConstraintsAMatrix[i].length; xIndex++)
         {
            linearEqualityConstraintSum = linearEqualityConstraintSum + linearEqualityConstraintsAMatrix[i][xIndex] * xVector[xIndex];
         }

         linearEqualityConstraintSum = linearEqualityConstraintSum - linearEqualityConstraintsBVector[i];
         linearEqualityConstraintErrorsToPack[i] = linearEqualityConstraintSum;
      }
   }

   private void computeLinearInequalityConstraintSums(double[] xVector, double[][] linearInequalityConstraintCVectors, double[] linearInequalityConstraintBs,
           double[] linearInequalityConstraintSumsToPack)
   {
      if (linearInequalityConstraintCVectors == null)
      {
         return;
      }

      for (int i = 0; i < linearInequalityConstraintCVectors.length; i++)
      {
         double linearInequalityConstraintSum = 0.0;
         for (int xIndex = 0; xIndex < linearInequalityConstraintCVectors[i].length; xIndex++)
         {
            linearInequalityConstraintSum = linearInequalityConstraintSum + linearInequalityConstraintCVectors[i][xIndex] * xVector[xIndex];
         }

         linearInequalityConstraintSum = linearInequalityConstraintSum - linearInequalityConstraintBs[i];
         linearInequalityConstraintSumsToPack[i] = linearInequalityConstraintSum;
      }
   }

   public void dispose()
   {
   }

   public void addQuadraticInequalities(double[][] pMatrix, double[] qVector, double r)
   {
      throw new RuntimeException("Not yet implemented!");
   }

   public void addSecondOrderConeConstraints(double[][] secondOrderConeAMatrix, double secondOrderConeBScalar, double[] secondOrderConeCVector,
           double secondOrderConeDScalar)
   {
      throw new RuntimeException("Not yet implemented");
   }

   public void addSecondOrderConeConstraints(double[][] secondOrderConeAMatrix, double[] secondOrderConeBVector, double[] secondOrderConeCVector,
           double secondOrderConeDScalar)
   {
      throw new RuntimeException("Not yet implemented");
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }
}
