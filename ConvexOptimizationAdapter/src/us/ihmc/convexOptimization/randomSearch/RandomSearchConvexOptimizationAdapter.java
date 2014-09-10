package us.ihmc.convexOptimization.randomSearch;

import java.util.Random;

import us.ihmc.convexOptimization.ConvexOptimizationAdapter;

public class RandomSearchConvexOptimizationAdapter implements ConvexOptimizationAdapter
{
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
      int numberOfVariables = linearCostFunctionFVector.length;
      
      double[] bestXGuess = new double[numberOfVariables];
      
      double bestCost = computeCost(linearCostFunctionFVector, bestXGuess);
      
      double[] bestLinearInequalityConstraintSums = null;
      if (linearInequalityConstraintCVectors != null)
      {
         bestLinearInequalityConstraintSums = new double[linearInequalityConstraintCVectors.length];
      }
      
      computeLinearInequalityConstraintSums(bestXGuess, linearInequalityConstraintCVectors, linearInequalityConstraintBs, bestLinearInequalityConstraintSums);
      boolean bestLinearInequalityConstraintsAreMet = areAllLinearInequalityConstraintsMet(bestLinearInequalityConstraintSums);
      

      double delta = 0.001;
      
      int numberOfIterations = 100000;
      int iterationNumber = 0;
      
      while(iterationNumber++ < numberOfIterations)
      {
         double[] newXGuess = generateRandomGuess(random, bestXGuess, delta);
         double newCost = computeCost(linearCostFunctionFVector, newXGuess);
         double[] newLinearInequalityConstraintSums = null;
         
         if (linearInequalityConstraintCVectors != null)
         {
            newLinearInequalityConstraintSums = new double[linearInequalityConstraintCVectors.length];
         }
         
         computeLinearInequalityConstraintSums(newXGuess, linearInequalityConstraintCVectors, linearInequalityConstraintBs, newLinearInequalityConstraintSums);
         boolean newLinearInequalityConstraintsAreMet = areAllLinearInequalityConstraintsMet(newLinearInequalityConstraintSums);

         if (bestLinearInequalityConstraintsAreMet && !newLinearInequalityConstraintsAreMet)
         {
            continue;
         }
         
         if (newLinearInequalityConstraintsAreMet && !bestLinearInequalityConstraintsAreMet)
         {
            System.out.println("Found a constraint satisfaction!");

            bestXGuess = newXGuess;
            bestCost = newCost;
            bestLinearInequalityConstraintSums = newLinearInequalityConstraintSums;
            bestLinearInequalityConstraintsAreMet = newLinearInequalityConstraintsAreMet;
            continue;
         }
         
         if (!newLinearInequalityConstraintsAreMet)
         {
            if (areAllInequalityConstraintsSameOrImproved(bestLinearInequalityConstraintSums, newLinearInequalityConstraintSums))
            {
               System.out.println("Found reduced constraint violations!");

               bestXGuess = newXGuess;
               bestCost = newCost;
               bestLinearInequalityConstraintSums = newLinearInequalityConstraintSums;
               bestLinearInequalityConstraintsAreMet = newLinearInequalityConstraintsAreMet;
            }
            
            continue;
         }
         

         if (newCost < bestCost)
         {
            System.out.println("Found better cost! bestCost = " + bestCost);
            
            bestXGuess = newXGuess;
            bestCost = newCost;
            bestLinearInequalityConstraintSums = newLinearInequalityConstraintSums;
            bestLinearInequalityConstraintsAreMet = newLinearInequalityConstraintsAreMet;
            continue;
         }

      }
      
      if (bestLinearInequalityConstraintsAreMet) return bestXGuess;
      return null;
   }
   
   
   private double computeCost(double[] linearCostFunctionFVector, double[] xGuess)
   {
      double cost = 0.0;
      
      for (int i=0; i<linearCostFunctionFVector.length; i++)
      {
         cost = cost + linearCostFunctionFVector[i] * xGuess[i];
      }
      
      return cost;
   }

   private boolean areAllInequalityConstraintsSameOrImproved(double[] previousInequalityConstraintSums, double[] newInequalityConstraintSums)
   {
      for (int i=0; i<previousInequalityConstraintSums.length; i++)
      {
         if (newInequalityConstraintSums[i] > previousInequalityConstraintSums[i]) return false;
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
         if (sum > 0.0) return false;
      }
      
      return true;
   }

   private double[] generateRandomGuess(Random random, double[] bestXGuess, double delta)
   {
      double[] ret = new double[bestXGuess.length];
      
      for (int i=0; i<bestXGuess.length; i++)
      {
         ret[i] = bestXGuess[i] + delta * (1.0 - 2.0 * random.nextDouble());
      }
      
      return ret;
   }

   private void computeLinearInequalityConstraintSums(double[] xVector, double[][] linearInequalityConstraintCVectors, double[] linearInequalityConstraintBs, double[] linearInequalityConstraintSumsToPack)
   {
      if (linearInequalityConstraintCVectors == null)
      {
         return;
      }
      
      for (int i=0; i<linearInequalityConstraintCVectors.length; i++)
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
}
