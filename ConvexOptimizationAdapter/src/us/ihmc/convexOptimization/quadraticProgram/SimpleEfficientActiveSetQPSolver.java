package us.ihmc.convexOptimization.quadraticProgram;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.linearAlgebra.MatrixTools;

// Uses the algorithm and naming convention found in MIT Paper
// "An efficiently solvable quadratic program for stabilizing dynamic locomotion"
// by Scott Kuindersma, Frank Permenter, and Russ Tedrake.
public class SimpleEfficientActiveSetQPSolver implements SimpleActiveSetQPSolverInterface
{
   private int maxNumberOfIterations = 10;

   private final DenseMatrix64F quadraticCostQMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F quadraticCostQVector = new DenseMatrix64F(0, 0);
   private double quadraticCostScalar;

   private final DenseMatrix64F linearEqualityConstraintsAMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F linearEqualityConstraintsBVector = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F linearInequalityConstraintsCMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F linearInequalityConstraintsDVector = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F augmentedLinearEqualityConstraintsAMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F augmentedLinearEqualityConstraintsBVector = new DenseMatrix64F(0, 0);

   private final ArrayList<Integer> activeSetIndices = new ArrayList<>();

   // Some temporary matrices:
   private final DenseMatrix64F CBar = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F DBar = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F linearEqualityConstraintsAMatrixTranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F quadraticCostQMatrixInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F linearInequalityConstraintsCMatrixTranspose = new DenseMatrix64F(0, 0);
//   private final DenseMatrix64F augmentedLinearEqualityConstraintsAMatrixTranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F ATranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CBarTranspose = new DenseMatrix64F(0, 0);
   
   private final DenseMatrix64F QInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F AQInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F QInverseATranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CBarQInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F AQInverseATranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F AQInverseCBarTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CBarQInverseATranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F QInverseCBarTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CBarQInverseCBarTranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F bigMatrixForLagrangeMultiplierSolution = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F bigVectorForLagrangeMultiplierSolution = new DenseMatrix64F(0, 0);


   @Override
   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      this.maxNumberOfIterations = maxNumberOfIterations;
   }


   @Override
   public void clear()
   {      
      linearEqualityConstraintsAMatrix.reshape(0, 0);
//      linearEqualityConstraintsAMatrixTranspose.reshape(0, 0);
      linearEqualityConstraintsBVector.reshape(0, 0);
      quadraticCostQMatrix.reshape(0, 0);
      quadraticCostQVector.reshape(0, 0);
      //      numberOfVariablesToSolve = -1;

      quadraticCostQMatrix.reshape(0, 0);
      quadraticCostQVector.reshape(0, 0);

      linearEqualityConstraintsAMatrix.reshape(0, 0);
      //      linearEqualityConstraintsAMatrixTranspose.reshape(0, 0);
      linearEqualityConstraintsBVector.reshape(0, 0);

      linearInequalityConstraintsCMatrix.reshape(0, 0);
      linearInequalityConstraintsDVector.reshape(0, 0);

      augmentedLinearEqualityConstraintsAMatrix.reshape(0, 0);
      augmentedLinearEqualityConstraintsBVector.reshape(0, 0);

      CBar.reshape(0, 0);
      DBar.reshape(0, 0);
      activeSetIndices.clear();
   }

   @Override
   public void setQuadraticCostFunction(double[][] quadraticCostFunctionWMatrix, double[] quadraticCostFunctionGVector, double quadraticCostScalar)
   {
      setQuadraticCostFunction(new DenseMatrix64F(quadraticCostFunctionWMatrix), MatrixTools.createVector(quadraticCostFunctionGVector), quadraticCostScalar);
   }


   @Override
   public void setQuadraticCostFunction(DenseMatrix64F costQuadraticMatrix, DenseMatrix64F costLinearVector, double quadraticCostScalar)
   {
      DenseMatrix64F symCostQuadraticMatrix = new DenseMatrix64F(costQuadraticMatrix);
      CommonOps.transpose(symCostQuadraticMatrix);
      CommonOps.add(costQuadraticMatrix, symCostQuadraticMatrix, symCostQuadraticMatrix);
      CommonOps.scale(0.5, symCostQuadraticMatrix);
      this.quadraticCostQMatrix.set(symCostQuadraticMatrix);
      this.quadraticCostQVector.set(costLinearVector);
      this.quadraticCostScalar = quadraticCostScalar;
   }

   private final DenseMatrix64F computedObjectiveFunctionValue = new DenseMatrix64F(1, 1);

   @Override
   public double getObjectiveCost(DenseMatrix64F x)
   {
      MatrixTools.multQuad(x, quadraticCostQMatrix, computedObjectiveFunctionValue);
      CommonOps.scale(0.5, computedObjectiveFunctionValue);
      CommonOps.multAddTransA(quadraticCostQVector, x, computedObjectiveFunctionValue);
      return computedObjectiveFunctionValue.get(0, 0) + quadraticCostScalar;
   }

   @Override
   public void setLinearEqualityConstraints(double[][] linearEqualityConstraintsAMatrix, double[] linearEqualityConstraintsBVector)
   {
      if (linearEqualityConstraintsAMatrix.length != linearEqualityConstraintsBVector.length)
         throw new RuntimeException();
      setLinearEqualityConstraints(new DenseMatrix64F(linearEqualityConstraintsAMatrix), MatrixTools.createVector(linearEqualityConstraintsBVector));
   }

   @Override
   public void setLinearEqualityConstraints(DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector)
   {
      this.linearEqualityConstraintsBVector.set(linearEqualityConstraintsBVector);
      this.linearEqualityConstraintsAMatrix.set(linearEqualityConstraintsAMatrix);
      //      this.linearEqualityConstraintsAMatrixTranspose.set(CommonOps.transpose(linearEqualityConstraintsAMatrix, null));
   }

   @Override
   public void setLinearInequalityConstraints(double[][] linearInequalityConstraintsCMatrix, double[] linearInqualityConstraintsDVector)
   {
      if (linearInequalityConstraintsCMatrix.length != linearInqualityConstraintsDVector.length)
         throw new RuntimeException();
      setLinearInequalityConstraints(new DenseMatrix64F(linearInequalityConstraintsCMatrix), MatrixTools.createVector(linearInqualityConstraintsDVector));
   }

   @Override
   public void setLinearInequalityConstraints(DenseMatrix64F linearInequalityConstraintCMatrix, DenseMatrix64F linearInequalityConstraintDVector)
   {
      this.linearInequalityConstraintsDVector.set(linearInequalityConstraintDVector);
      this.linearInequalityConstraintsCMatrix.set(linearInequalityConstraintCMatrix);
      //      this.linearInequalityConstraintsCMatrixTranspose.set(CommonOps.transpose(linearInequalityConstraintsCMatrix, null));
   }

   @Override
   public int solve(double[] solutionToPack, double[] lagrangeEqualityConstraintMultipliersToPack, double[] lagrangeInequalityConstraintMultipliersToPack)
   {
      int numberOfVariables = quadraticCostQMatrix.getNumCols();
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrix.getNumRows();

      if (solutionToPack.length != numberOfVariables)
         throw new RuntimeException("solutionToPack.length != numberOfVariables");
      if (lagrangeEqualityConstraintMultipliersToPack.length != numberOfEqualityConstraints)
         throw new RuntimeException("lagrangeEqualityConstraintMultipliersToPack.length != numberOfEqualityConstraints");
      if (lagrangeInequalityConstraintMultipliersToPack.length != numberOfInequalityConstraints)
         throw new RuntimeException("lagrangeInequalityConstraintMultipliersToPack.length != numberOfInequalityConstraints");

      DenseMatrix64F solution = new DenseMatrix64F(numberOfVariables, 1);
      DenseMatrix64F lagrangeEqualityConstraintMultipliers = new DenseMatrix64F(numberOfEqualityConstraints, 1);
      DenseMatrix64F lagrangeInequalityConstraintMultipliers = new DenseMatrix64F(numberOfInequalityConstraints, 1);

      int numberOfIterations = solve(solution, lagrangeEqualityConstraintMultipliers, lagrangeInequalityConstraintMultipliers);

      double[] solutionData = solution.getData();

      for (int i = 0; i < numberOfVariables; i++)
      {
         solutionToPack[i] = solutionData[i];
      }

      double[] lagrangeEqualityConstraintMultipliersData = lagrangeEqualityConstraintMultipliers.getData();
      double[] lagrangeInequalityConstraintMultipliersData = lagrangeInequalityConstraintMultipliers.getData();

      for (int i = 0; i < numberOfEqualityConstraints; i++)
      {
         lagrangeEqualityConstraintMultipliersToPack[i] = lagrangeEqualityConstraintMultipliersData[i];
      }

      for (int i = 0; i < numberOfInequalityConstraints; i++)
      {
         lagrangeInequalityConstraintMultipliersToPack[i] = lagrangeInequalityConstraintMultipliersData[i];
      }

      return numberOfIterations;
   }


   @Override
   public int solve(DenseMatrix64F solutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack, DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack)
   {
      int numberOfIterations = 1;

      int numberOfVariables = quadraticCostQMatrix.getNumRows();
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrix.getNumRows();

      lagrangeEqualityConstraintMultipliersToPack.reshape(numberOfEqualityConstraints, 1);
      lagrangeEqualityConstraintMultipliersToPack.zero();
      lagrangeInequalityConstraintMultipliersToPack.reshape(numberOfInequalityConstraints, 1);
      lagrangeInequalityConstraintMultipliersToPack.zero();
      
      augmentedLinearEqualityConstraintsAMatrix.set(linearEqualityConstraintsAMatrix);
      augmentedLinearEqualityConstraintsBVector.set(linearEqualityConstraintsBVector);
      
      computeQInverseAndAQInverse();
      CBar.reshape(0, 0);
      DBar.reshape(0, 0);
      activeSetIndices.clear();

      solveEqualityConstrainedSubproblem(solutionToPack, lagrangeEqualityConstraintMultipliersToPack);
      if (numberOfInequalityConstraints == 0)
         return numberOfIterations;

      // Test the inequality constraints:
      activeSetIndices.clear();

      for (int i = 0; i < maxNumberOfIterations - 1; i++)
      {
         boolean activeSetWasModified = modifyActiveSetAndTryAgain(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack);
         if (!activeSetWasModified)
            return numberOfIterations;
         numberOfIterations++;
      }

      return numberOfIterations;
   }

   private void computeQInverseAndAQInverse()
   {
      ATranspose.set(linearEqualityConstraintsAMatrix);
      CommonOps.transpose(ATranspose);
      QInverse.reshape(quadraticCostQMatrix.getNumCols(), quadraticCostQMatrix.getNumRows());
      CommonOps.invert(quadraticCostQMatrix, QInverse);
      
      AQInverse.reshape(linearEqualityConstraintsAMatrix.getNumRows(), QInverse.getNumCols());
      if (linearEqualityConstraintsAMatrix.getNumRows() > 0)
         CommonOps.mult(linearEqualityConstraintsAMatrix, QInverse, AQInverse);
      
      QInverseATranspose.reshape(QInverse.getNumRows(), ATranspose.getNumCols());
      if (ATranspose.getNumCols() > 0)
      CommonOps.mult(QInverse, ATranspose, QInverseATranspose);
      
      AQInverseATranspose.reshape(AQInverse.getNumRows(), ATranspose.getNumCols());
      if (ATranspose.getNumCols() > 0)
      CommonOps.mult(AQInverse, ATranspose, AQInverseATranspose);
   }
   
   private void computeCBarTempMatrices()
   {
      CBarTranspose.set(CBar);
      CommonOps.transpose(CBarTranspose);
      
      AQInverseCBarTranspose.reshape(AQInverse.getNumRows(), CBarTranspose.getNumCols());
      CommonOps.mult(AQInverse, CBarTranspose, AQInverseCBarTranspose);
      
      CBarQInverseATranspose.reshape(CBar.getNumRows(), QInverseATranspose.getNumCols());
      CommonOps.mult(CBar, QInverseATranspose, CBarQInverseATranspose);
      
      CBarQInverse.reshape(CBar.getNumRows(), QInverse.getNumCols());
      CommonOps.mult(CBar, QInverse, CBarQInverse);

      QInverseCBarTranspose.reshape(QInverse.getNumRows(), CBarTranspose.getNumCols());
      CommonOps.mult(QInverse, CBarTranspose, QInverseCBarTranspose);
      
      CBarQInverseCBarTranspose.reshape(CBar.getNumRows(), QInverseCBarTranspose.getNumCols());
      CommonOps.mult(CBar, QInverseCBarTranspose, CBarQInverseCBarTranspose); 
   }


   private boolean modifyActiveSetAndTryAgain(DenseMatrix64F solutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack, DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack)
   {
      if (containsNaN(solutionToPack)) return false;
      
      boolean activeSetWasModified = false;

      int numberOfVariables = quadraticCostQMatrix.getNumRows();
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrix.getNumRows();

      if (linearInequalityConstraintsCMatrix.getNumCols() != numberOfVariables)
         throw new RuntimeException();

      DenseMatrix64F linearInequalityConstraintsCheck = new DenseMatrix64F(numberOfInequalityConstraints, 1);
      CommonOps.mult(linearInequalityConstraintsCMatrix, solutionToPack, linearInequalityConstraintsCheck);
      CommonOps.subtractEquals(linearInequalityConstraintsCheck, linearInequalityConstraintsDVector);

      ArrayList<Integer> indicesToAddToActiveSet = new ArrayList<>();

      for (int i = 0; i < numberOfInequalityConstraints; i++)
      {
         if (activeSetIndices.contains(i))
            continue; // Only check violation on those that are not active. Otherwise check should just return 0.0, but roundoff could cause problems.
         if (linearInequalityConstraintsCheck.get(i, 0) > 0.0)
         {
            activeSetWasModified = true;
            indicesToAddToActiveSet.add(i);
         }
      }

      ArrayList<Integer> indicesToRemoveFromActiveSet = new ArrayList<>();
      for (int indexToCheck : activeSetIndices)
      {
         double lagrangeMultiplier = lagrangeInequalityConstraintMultipliersToPack.get(indexToCheck);
//         if ((lagrangeMultiplier < 0) || (Double.isInfinite(lagrangeMultiplier)))
         if (lagrangeMultiplier < 0)
         {
            activeSetWasModified = true;
            indicesToRemoveFromActiveSet.add(indexToCheck);
         }
      }

      if (!activeSetWasModified)
         return false;

      activeSetIndices.addAll(indicesToAddToActiveSet);
      activeSetIndices.removeAll(indicesToRemoveFromActiveSet);

      // Add active set constraints as equality constraints:
      int sizeOfActiveSet = activeSetIndices.size();

      augmentedLinearEqualityConstraintsAMatrix.reshape(numberOfEqualityConstraints + sizeOfActiveSet, numberOfVariables);
      augmentedLinearEqualityConstraintsBVector.reshape(numberOfEqualityConstraints + sizeOfActiveSet, 1);

      
      CBar.reshape(sizeOfActiveSet, numberOfVariables);
      DBar.reshape(sizeOfActiveSet, 1);


      CommonOps.insert(linearEqualityConstraintsAMatrix, augmentedLinearEqualityConstraintsAMatrix, 0, 0);
      CommonOps.insert(linearEqualityConstraintsBVector, augmentedLinearEqualityConstraintsBVector, 0, 0);

      for (int i = 0; i < sizeOfActiveSet; i++)
      {
         Integer inequalityConstraintIndex = activeSetIndices.get(i);
         CommonOps.extract(linearInequalityConstraintsCMatrix, inequalityConstraintIndex, inequalityConstraintIndex + 1, 0, numberOfVariables, augmentedLinearEqualityConstraintsAMatrix, numberOfEqualityConstraints + i,
               0);
         CommonOps.extract(linearInequalityConstraintsDVector, inequalityConstraintIndex, inequalityConstraintIndex + 1, 0, 1, augmentedLinearEqualityConstraintsBVector, numberOfEqualityConstraints + i, 0);
      
         
         CommonOps.extract(linearInequalityConstraintsCMatrix, inequalityConstraintIndex, inequalityConstraintIndex + 1, 0, numberOfVariables, CBar, i, 0);
         CommonOps.extract(linearInequalityConstraintsDVector, inequalityConstraintIndex, inequalityConstraintIndex + 1, 0, 1, DBar, i, 0);
      }

//      equalityConstrainedSolver.clear();
//      equalityConstrainedSolver.setQuadraticCostFunction(quadraticCostQMatrix, quadraticCostQVector, quadraticCostScalar);
//      equalityConstrainedSolver.setLinearEqualityConstraints(augmentedLinearEqualityConstraintsAMatrix, augmentedLinearEqualityConstraintsBVector);

//      DenseMatrix64F augmentedLagrangeEqualityConstraintMultipliers = new DenseMatrix64F(numberOfEqualityConstraints + sizeOfActiveSet, 1);
      solveEqualityConstrainedSubproblemEfficiently(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack);

      return true;
   }
   
   
   private boolean containsNaN(DenseMatrix64F solution)
   {
      for (int i=0; i<solution.getNumRows(); i++)
      {
         if (Double.isNaN(solution.get(i, 0))) return true;
      }
      
      return false;
   }


   private void solveEqualityConstrainedSubproblem(DenseMatrix64F xSolutionToPack, DenseMatrix64F lagrangeMultipliersToPack)
   {
      int numberOfVariables = quadraticCostQMatrix.getNumCols();
      if (numberOfVariables != quadraticCostQMatrix.getNumRows())
         throw new RuntimeException("numCols != numRows");

      int numberOfEqualityConstraints = augmentedLinearEqualityConstraintsAMatrix.getNumRows();
      if (numberOfEqualityConstraints > 0)
      {
         if (augmentedLinearEqualityConstraintsAMatrix.getNumCols() != numberOfVariables)
            throw new RuntimeException("linearEqualityConstraintA.getNumCols() != numberOfVariables");
      }

      if (quadraticCostQVector.getNumRows() != numberOfVariables)
         throw new RuntimeException("quadraticCostQVector.getNumRows() != numRows");
      if (quadraticCostQVector.getNumCols() != 1)
         throw new RuntimeException("quadraticCostQVector.getNumCols() != 1");

      DenseMatrix64F negativeQuadraticCostQVector = new DenseMatrix64F(quadraticCostQVector);
      CommonOps.scale(-1.0, negativeQuadraticCostQVector);

      if (numberOfEqualityConstraints == 0)
      {
         xSolutionToPack.reshape(numberOfVariables, 1);
         CommonOps.solve(quadraticCostQMatrix, negativeQuadraticCostQVector, xSolutionToPack);
         return;
      }

      linearEqualityConstraintsAMatrixTranspose.set(augmentedLinearEqualityConstraintsAMatrix);
      CommonOps.transpose(linearEqualityConstraintsAMatrixTranspose);
      DenseMatrix64F bigMatrix = new DenseMatrix64F(numberOfVariables + numberOfEqualityConstraints, numberOfVariables + numberOfEqualityConstraints);
      DenseMatrix64F bigVector = new DenseMatrix64F(numberOfVariables + numberOfEqualityConstraints, 1);

      CommonOps.insert(quadraticCostQMatrix, bigMatrix, 0, 0);
      CommonOps.insert(augmentedLinearEqualityConstraintsAMatrix, bigMatrix, numberOfVariables, 0);
      CommonOps.insert(linearEqualityConstraintsAMatrixTranspose, bigMatrix, 0, numberOfVariables);

      CommonOps.insert(negativeQuadraticCostQVector, bigVector, 0, 0);
      CommonOps.insert(augmentedLinearEqualityConstraintsBVector, bigVector, numberOfVariables, 0);

      DenseMatrix64F xAndLagrangeMultiplierSolution = new DenseMatrix64F(numberOfVariables + numberOfEqualityConstraints, 1);
      CommonOps.solve(bigMatrix, bigVector, xAndLagrangeMultiplierSolution);

      for (int i = 0; i < numberOfVariables; i++)
      {
         xSolutionToPack.set(i, 0, xAndLagrangeMultiplierSolution.get(i, 0));
      }

      for (int i = 0; i < numberOfEqualityConstraints; i++)
      {
         lagrangeMultipliersToPack.set(i, 0, xAndLagrangeMultiplierSolution.get(numberOfVariables + i, 0));
      }
   }
   
   
   private void solveEqualityConstrainedSubproblemEfficiently(DenseMatrix64F xSolutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack, DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack)
   {
      int numberOfVariables = quadraticCostQMatrix.getNumCols();
      if (numberOfVariables != quadraticCostQMatrix.getNumRows())
         throw new RuntimeException("numCols != numRows");

      int numberOfAugmentedEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows() + CBar.getNumRows();
      int numberOfActiveSetConstraints = CBar.getNumRows();

//      int numberOfEqualityConstraints = augmentedLinearEqualityConstraintsAMatrix.getNumRows();

//      if (numberOfEqualityConstraints > 0)
//      {
//         if (augmentedLinearEqualityConstraintsAMatrix.getNumCols() != numberOfVariables)
//            throw new RuntimeException("linearEqualityConstraintA.getNumCols() != numberOfVariables");
//      }

      if (quadraticCostQVector.getNumRows() != numberOfVariables)
         throw new RuntimeException("quadraticCostQVector.getNumRows() != numRows");
      if (quadraticCostQVector.getNumCols() != 1)
         throw new RuntimeException("quadraticCostQVector.getNumCols() != 1");

      DenseMatrix64F negativeQuadraticCostQVector = new DenseMatrix64F(quadraticCostQVector);
      CommonOps.scale(-1.0, negativeQuadraticCostQVector);

      if (numberOfAugmentedEqualityConstraints == 0)
      {
         xSolutionToPack.reshape(numberOfVariables, 1);  
         CommonOps.mult(QInverse, negativeQuadraticCostQVector, xSolutionToPack);
//         CommonOps.solve(quadraticCostQMatrix, negativeQuadraticCostQVector, xSolutionToPack);
         return;
      }

      computeCBarTempMatrices();

      
      int numberOfOriginalEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      DenseMatrix64F bigMatrixForLagrangeMultipliers = new DenseMatrix64F(numberOfOriginalEqualityConstraints + numberOfActiveSetConstraints, numberOfOriginalEqualityConstraints + numberOfActiveSetConstraints);
      
      CommonOps.insert(AQInverseATranspose, bigMatrixForLagrangeMultipliers, 0, 0);
      CommonOps.insert(AQInverseCBarTranspose, bigMatrixForLagrangeMultipliers, 0, numberOfOriginalEqualityConstraints);
      CommonOps.insert(CBarQInverseATranspose, bigMatrixForLagrangeMultipliers, numberOfOriginalEqualityConstraints, 0);
      CommonOps.insert(CBarQInverseCBarTranspose, bigMatrixForLagrangeMultipliers, numberOfOriginalEqualityConstraints, numberOfOriginalEqualityConstraints);
      
      DenseMatrix64F bigVectorForLagrangeMultipliers = new DenseMatrix64F(numberOfOriginalEqualityConstraints + numberOfActiveSetConstraints, 1);
      
      if (numberOfOriginalEqualityConstraints > 0)
      {
//         System.out.println("Fooy");
//         System.out.println(numberOfEqualityConstraints);
//         System.out.println(quadraticCostQMatrix);
//         System.out.println(linearEqualityConstraintsAMatrix);
//         System.out.println(QInverse);
//         System.out.println(AQInverse);
         
         
         DenseMatrix64F tempVector = new DenseMatrix64F(numberOfOriginalEqualityConstraints, 1);
         CommonOps.mult(AQInverse, quadraticCostQVector, tempVector);
         CommonOps.addEquals(tempVector, linearEqualityConstraintsBVector);
         CommonOps.scale(-1.0, tempVector);


         CommonOps.insert(tempVector, bigVectorForLagrangeMultipliers, 0, 0);
      }
      
      
      if (numberOfActiveSetConstraints > 0)
      {
         DenseMatrix64F tempVector = new DenseMatrix64F(numberOfActiveSetConstraints, 1);
         CommonOps.mult(CBarQInverse, quadraticCostQVector, tempVector);
         CommonOps.addEquals(tempVector, DBar);
         CommonOps.scale(-1.0, tempVector);

         CommonOps.insert(tempVector, bigVectorForLagrangeMultipliers, numberOfOriginalEqualityConstraints, 0);
      }
      
      
      DenseMatrix64F lagrangeMultipliersToPack = new DenseMatrix64F(numberOfOriginalEqualityConstraints + numberOfActiveSetConstraints, 1);

      
      DenseMatrix64F lagrangeMultipliers = new DenseMatrix64F(numberOfOriginalEqualityConstraints + numberOfActiveSetConstraints, 1);
      CommonOps.solve(bigMatrixForLagrangeMultipliers, bigVectorForLagrangeMultipliers, lagrangeMultipliers);
      lagrangeMultipliersToPack.set(lagrangeMultipliers);

      
      
      
      
      DenseMatrix64F ATransposeAndCTranspose = new DenseMatrix64F(numberOfVariables, numberOfOriginalEqualityConstraints + numberOfActiveSetConstraints);
      CommonOps.insert(ATranspose, ATransposeAndCTranspose, 0, 0);
      CommonOps.insert(CBarTranspose, ATransposeAndCTranspose, 0, numberOfOriginalEqualityConstraints);

      
      DenseMatrix64F ATransposeMuAndCTransposeLambda = new DenseMatrix64F(numberOfVariables, 1);
      CommonOps.mult(ATransposeAndCTranspose, lagrangeMultipliers, ATransposeMuAndCTransposeLambda);
      
      
      DenseMatrix64F tempVector = new DenseMatrix64F(quadraticCostQVector);
      CommonOps.scale(-1.0, tempVector);
      CommonOps.subtractEquals(tempVector, ATransposeMuAndCTransposeLambda);
      
      
      CommonOps.mult(QInverse, tempVector, xSolutionToPack);

      
      
      System.out.println("******");
      System.out.println("numberOfEqualityConstraints = " + numberOfAugmentedEqualityConstraints);
      System.out.println("numberOfOriginalEqualityConstraints = " + numberOfOriginalEqualityConstraints);
      System.out.println("lagrangeMultipliersToPack = " + lagrangeMultipliers);
      
      
//      DenseMatrix64F lagrangeMultipliersForChecking = new DenseMatrix64F(numberOfAugmentedEqualityConstraints, 1);
//      DenseMatrix64F xSolutionForChecking = new DenseMatrix64F(numberOfVariables, 1);
//      
//      linearEqualityConstraintsAMatrixTranspose.set(augmentedLinearEqualityConstraintsAMatrix);
//      CommonOps.transpose(linearEqualityConstraintsAMatrixTranspose);
//      DenseMatrix64F bigMatrix = new DenseMatrix64F(numberOfVariables + numberOfAugmentedEqualityConstraints, numberOfVariables + numberOfAugmentedEqualityConstraints);
//      DenseMatrix64F bigVector = new DenseMatrix64F(numberOfVariables + numberOfAugmentedEqualityConstraints, 1);
//
//      CommonOps.insert(quadraticCostQMatrix, bigMatrix, 0, 0);
//      CommonOps.insert(augmentedLinearEqualityConstraintsAMatrix, bigMatrix, numberOfVariables, 0);
//      CommonOps.insert(linearEqualityConstraintsAMatrixTranspose, bigMatrix, 0, numberOfVariables);
//
//      CommonOps.insert(negativeQuadraticCostQVector, bigVector, 0, 0);
//      CommonOps.insert(augmentedLinearEqualityConstraintsBVector, bigVector, numberOfVariables, 0);
//
//      DenseMatrix64F xAndLagrangeMultiplierSolution = new DenseMatrix64F(numberOfVariables + numberOfAugmentedEqualityConstraints, 1);
//      CommonOps.solve(bigMatrix, bigVector, xAndLagrangeMultiplierSolution);
//
//      for (int i = 0; i < numberOfVariables; i++)
//      {
//         xSolutionForChecking.set(i, 0, xAndLagrangeMultiplierSolution.get(i, 0));
//      }
//
//      for (int i = 0; i < numberOfAugmentedEqualityConstraints; i++)
//      {
//         lagrangeMultipliersForChecking.set(i, 0, xAndLagrangeMultiplierSolution.get(numberOfVariables + i, 0));
//         lagrangeMultipliersToPack.set(i, 0, xAndLagrangeMultiplierSolution.get(numberOfVariables + i, 0));
//      }
//      
//      System.out.println("lagrangeMultipliersForChecking = " + lagrangeMultipliersForChecking);
      
      
      
      
      lagrangeEqualityConstraintMultipliersToPack.reshape(numberOfOriginalEqualityConstraints, 1);
      lagrangeEqualityConstraintMultipliersToPack.zero();
      if (numberOfOriginalEqualityConstraints > 0)
      {
         CommonOps.extract(lagrangeMultipliersToPack, 0, numberOfOriginalEqualityConstraints, 0, 1, lagrangeEqualityConstraintMultipliersToPack, 0, 0);
      }

      lagrangeInequalityConstraintMultipliersToPack.zero();
      for (int i = 0; i < numberOfActiveSetConstraints; i++)
      {
         Integer inequalityConstraintIndex = activeSetIndices.get(i);
         CommonOps.extract(lagrangeMultipliersToPack, numberOfOriginalEqualityConstraints + i, numberOfOriginalEqualityConstraints + i + 1, 0, 1, lagrangeInequalityConstraintMultipliersToPack,
               inequalityConstraintIndex, 0);
      }

   }

}
