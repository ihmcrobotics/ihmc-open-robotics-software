package us.ihmc.convexOptimization.quadraticProgram;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
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

   private final ArrayList<Integer> activeSetIndices = new ArrayList<>();

   // Some temporary matrices:
   private final DenseMatrix64F symmetricCostQuadraticMatrix = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F negativeQuadraticCostQVector = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F linearInequalityConstraintsCheck = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F CBar = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F DBar = new DenseMatrix64F(0, 0);

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

   private final DenseMatrix64F ATransposeAndCTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F ATransposeMuAndCTransposeLambda = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F bigMatrixForLagrangeMultiplierSolution = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F bigVectorForLagrangeMultiplierSolution = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F tempVector = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F augmentedLagrangeMultipliers = new DenseMatrix64F(0, 0);

   private final ArrayList<Integer> indicesToAddToActiveSet = new ArrayList<>();
   private final ArrayList<Integer> indicesToRemoveFromActiveSet = new ArrayList<>();

   private final DenseMatrix64F computedObjectiveFunctionValue = new DenseMatrix64F(1, 1);

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);
   @Override
   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      this.maxNumberOfIterations = maxNumberOfIterations;
   }

   @Override
   public void clear()
   {
      quadraticCostQMatrix.reshape(0, 0);
      quadraticCostQVector.reshape(0, 0);

      linearEqualityConstraintsAMatrix.reshape(0, 0);
      linearEqualityConstraintsBVector.reshape(0, 0);

      linearInequalityConstraintsCMatrix.reshape(0, 0);
      linearInequalityConstraintsDVector.reshape(0, 0);

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
      symmetricCostQuadraticMatrix.reshape(costQuadraticMatrix.getNumCols(), costQuadraticMatrix.getNumRows());
      CommonOps.transpose(costQuadraticMatrix, symmetricCostQuadraticMatrix);
      
      CommonOps.add(costQuadraticMatrix, symmetricCostQuadraticMatrix, symmetricCostQuadraticMatrix);
      CommonOps.scale(0.5, symmetricCostQuadraticMatrix);
      this.quadraticCostQMatrix.set(symmetricCostQuadraticMatrix);
      this.quadraticCostQVector.set(costLinearVector);
      this.quadraticCostScalar = quadraticCostScalar;
   }

   @Override
   public double getObjectiveCost(DenseMatrix64F x)
   {
      multQuad(x, quadraticCostQMatrix, computedObjectiveFunctionValue);
      CommonOps.scale(0.5, computedObjectiveFunctionValue);
      CommonOps.multAddTransA(quadraticCostQVector, x, computedObjectiveFunctionValue);
      return computedObjectiveFunctionValue.get(0, 0) + quadraticCostScalar;
   }
   
   private final DenseMatrix64F temporaryMatrix = new DenseMatrix64F(0, 0);

   private void multQuad(DenseMatrix64F xVector, DenseMatrix64F QMatrix, DenseMatrix64F xTransposeQx)
   {
      temporaryMatrix.reshape(xVector.numCols, QMatrix.numCols);
      CommonOps.multTransA(xVector, QMatrix, temporaryMatrix);
      CommonOps.mult(temporaryMatrix, xVector, xTransposeQx);
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
   public int solve(double[] solutionToPack)
   {
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrix.getNumRows();
      
      double[] lagrangeEqualityConstraintMultipliersToPack = new double[numberOfEqualityConstraints];
      double[] lagrangeInequalityConstraintMultipliersToPack = new double[numberOfInequalityConstraints];
      
      return solve(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack);
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

   private final DenseMatrix64F lagrangeEqualityConstraintMultipliersToThrowAway = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F lagrangeInequalityConstraintMultipliersToThrowAway = new DenseMatrix64F(0, 0);
   
   @Override
   public int solve(DenseMatrix64F solutionToPack)
   {
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrix.getNumRows();
      
      lagrangeEqualityConstraintMultipliersToThrowAway.reshape(numberOfEqualityConstraints, 1);
      lagrangeInequalityConstraintMultipliersToThrowAway.reshape(numberOfInequalityConstraints, 1);
      
      return solve(solutionToPack, lagrangeEqualityConstraintMultipliersToThrowAway, lagrangeInequalityConstraintMultipliersToThrowAway);
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

      computeQInverseAndAQInverse();
      CBar.reshape(0, 0);
      DBar.reshape(0, 0);
      activeSetIndices.clear();

      solveEqualityConstrainedSubproblemEfficiently(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack);
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
      int numberOfVariables = quadraticCostQMatrix.getNumRows();
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();

      ATranspose.reshape(linearEqualityConstraintsAMatrix.getNumCols(), linearEqualityConstraintsAMatrix.getNumRows());
      CommonOps.transpose(linearEqualityConstraintsAMatrix, ATranspose);
      QInverse.reshape(numberOfVariables, numberOfVariables);
      
      solver.setA(quadraticCostQMatrix);
      solver.invert(QInverse);

      AQInverse.reshape(numberOfEqualityConstraints, numberOfVariables);
      QInverseATranspose.reshape(numberOfVariables, numberOfEqualityConstraints);
      AQInverseATranspose.reshape(numberOfEqualityConstraints, numberOfEqualityConstraints);

      if (numberOfEqualityConstraints > 0)
      {
         CommonOps.mult(linearEqualityConstraintsAMatrix, QInverse, AQInverse);
         CommonOps.mult(QInverse, ATranspose, QInverseATranspose);
         CommonOps.mult(AQInverse, ATranspose, AQInverseATranspose);
      }

   }

   private void computeCBarTempMatrices()
   {
      if (CBar.getNumRows() > 0)
      {
         CBarTranspose.reshape(CBar.getNumCols(), CBar.getNumRows());
         CommonOps.transpose(CBar, CBarTranspose);

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
      else
      {
         CBarTranspose.reshape(0, 0);
         AQInverseCBarTranspose.reshape(0, 0);
         CBarQInverseATranspose.reshape(0, 0);
         CBarQInverse.reshape(0, 0);
         QInverseCBarTranspose.reshape(0, 0);
         CBarQInverseCBarTranspose.reshape(0, 0);
      }
   }

   private boolean modifyActiveSetAndTryAgain(DenseMatrix64F solutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack, DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack)
   {
      if (containsNaN(solutionToPack))
         return false;

      boolean activeSetWasModified = false;

      int numberOfVariables = quadraticCostQMatrix.getNumRows();
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrix.getNumRows();

      if (linearInequalityConstraintsCMatrix.getNumCols() != numberOfVariables)
         throw new RuntimeException();

      linearInequalityConstraintsCheck.reshape(numberOfInequalityConstraints, 1);
      CommonOps.mult(linearInequalityConstraintsCMatrix, solutionToPack, linearInequalityConstraintsCheck);
      CommonOps.subtractEquals(linearInequalityConstraintsCheck, linearInequalityConstraintsDVector);

      indicesToAddToActiveSet.clear();

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

      indicesToRemoveFromActiveSet.clear();
      for (int i=0; i < activeSetIndices.size(); i++)
      {
         int indexToCheck = activeSetIndices.get(i);

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

      CBar.reshape(sizeOfActiveSet, numberOfVariables);
      DBar.reshape(sizeOfActiveSet, 1);

      for (int i = 0; i < sizeOfActiveSet; i++)
      {
         Integer inequalityConstraintIndex = activeSetIndices.get(i);
         CommonOps.extract(linearInequalityConstraintsCMatrix, inequalityConstraintIndex, inequalityConstraintIndex + 1, 0, numberOfVariables, CBar, i, 0);
         CommonOps.extract(linearInequalityConstraintsDVector, inequalityConstraintIndex, inequalityConstraintIndex + 1, 0, 1, DBar, i, 0);
      }

      solveEqualityConstrainedSubproblemEfficiently(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack);

      return true;
   }

   private boolean containsNaN(DenseMatrix64F solution)
   {
      for (int i = 0; i < solution.getNumRows(); i++)
      {
         if (Double.isNaN(solution.get(i, 0)))
            return true;
      }

      return false;
   }

   private void solveEqualityConstrainedSubproblemEfficiently(DenseMatrix64F xSolutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack, DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack)
   {
      int numberOfVariables = quadraticCostQMatrix.getNumCols();
      if (numberOfVariables != quadraticCostQMatrix.getNumRows())
         throw new RuntimeException("numCols != numRows");
      int numberOfOriginalEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();

      int numberOfActiveSetConstraints = CBar.getNumRows();
      int numberOfAugmentedEqualityConstraints = numberOfOriginalEqualityConstraints + numberOfActiveSetConstraints;

      if (quadraticCostQVector.getNumRows() != numberOfVariables)
         throw new RuntimeException("quadraticCostQVector.getNumRows() != numRows");
      if (quadraticCostQVector.getNumCols() != 1)
         throw new RuntimeException("quadraticCostQVector.getNumCols() != 1");

      negativeQuadraticCostQVector.set(quadraticCostQVector);
      CommonOps.scale(-1.0, negativeQuadraticCostQVector);

      if (numberOfAugmentedEqualityConstraints == 0)
      {
         xSolutionToPack.reshape(numberOfVariables, 1);
         CommonOps.mult(QInverse, negativeQuadraticCostQVector, xSolutionToPack);
         //         CommonOps.solve(quadraticCostQMatrix, negativeQuadraticCostQVector, xSolutionToPack);
         return;
      }

      computeCBarTempMatrices();

      bigMatrixForLagrangeMultiplierSolution.reshape(numberOfAugmentedEqualityConstraints, numberOfAugmentedEqualityConstraints);
      bigVectorForLagrangeMultiplierSolution.reshape(numberOfAugmentedEqualityConstraints, 1);

      CommonOps.insert(AQInverseATranspose, bigMatrixForLagrangeMultiplierSolution, 0, 0);
      CommonOps.insert(AQInverseCBarTranspose, bigMatrixForLagrangeMultiplierSolution, 0, numberOfOriginalEqualityConstraints);
      CommonOps.insert(CBarQInverseATranspose, bigMatrixForLagrangeMultiplierSolution, numberOfOriginalEqualityConstraints, 0);
      CommonOps.insert(CBarQInverseCBarTranspose, bigMatrixForLagrangeMultiplierSolution, numberOfOriginalEqualityConstraints, numberOfOriginalEqualityConstraints);

      if (numberOfOriginalEqualityConstraints > 0)
      {
         tempVector.reshape(numberOfOriginalEqualityConstraints, 1);
         CommonOps.mult(AQInverse, quadraticCostQVector, tempVector);
         CommonOps.addEquals(tempVector, linearEqualityConstraintsBVector);
         CommonOps.scale(-1.0, tempVector);

         CommonOps.insert(tempVector, bigVectorForLagrangeMultiplierSolution, 0, 0);
      }

      if (numberOfActiveSetConstraints > 0)
      {
         tempVector.reshape(numberOfActiveSetConstraints, 1);
         CommonOps.mult(CBarQInverse, quadraticCostQVector, tempVector);
         CommonOps.addEquals(tempVector, DBar);
         CommonOps.scale(-1.0, tempVector);

         CommonOps.insert(tempVector, bigVectorForLagrangeMultiplierSolution, numberOfOriginalEqualityConstraints, 0);
      }

      augmentedLagrangeMultipliers.reshape(numberOfAugmentedEqualityConstraints, 1);
      solver.setA(bigMatrixForLagrangeMultiplierSolution);
      solver.solve(bigVectorForLagrangeMultiplierSolution, augmentedLagrangeMultipliers);

      ATransposeAndCTranspose.reshape(numberOfVariables, numberOfAugmentedEqualityConstraints);
      CommonOps.insert(ATranspose, ATransposeAndCTranspose, 0, 0);
      CommonOps.insert(CBarTranspose, ATransposeAndCTranspose, 0, numberOfOriginalEqualityConstraints);

      ATransposeMuAndCTransposeLambda.reshape(numberOfVariables, 1);
      CommonOps.mult(ATransposeAndCTranspose, augmentedLagrangeMultipliers, ATransposeMuAndCTransposeLambda);

      tempVector.set(quadraticCostQVector);
      CommonOps.scale(-1.0, tempVector);
      CommonOps.subtractEquals(tempVector, ATransposeMuAndCTransposeLambda);

      CommonOps.mult(QInverse, tempVector, xSolutionToPack);

      lagrangeEqualityConstraintMultipliersToPack.reshape(numberOfOriginalEqualityConstraints, 1);
      lagrangeEqualityConstraintMultipliersToPack.zero();
      if (numberOfOriginalEqualityConstraints > 0)
      {
         CommonOps.extract(augmentedLagrangeMultipliers, 0, numberOfOriginalEqualityConstraints, 0, 1, lagrangeEqualityConstraintMultipliersToPack, 0, 0);
      }

      lagrangeInequalityConstraintMultipliersToPack.zero();
      for (int i = 0; i < numberOfActiveSetConstraints; i++)
      {
         Integer inequalityConstraintIndex = activeSetIndices.get(i);
         CommonOps.extract(augmentedLagrangeMultipliers, numberOfOriginalEqualityConstraints + i, numberOfOriginalEqualityConstraints + i + 1, 0, 1, lagrangeInequalityConstraintMultipliersToPack,
               inequalityConstraintIndex, 0);
      }

   }

}
