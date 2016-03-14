package us.ihmc.convexOptimization.quadraticProgram;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class SimpleInefficientActiveSetQPSolver
{
   private final SimpleInefficientEqualityConstrainedQPSolver equalityConstrainedSolver = new SimpleInefficientEqualityConstrainedQPSolver();
   
   private final DenseMatrix64F quadraticCostQMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F quadraticCostQVector = new DenseMatrix64F(0, 0);
   private double quadraticCostScalar;
   
   private final DenseMatrix64F linearEqualityConstraintsAMatrix = new DenseMatrix64F(0, 0);
//   private final DenseMatrix64F linearEqualityConstraintsAMatrixTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F linearEqualityConstraintsBVector = new DenseMatrix64F(0, 0);
   
   private final DenseMatrix64F linearInequalityConstraintsCMatrix = new DenseMatrix64F(0, 0);
//   private final DenseMatrix64F linearInequalityConstraintsCMatrixTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F linearInequalityConstraintsDVector = new DenseMatrix64F(0, 0);
   
   private final DenseMatrix64F augmentedLinearEqualityConstraintsAMatrix = new DenseMatrix64F(0, 0);
//   private final DenseMatrix64F augmentedLinearEqualityConstraintsAMatrixTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F augmentedLinearEqualityConstraintsBVector = new DenseMatrix64F(0, 0);
   
   private final ArrayList<Integer> activeSetIndices = new ArrayList<>();
   
   public void clear()
   {
      equalityConstrainedSolver.clear();
      
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
      
      activeSetIndices.clear();
   }
   
    
   public void setQuadraticCostFunction(double[][] quadraticCostFunctionWMatrix, double[] quadraticCostFunctionGVector, double quadraticCostScalar)
   {
      setQuadraticCostFunction(new DenseMatrix64F(quadraticCostFunctionWMatrix), MatrixTools.createVector(quadraticCostFunctionGVector), quadraticCostScalar);
   }
   
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

   public double getObjectiveCost(DenseMatrix64F solutionMatrix)
   {
      return equalityConstrainedSolver.getObjectiveCost(solutionMatrix);
   }

   public void setLinearEqualityConstraints(double[][] linearEqualityConstraintsAMatrix, double[] linearEqualityConstraintsBVector)
   {
      if (linearEqualityConstraintsAMatrix.length != linearEqualityConstraintsBVector.length) throw new RuntimeException();
      setLinearEqualityConstraints(new DenseMatrix64F(linearEqualityConstraintsAMatrix), MatrixTools.createVector(linearEqualityConstraintsBVector));
   }

   public void setLinearEqualityConstraints(DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector)
   {
      this.linearEqualityConstraintsBVector.set(linearEqualityConstraintsBVector);
      this.linearEqualityConstraintsAMatrix.set(linearEqualityConstraintsAMatrix);
//      this.linearEqualityConstraintsAMatrixTranspose.set(CommonOps.transpose(linearEqualityConstraintsAMatrix, null));
   }

   public void setLinearInequalityConstraints(double[][] linearInequalityConstraintsCMatrix, double[] linearInqualityConstraintsDVector)
   {
      if (linearInequalityConstraintsCMatrix.length != linearInqualityConstraintsDVector.length) throw new RuntimeException();
      setLinearInequalityConstraints(new DenseMatrix64F(linearInequalityConstraintsCMatrix), MatrixTools.createVector(linearInqualityConstraintsDVector));  
   }
   
   
   public void setLinearInequalityConstraints(DenseMatrix64F linearInequalityConstraintCMatrix, DenseMatrix64F linearInequalityConstraintDVector)
   {
      this.linearInequalityConstraintsDVector.set(linearInequalityConstraintDVector);
      this.linearInequalityConstraintsCMatrix.set(linearInequalityConstraintCMatrix);
//      this.linearInequalityConstraintsCMatrixTranspose.set(CommonOps.transpose(linearInequalityConstraintsCMatrix, null));
   }
   
   
   public void solve(double[] solutionToPack, double[] lagrangeEqualityConstraintMultipliersToPack, double[] lagrangeInequalityConstraintMultipliersToPack)
   {
      int numberOfVariables = quadraticCostQMatrix.getNumCols();      
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrix.getNumRows();
      
      if (solutionToPack.length != numberOfVariables) throw new RuntimeException("solutionToPack.length != numberOfVariables");
      if (lagrangeEqualityConstraintMultipliersToPack.length != numberOfEqualityConstraints) throw new RuntimeException("lagrangeEqualityConstraintMultipliersToPack.length != numberOfEqualityConstraints");
      if (lagrangeInequalityConstraintMultipliersToPack.length != numberOfInequalityConstraints) throw new RuntimeException("lagrangeInequalityConstraintMultipliersToPack.length != numberOfInequalityConstraints");

      
      DenseMatrix64F solution = new DenseMatrix64F(numberOfVariables, 1);
      DenseMatrix64F lagrangeEqualityConstraintMultipliers = new DenseMatrix64F(numberOfEqualityConstraints, 1);
      DenseMatrix64F lagrangeInequalityConstraintMultipliers = new DenseMatrix64F(numberOfInequalityConstraints, 1);
    
      solve(solution, lagrangeEqualityConstraintMultipliers, lagrangeInequalityConstraintMultipliers);
      
      double[] solutionData = solution.getData();
      
      for (int i=0; i<numberOfVariables; i++)
      {
         solutionToPack[i] = solutionData[i];
      }
      
      double[] lagrangeEqualityConstraintMultipliersData = lagrangeEqualityConstraintMultipliers.getData();
      double[] lagrangeInequalityConstraintMultipliersData = lagrangeInequalityConstraintMultipliers.getData();
      
      for (int i=0; i<numberOfEqualityConstraints; i++)
      {
         lagrangeEqualityConstraintMultipliersToPack[i] = lagrangeEqualityConstraintMultipliersData[i];
      }
      
      for (int i=0; i<numberOfInequalityConstraints; i++)
      {
         lagrangeInequalityConstraintMultipliersToPack[i] = lagrangeInequalityConstraintMultipliersData[i];
      }
   }
   
   public void solve(DenseMatrix64F solutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack, DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack)
   {
      int numberOfVariables = quadraticCostQMatrix.getNumRows();
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrix.getNumRows();
      
      equalityConstrainedSolver.setQuadraticCostFunction(quadraticCostQMatrix, quadraticCostQVector, quadraticCostScalar);
      
      if (linearEqualityConstraintsAMatrix.getNumRows() > 0)
         equalityConstrainedSolver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      equalityConstrainedSolver.solve(solutionToPack, lagrangeEqualityConstraintMultipliersToPack);
      
      lagrangeInequalityConstraintMultipliersToPack.reshape(numberOfInequalityConstraints, 1);
      lagrangeInequalityConstraintMultipliersToPack.zero();
      
      if (numberOfInequalityConstraints == 0) return;
      
      // Test the inequality constraints:

      if (linearInequalityConstraintsCMatrix.getNumCols() != numberOfVariables) throw new RuntimeException();
      
      DenseMatrix64F linearInequalityConstraintsCheck = new DenseMatrix64F(numberOfInequalityConstraints, 1);
      CommonOps.mult(linearInequalityConstraintsCMatrix, solutionToPack, linearInequalityConstraintsCheck);
      CommonOps.subtractEquals(linearInequalityConstraintsCheck, linearInequalityConstraintsDVector);

      activeSetIndices.clear();

      for (int i = 0; i<numberOfInequalityConstraints; i++)
      {
         if (linearInequalityConstraintsCheck.get(i, 0) > 0.0)
         {
            activeSetIndices.add(i);
         }
      }

      if (activeSetIndices.isEmpty()) return;

      // Add violated inequality constraints as equality constraints:
      int numberOfViolations = activeSetIndices.size();

      augmentedLinearEqualityConstraintsAMatrix.reshape(numberOfEqualityConstraints + numberOfViolations, numberOfVariables);
      augmentedLinearEqualityConstraintsBVector.reshape(numberOfEqualityConstraints + numberOfViolations, 1);

      CommonOps.insert(linearEqualityConstraintsAMatrix, augmentedLinearEqualityConstraintsAMatrix, 0, 0);
      CommonOps.insert(linearEqualityConstraintsBVector, augmentedLinearEqualityConstraintsBVector, 0, 0);

      for (int i=0; i<numberOfViolations; i++)
      {
         Integer inequalityConstraintIndex = activeSetIndices.get(i);
         CommonOps.extract(linearInequalityConstraintsCMatrix, inequalityConstraintIndex, inequalityConstraintIndex+1, 0, numberOfVariables, augmentedLinearEqualityConstraintsAMatrix, numberOfEqualityConstraints + i, 0);
         CommonOps.extract(linearInequalityConstraintsDVector, inequalityConstraintIndex, inequalityConstraintIndex+1, 0, 1, augmentedLinearEqualityConstraintsBVector, numberOfEqualityConstraints + i, 0);
      }

      equalityConstrainedSolver.clear();
      equalityConstrainedSolver.setQuadraticCostFunction(quadraticCostQMatrix, quadraticCostQVector, quadraticCostScalar);
      equalityConstrainedSolver.setLinearEqualityConstraints(augmentedLinearEqualityConstraintsAMatrix, augmentedLinearEqualityConstraintsBVector);
      
      DenseMatrix64F augmentedLagrangeEqualityConstraintMultipliers = new DenseMatrix64F(numberOfEqualityConstraints + numberOfViolations, 1);
      equalityConstrainedSolver.solve(solutionToPack, augmentedLagrangeEqualityConstraintMultipliers);

      lagrangeEqualityConstraintMultipliersToPack.reshape(numberOfEqualityConstraints, 1);
      if (numberOfEqualityConstraints > 0)
      {
         CommonOps.extract(augmentedLagrangeEqualityConstraintMultipliers, 0, numberOfEqualityConstraints, 0, 1, lagrangeEqualityConstraintMultipliersToPack, 0, 0);
      }
      
      for (int i=0; i<numberOfViolations; i++)
      {
         Integer inequalityConstraintIndex = activeSetIndices.get(i);
         CommonOps.extract(augmentedLagrangeEqualityConstraintMultipliers, numberOfEqualityConstraints + i, numberOfEqualityConstraints + i+1, 0, 1, lagrangeInequalityConstraintMultipliersToPack, inequalityConstraintIndex, 0);
      }
   }


}
