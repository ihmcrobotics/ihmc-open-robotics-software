package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class JavaQuadProgSolverWithInactiveVariables extends JavaQuadProgSolver implements  ActiveSetQPSolverWithInactiveVariablesInterface
{
   private final DenseMatrix64F originalQuadraticCostQMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F originalQuadraticCostQVector = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F originalLinearEqualityConstraintsAMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F originalLinearEqualityConstraintsBVector = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F originalLinearInequalityConstraintsCMatrixO = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F originalLinearInequalityConstraintsDVectorO = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F originalVariableLowerBounds = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F originalVariableUpperBounds = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F activeVariables = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F activeVariableSolution = new DenseMatrix64F(0, 0);

   @Override
   public void setActiveVariables(DenseMatrix64F activeVariables)
   {
      if (activeVariables.getNumRows() != originalQuadraticCostQMatrix.getNumRows())
         throw new RuntimeException("activeVariables.getNumRows() != quadraticCostQMatrix.getNumRows()");

      this.activeVariables.set(activeVariables);
   }

   @Override
   public void setVariableActive(int variableIndex)
   {
      if (variableIndex >= activeVariables.getNumRows())
         throw new RuntimeException("variable index is outside the number of variables.");

      activeVariables.set(variableIndex, 1, 1.0);
   }

   @Override
   public void setVariableInactive(int variableIndex)
   {
      if (variableIndex >= activeVariables.getNumRows())
         throw new RuntimeException("variable index is outside the number of variables.");

      activeVariables.set(variableIndex, 1, 0.0);
   }

   @Override
   public void setAllVariablesActive()
   {
      activeVariables.reshape(originalQuadraticCostQMatrix.getNumRows(), 1);
      CommonOps.fill(activeVariables, 1.0);
   }

   @Override
   public void clear()
   {
      super.clear();

      originalQuadraticCostQMatrix.reshape(0, 0);
      originalQuadraticCostQVector.reshape(0, 0);

      originalLinearEqualityConstraintsAMatrix.reshape(0, 0);
      originalLinearEqualityConstraintsBVector.reshape(0, 0);

      originalLinearInequalityConstraintsCMatrixO.reshape(0, 0);
      originalLinearInequalityConstraintsDVectorO.reshape(0, 0);

      originalVariableLowerBounds.reshape(0, 0);
      originalVariableUpperBounds.reshape(0, 0);

      activeVariables.reshape(0, 0);
      activeVariableSolution.reshape(0, 0);
   }
}
