package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public interface ActiveSetQPSolverWithInactiveVariablesInterface extends SimpleActiveSetQPSolverInterface
{
   void setActiveVariables(DenseMatrix64F activeVariables);
   void setVariableActive(int variableIndex);
   void setVariableInactive(int variableIndex);
   void setAllVariablesActive();

   default void setActiveVariables(double[] activeVariables)
   {
      setActiveVariables(MatrixTools.createVector(activeVariables));
   }

}
