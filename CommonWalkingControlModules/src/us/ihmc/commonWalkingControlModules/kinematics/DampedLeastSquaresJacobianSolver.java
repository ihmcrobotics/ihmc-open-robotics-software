package us.ihmc.commonWalkingControlModules.kinematics;

import org.ejml.alg.dense.linsol.LinearSolver;
import org.ejml.alg.dense.linsol.LinearSolverFactory;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.screwTheory.JacobianSolver;

public class DampedLeastSquaresJacobianSolver implements JacobianSolver
{
   private final YoVariableRegistry registry;
   private final DenseMatrix64F tempVector;
   private final DenseMatrix64F tempMatrix;
   private final LinearSolver<DenseMatrix64F> dampedLeastSquaresSolver;
   private final DoubleYoVariable alpha;

   public DampedLeastSquaresJacobianSolver(String namePrefix, int matrixSize, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix);
      this.alpha = new DoubleYoVariable(namePrefix + "alpha", registry);
      
      this.tempVector = new DenseMatrix64F(matrixSize, 1);
      this.tempMatrix = new DenseMatrix64F(matrixSize, matrixSize);
      this.dampedLeastSquaresSolver = LinearSolverFactory.linear(matrixSize);
      parentRegistry.addChild(registry);
   }

   public void setAlpha(double alpha)
   {
      this.alpha.set(alpha);
   }

   public void solve(DenseMatrix64F solutionToPack, DenseMatrix64F jacobianMatrix, DenseMatrix64F taskSpaceVector)
   { 
      CommonOps.multTransA(jacobianMatrix, taskSpaceVector, tempVector);
      CommonOps.multTransA(jacobianMatrix, jacobianMatrix, tempMatrix);
      MatrixTools.addDiagonal(tempMatrix, MathTools.square(alpha.getDoubleValue()));
      dampedLeastSquaresSolver.setA(tempMatrix);

      dampedLeastSquaresSolver.solve(tempVector, solutionToPack);
   }
}
