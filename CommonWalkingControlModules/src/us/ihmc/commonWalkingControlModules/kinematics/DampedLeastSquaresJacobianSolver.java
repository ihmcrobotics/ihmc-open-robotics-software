package us.ihmc.commonWalkingControlModules.kinematics;

import org.ejml.alg.dense.linsol.LinearSolver;
import org.ejml.alg.dense.linsol.LinearSolverFactory;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.utilities.screwTheory.JacobianSolver;

public class DampedLeastSquaresJacobianSolver implements JacobianSolver
{
   private final YoVariableRegistry registry;
   private final DenseMatrix64F jacobianTransposeTimesVector;
   private final DenseMatrix64F alphaMatrix;
   private final DenseMatrix64F longTerm;
   private final LinearSolver<DenseMatrix64F> dampedLeastSquaresSolver;
   private final DoubleYoVariable alpha;

   public DampedLeastSquaresJacobianSolver(String namePrefix, int matrixSize, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix);
      this.alpha = new DoubleYoVariable(namePrefix + "alpha", registry);
      
      this.jacobianTransposeTimesVector = new DenseMatrix64F(matrixSize, 1);
      this.alphaMatrix = new DenseMatrix64F(matrixSize, matrixSize);
      this.longTerm = new DenseMatrix64F(matrixSize, matrixSize);
      this.dampedLeastSquaresSolver = LinearSolverFactory.linear(matrixSize);
      parentRegistry.addChild(registry);
   }

   public void setAlpha(double alpha)
   {
      this.alpha.set(alpha);
   }

   public void solve(DenseMatrix64F solutionToPack, DenseMatrix64F jacobianMatrix, DenseMatrix64F vector)
   {
      CommonOps.multTransA(jacobianMatrix, vector, jacobianTransposeTimesVector);

      CommonOps.setIdentity(alphaMatrix);
      CommonOps.scale(alpha.getDoubleValue() * alpha.getDoubleValue(), alphaMatrix);

      // TODO: use CommonOps.multInner(jacobian, jTransposeJ)
      CommonOps.multTransA(jacobianMatrix, jacobianMatrix, longTerm);
      CommonOps.addEquals(longTerm, alphaMatrix);

      dampedLeastSquaresSolver.setA(longTerm);
      
      dampedLeastSquaresSolver.solve(jacobianTransposeTimesVector, solutionToPack);
   }

}
