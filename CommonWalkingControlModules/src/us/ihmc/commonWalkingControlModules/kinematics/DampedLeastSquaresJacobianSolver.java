package us.ihmc.commonWalkingControlModules.kinematics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.math.MathTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.utilities.screwTheory.JacobianSolver;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


/*
 * Use DampedLeastSquaresSolver instead
 */
@Deprecated
public class DampedLeastSquaresJacobianSolver implements JacobianSolver
{
   private final YoVariableRegistry registry;
   private final DenseMatrix64F tempVector;
   private final DenseMatrix64F tempMatrix;
   private final LinearSolver<DenseMatrix64F> dampedLeastSquaresSolver;
   private final DoubleYoVariable alpha;
   private final DenseMatrix64F jacobianMatrix;

   public DampedLeastSquaresJacobianSolver(String namePrefix, int matrixSize, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix);
      this.alpha = new DoubleYoVariable(namePrefix + "alpha", registry);
      
      this.tempVector = new DenseMatrix64F(matrixSize, 1);
      this.tempMatrix = new DenseMatrix64F(matrixSize, matrixSize);
      this.jacobianMatrix = new DenseMatrix64F(matrixSize, matrixSize);
      this.dampedLeastSquaresSolver = LinearSolverFactory.linear(matrixSize);
      parentRegistry.addChild(registry);
   }

   public void setAlpha(double alpha)
   {
      this.alpha.set(alpha);
   }

   public void solve(DenseMatrix64F jointSpaceVectorToPack, DenseMatrix64F taskSpaceVector)
   { 
      CommonOps.multTransA(jacobianMatrix, taskSpaceVector, tempVector);
      CommonOps.multTransA(jacobianMatrix, jacobianMatrix, tempMatrix);
      MatrixTools.addDiagonal(tempMatrix, MathTools.square(alpha.getDoubleValue()));
      dampedLeastSquaresSolver.setA(tempMatrix);

      dampedLeastSquaresSolver.solve(tempVector, jointSpaceVectorToPack);
   }

   public void inverseSolve(DenseMatrix64F taskSpaceVectorToPack, DenseMatrix64F jointSpaceVector)
   {
      CommonOps.transpose(jacobianMatrix, tempMatrix);
      dampedLeastSquaresSolver.setA(tempMatrix);
      dampedLeastSquaresSolver.solve(jointSpaceVector, taskSpaceVectorToPack);
      CommonOps.scale(MathTools.square(alpha.getDoubleValue()), taskSpaceVectorToPack);
      CommonOps.mult(jacobianMatrix, jointSpaceVector, tempVector);
      CommonOps.addEquals(taskSpaceVectorToPack, tempVector);
   }

   public void setJacobian(DenseMatrix64F jacobianMatrix)
   {
      this.jacobianMatrix.set(jacobianMatrix);
   }
   
   protected DenseMatrix64F getJacobianMatrix()
   {
      return jacobianMatrix;
   }
}
