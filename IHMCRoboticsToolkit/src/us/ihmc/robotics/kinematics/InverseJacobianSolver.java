package us.ihmc.robotics.kinematics;

import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

public class InverseJacobianSolver
{
   private final LinearSolver<DenseMatrix64F> linearAlgebraSolver;

   private final DenseMatrix64F selectionMatrix;
   private final DenseMatrix64F subspaceJacobianMatrix;
   private final DenseMatrix64F subspaceSpatialVelocity;
   private final DenseMatrix64F jacobianMatrixTransposed;
   private final DenseMatrix64F jacobianTimesJacobianTransposedMatrix;
   private final DenseMatrix64F lamdaSquaredMatrix;
   private final DenseMatrix64F jacobianTimesJacobianTransposedPlusLamdaSquaredMatrix;
   private final DenseMatrix64F jacobianTimesSpatialVelocity;
   private final DenseMatrix64F jacobianTransposedTimesJacobianMatrix;

   private final DenseMatrix64F intermediateSubspaceSpatialVelocity;
   private final DenseMatrix64F intermediateResultInTaskspace;
   private final DenseMatrix64F jointspaceVelocity;

   private final int maximumNumberOfConstraints;
   private int numberOfConstraints;
   private final int numberOfDoF;

   public static InverseJacobianSolver createInverseJacobianSolver(int maximumNumberOfConstraints, int numberOfDoF, boolean useSVD)
   {
      LinearSolver<DenseMatrix64F> solver;
      if (useSVD)
      {
         solver = new SolvePseudoInverseSvd(maximumNumberOfConstraints, maximumNumberOfConstraints);
      }
      else
      {
         solver = LinearSolverFactory.leastSquares(maximumNumberOfConstraints, maximumNumberOfConstraints);
      }
      return new InverseJacobianSolver(maximumNumberOfConstraints, numberOfDoF, solver);
   }

   public InverseJacobianSolver(int maximumNumberOfConstraints, int numberOfDoF, LinearSolver<DenseMatrix64F> solver)
   {
      selectionMatrix = CommonOps.identity(maximumNumberOfConstraints);

      this.numberOfDoF = numberOfDoF;
      this.maximumNumberOfConstraints = maximumNumberOfConstraints;
      this.numberOfConstraints = maximumNumberOfConstraints;

      subspaceJacobianMatrix = new DenseMatrix64F(maximumNumberOfConstraints, numberOfDoF);
      subspaceSpatialVelocity = new DenseMatrix64F(maximumNumberOfConstraints, 1);
      jacobianMatrixTransposed = new DenseMatrix64F(numberOfDoF, maximumNumberOfConstraints);
      jacobianTimesJacobianTransposedMatrix = new DenseMatrix64F(maximumNumberOfConstraints, maximumNumberOfConstraints);

      jacobianTimesSpatialVelocity = new DenseMatrix64F(numberOfDoF, 1);
      jacobianTransposedTimesJacobianMatrix = new DenseMatrix64F(numberOfDoF, numberOfDoF);

      lamdaSquaredMatrix = new DenseMatrix64F(numberOfDoF, numberOfDoF);
      jacobianTimesJacobianTransposedPlusLamdaSquaredMatrix = new DenseMatrix64F(numberOfDoF, numberOfDoF);

      linearAlgebraSolver = solver;

      intermediateSubspaceSpatialVelocity = new DenseMatrix64F(maximumNumberOfConstraints, 1);
      intermediateResultInTaskspace = new DenseMatrix64F(maximumNumberOfConstraints, 1);
      jointspaceVelocity = new DenseMatrix64F(numberOfDoF, 1);
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      numberOfConstraints = selectionMatrix.getNumRows();
      this.selectionMatrix.reshape(numberOfConstraints, selectionMatrix.getNumCols());
      this.selectionMatrix.set(selectionMatrix);
   }

   public void setSelectionMatrixForFullConstraint()
   {
      numberOfConstraints = maximumNumberOfConstraints;
      this.selectionMatrix.reshape(numberOfConstraints, numberOfConstraints);
      CommonOps.setIdentity(selectionMatrix);
   }

   public DenseMatrix64F getJointspaceVelocity()
   {
      return jointspaceVelocity;
   }

   public DenseMatrix64F getSubspaceSpatialVelocity()
   {
      return subspaceSpatialVelocity;
   }

   public boolean solveUsingJacobianInverse(DenseMatrix64F spatialVelocity, DenseMatrix64F jacobianMatrix)
   {
      computeSubspaceJacobian(subspaceJacobianMatrix, jacobianMatrix);
      computeSubspaceSpatialVelocity(subspaceSpatialVelocity, spatialVelocity);

      boolean success = linearAlgebraSolver.setA(subspaceJacobianMatrix);
      if (success)
         linearAlgebraSolver.solve(subspaceSpatialVelocity, jointspaceVelocity);

      return success;
   }

   public boolean solveUsingJacobianPseudoInverseOne(DenseMatrix64F spatialVelocity, DenseMatrix64F jacobianMatrix)
   {
      computeSubspaceJacobian(subspaceJacobianMatrix, jacobianMatrix);
      computeSubspaceSpatialVelocity(subspaceSpatialVelocity, spatialVelocity);

      // J^T
      computeJacobianTransposed(jacobianMatrixTransposed, subspaceJacobianMatrix);

      // J^T J
      computeJacobianTransposedTimesJacobian(jacobianTransposedTimesJacobianMatrix, subspaceJacobianMatrix);

      // J^T*deltaX
      jacobianTimesSpatialVelocity.reshape(numberOfDoF, 1);
      CommonOps.mult(jacobianMatrixTransposed, subspaceSpatialVelocity, jacobianTimesSpatialVelocity);

      boolean success = linearAlgebraSolver.setA(jacobianTransposedTimesJacobianMatrix);
      // Solve J^T*J delta q = J^T * deltaX
      if (success)
         linearAlgebraSolver.solve(jacobianTimesSpatialVelocity, jointspaceVelocity);

      return success;
   }

   public boolean solveUsingJacobianPseudoInverseTwo(DenseMatrix64F spatialVelocity, DenseMatrix64F jacobianMatrix)
   {
      computeSubspaceJacobian(subspaceJacobianMatrix, jacobianMatrix);
      computeSubspaceSpatialVelocity(subspaceSpatialVelocity, spatialVelocity);

      // J^T
      computeJacobianTransposed(jacobianMatrixTransposed, subspaceJacobianMatrix);

      // J J^T
      computeJacobianTimesJacobianTransposed(jacobianTimesJacobianTransposedMatrix, subspaceJacobianMatrix);

      intermediateResultInTaskspace.reshape(numberOfConstraints, 1);

      boolean success = linearAlgebraSolver.setA(jacobianTimesJacobianTransposedMatrix);

      // Solve J*J^T deltaX = f
      if (success)
         linearAlgebraSolver.solve(subspaceSpatialVelocity, intermediateResultInTaskspace);
      CommonOps.mult(jacobianMatrixTransposed, intermediateResultInTaskspace, jointspaceVelocity);

      return success;
   }

   public boolean solveUsingDampedLeastSquares(DenseMatrix64F spatialVelocity, DenseMatrix64F jacobianMatrix, double lambdaLeastSquares)
   {
      // J
      computeSubspaceJacobian(subspaceJacobianMatrix, jacobianMatrix);
      computeSubspaceSpatialVelocity(subspaceSpatialVelocity, spatialVelocity);

      // J^T
      computeJacobianTransposed(jacobianMatrixTransposed, subspaceJacobianMatrix);

      // J J^T
      computeJacobianTimesJacobianTransposed(jacobianTimesJacobianTransposedMatrix, subspaceJacobianMatrix);

      intermediateResultInTaskspace.reshape(numberOfConstraints, 1);

      lamdaSquaredMatrix.reshape(numberOfConstraints, numberOfConstraints);
      lamdaSquaredMatrix.zero();
      for (int i = 0; i < numberOfConstraints; i++)
         lamdaSquaredMatrix.set(i, i, lambdaLeastSquares);

      jacobianTimesJacobianTransposedPlusLamdaSquaredMatrix.reshape(numberOfConstraints, numberOfConstraints);
      jacobianTimesJacobianTransposedPlusLamdaSquaredMatrix.set(jacobianTimesJacobianTransposedMatrix);
      CommonOps.add(jacobianTimesJacobianTransposedMatrix, lamdaSquaredMatrix, jacobianTimesJacobianTransposedPlusLamdaSquaredMatrix);

      boolean success = linearAlgebraSolver.setA(jacobianTimesJacobianTransposedPlusLamdaSquaredMatrix);

      // Solve J*J^T deltaX = f
      if (success)
         linearAlgebraSolver.solve(subspaceSpatialVelocity, intermediateResultInTaskspace);
      CommonOps.mult(jacobianMatrixTransposed, intermediateResultInTaskspace, jointspaceVelocity);

      return success;
   }

   public boolean solveUsingNullspaceMethod(DenseMatrix64F spatialVelocity, DenseMatrix64F jacobianMatrix, DenseMatrix64F privilegedJointVelocities)
   {
      computeSubspaceJacobian(subspaceJacobianMatrix, jacobianMatrix);
      computeSubspaceSpatialVelocity(subspaceSpatialVelocity, spatialVelocity);

      intermediateSubspaceSpatialVelocity.reshape(numberOfConstraints, 1);
      intermediateSubspaceSpatialVelocity.set(subspaceSpatialVelocity);
      // xDot - J qDot0
      CommonOps.multAdd(-1.0, subspaceJacobianMatrix, privilegedJointVelocities, intermediateSubspaceSpatialVelocity);

      // J^T
      computeJacobianTransposed(jacobianMatrixTransposed, subspaceJacobianMatrix);

      // J J^T
      computeJacobianTimesJacobianTransposed(jacobianTimesJacobianTransposedMatrix, subspaceJacobianMatrix);

      intermediateResultInTaskspace.reshape(numberOfConstraints, 1);

      boolean success = linearAlgebraSolver.setA(jacobianTimesJacobianTransposedMatrix);

      // Solve J*J^T xDot = f
      if (success)
         linearAlgebraSolver.solve(intermediateSubspaceSpatialVelocity, intermediateResultInTaskspace);
      // qDot = J^T f + qDot0
      jointspaceVelocity.set(privilegedJointVelocities);
      CommonOps.multAdd(jacobianMatrixTransposed, intermediateResultInTaskspace, jointspaceVelocity);

      return success;
   }

   private void computeJacobianTransposedTimesJacobian(DenseMatrix64F resultToPack, DenseMatrix64F jacobian)
   {
      resultToPack.reshape(numberOfDoF, numberOfDoF);
      CommonOps.multInner(jacobian, resultToPack);
   }

   private void computeJacobianTimesJacobianTransposed(DenseMatrix64F resultToPack, DenseMatrix64F jacobian)
   {
      resultToPack.reshape(numberOfConstraints, numberOfConstraints);
      CommonOps.multOuter(jacobian, resultToPack);
   }

   private void computeJacobianTransposed(DenseMatrix64F jacobianTransposedToPack, DenseMatrix64F jacobian)
   {
      jacobianTransposedToPack.reshape(numberOfDoF, numberOfConstraints);
      CommonOps.transpose(jacobian, jacobianTransposedToPack);
   }

   private void computeSubspaceJacobian(DenseMatrix64F subspaceJacobianMatrixToPack, DenseMatrix64F jacobianMatrix)
   {
      subspaceJacobianMatrixToPack.reshape(numberOfConstraints, numberOfDoF);
      CommonOps.mult(selectionMatrix, jacobianMatrix, subspaceJacobianMatrixToPack);
   }

   private void computeSubspaceSpatialVelocity(DenseMatrix64F subspaceSpatialVelocityToPack, DenseMatrix64F spatialVelocity)
   {
      subspaceSpatialVelocityToPack.reshape(numberOfConstraints, 1);
      CommonOps.mult(selectionMatrix, spatialVelocity, subspaceSpatialVelocityToPack);
   }

   public int getNumberOfConstraints()
   {
      return numberOfConstraints;
   }

   public double computeDeterminant(DenseMatrix64F jacobianMatrix)
   {
      computeJacobianTimesJacobianTransposed(jacobianTimesJacobianTransposedMatrix, jacobianMatrix);
      return CommonOps.det(jacobianTimesJacobianTransposedMatrix);
   }

   public double getLastComputedDeterminant()
   {
      double det = CommonOps.det(jacobianTimesJacobianTransposedMatrix);
      return det;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }
}
