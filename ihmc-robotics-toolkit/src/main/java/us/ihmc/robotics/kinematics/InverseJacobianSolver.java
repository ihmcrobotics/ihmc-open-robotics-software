package us.ihmc.robotics.kinematics;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.dense.row.linsol.svd.SolvePseudoInverseSvd_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

public class InverseJacobianSolver
{
   private final LinearSolverDense<DMatrixRMaj> linearAlgebraSolver;

   private final DMatrixRMaj selectionMatrix;
   private final DMatrixRMaj subspaceJacobianMatrix;
   private final DMatrixRMaj subspaceSpatialVelocity;
   private final DMatrixRMaj jacobianMatrixTransposed;
   private final DMatrixRMaj jacobianTimesJacobianTransposedMatrix;
   private final DMatrixRMaj lamdaSquaredMatrix;
   private final DMatrixRMaj jacobianTimesJacobianTransposedPlusLamdaSquaredMatrix;
   private final DMatrixRMaj jacobianTimesSpatialVelocity;
   private final DMatrixRMaj jacobianTransposedTimesJacobianMatrix;

   private final DMatrixRMaj intermediateSubspaceSpatialVelocity;
   private final DMatrixRMaj intermediateResultInTaskspace;
   private final DMatrixRMaj jointspaceVelocity;

   private final int maximumNumberOfConstraints;
   private int numberOfConstraints;
   private final int numberOfDoF;

   public static InverseJacobianSolver createInverseJacobianSolver(int maximumNumberOfConstraints, int numberOfDoF, boolean useSVD)
   {
      LinearSolverDense<DMatrixRMaj> solver;
      if (useSVD)
      {
         solver = new SolvePseudoInverseSvd_DDRM(maximumNumberOfConstraints, maximumNumberOfConstraints);
      }
      else
      {
         solver = LinearSolverFactory_DDRM.leastSquares(maximumNumberOfConstraints, maximumNumberOfConstraints);
      }
      return new InverseJacobianSolver(maximumNumberOfConstraints, numberOfDoF, solver);
   }

   public InverseJacobianSolver(int maximumNumberOfConstraints, int numberOfDoF, LinearSolverDense<DMatrixRMaj> solver)
   {
      selectionMatrix = CommonOps_DDRM.identity(maximumNumberOfConstraints);

      this.numberOfDoF = numberOfDoF;
      this.maximumNumberOfConstraints = maximumNumberOfConstraints;
      this.numberOfConstraints = maximumNumberOfConstraints;

      subspaceJacobianMatrix = new DMatrixRMaj(maximumNumberOfConstraints, numberOfDoF);
      subspaceSpatialVelocity = new DMatrixRMaj(maximumNumberOfConstraints, 1);
      jacobianMatrixTransposed = new DMatrixRMaj(numberOfDoF, maximumNumberOfConstraints);
      jacobianTimesJacobianTransposedMatrix = new DMatrixRMaj(maximumNumberOfConstraints, maximumNumberOfConstraints);

      jacobianTimesSpatialVelocity = new DMatrixRMaj(numberOfDoF, 1);
      jacobianTransposedTimesJacobianMatrix = new DMatrixRMaj(numberOfDoF, numberOfDoF);

      lamdaSquaredMatrix = new DMatrixRMaj(numberOfDoF, numberOfDoF);
      jacobianTimesJacobianTransposedPlusLamdaSquaredMatrix = new DMatrixRMaj(numberOfDoF, numberOfDoF);

      linearAlgebraSolver = solver;

      intermediateSubspaceSpatialVelocity = new DMatrixRMaj(maximumNumberOfConstraints, 1);
      intermediateResultInTaskspace = new DMatrixRMaj(maximumNumberOfConstraints, 1);
      jointspaceVelocity = new DMatrixRMaj(numberOfDoF, 1);
   }

   public void setSelectionMatrix(DMatrixRMaj selectionMatrix)
   {
      numberOfConstraints = selectionMatrix.getNumRows();
      this.selectionMatrix.reshape(numberOfConstraints, selectionMatrix.getNumCols());
      this.selectionMatrix.set(selectionMatrix);
   }

   public void setSelectionMatrixForFullConstraint()
   {
      numberOfConstraints = maximumNumberOfConstraints;
      this.selectionMatrix.reshape(numberOfConstraints, numberOfConstraints);
      CommonOps_DDRM.setIdentity(selectionMatrix);
   }

   public DMatrixRMaj getJointspaceVelocity()
   {
      return jointspaceVelocity;
   }

   public DMatrixRMaj getSubspaceSpatialVelocity()
   {
      return subspaceSpatialVelocity;
   }

   public boolean solveUsingJacobianInverse(DMatrixRMaj spatialVelocity, DMatrixRMaj jacobianMatrix)
   {
      computeSubspaceJacobian(subspaceJacobianMatrix, jacobianMatrix);
      computeSubspaceSpatialVelocity(subspaceSpatialVelocity, spatialVelocity);

      boolean success = linearAlgebraSolver.setA(subspaceJacobianMatrix);
      if (success)
         linearAlgebraSolver.solve(subspaceSpatialVelocity, jointspaceVelocity);

      return success;
   }

   public boolean solveUsingJacobianPseudoInverseOne(DMatrixRMaj spatialVelocity, DMatrixRMaj jacobianMatrix)
   {
      computeSubspaceJacobian(subspaceJacobianMatrix, jacobianMatrix);
      computeSubspaceSpatialVelocity(subspaceSpatialVelocity, spatialVelocity);

      // J^T
      computeJacobianTransposed(jacobianMatrixTransposed, subspaceJacobianMatrix);

      // J^T J
      computeJacobianTransposedTimesJacobian(jacobianTransposedTimesJacobianMatrix, subspaceJacobianMatrix);

      // J^T*deltaX
      jacobianTimesSpatialVelocity.reshape(numberOfDoF, 1);
      CommonOps_DDRM.mult(jacobianMatrixTransposed, subspaceSpatialVelocity, jacobianTimesSpatialVelocity);

      boolean success = linearAlgebraSolver.setA(jacobianTransposedTimesJacobianMatrix);
      // Solve J^T*J delta q = J^T * deltaX
      if (success)
         linearAlgebraSolver.solve(jacobianTimesSpatialVelocity, jointspaceVelocity);

      return success;
   }

   public boolean solveUsingJacobianPseudoInverseTwo(DMatrixRMaj spatialVelocity, DMatrixRMaj jacobianMatrix)
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
      CommonOps_DDRM.mult(jacobianMatrixTransposed, intermediateResultInTaskspace, jointspaceVelocity);

      return success;
   }

   public boolean solveUsingDampedLeastSquares(DMatrixRMaj spatialVelocity, DMatrixRMaj jacobianMatrix, double lambdaLeastSquares)
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
      CommonOps_DDRM.add(jacobianTimesJacobianTransposedMatrix, lamdaSquaredMatrix, jacobianTimesJacobianTransposedPlusLamdaSquaredMatrix);

      boolean success = linearAlgebraSolver.setA(jacobianTimesJacobianTransposedPlusLamdaSquaredMatrix);

      // Solve J*J^T deltaX = f
      if (success)
         linearAlgebraSolver.solve(subspaceSpatialVelocity, intermediateResultInTaskspace);
      CommonOps_DDRM.mult(jacobianMatrixTransposed, intermediateResultInTaskspace, jointspaceVelocity);

      return success;
   }

   public boolean solveUsingNullspaceMethod(DMatrixRMaj spatialVelocity, DMatrixRMaj jacobianMatrix, DMatrixRMaj privilegedJointVelocities)
   {
      computeSubspaceJacobian(subspaceJacobianMatrix, jacobianMatrix);
      computeSubspaceSpatialVelocity(subspaceSpatialVelocity, spatialVelocity);

      intermediateSubspaceSpatialVelocity.reshape(numberOfConstraints, 1);
      intermediateSubspaceSpatialVelocity.set(subspaceSpatialVelocity);
      // xDot - J qDot0
      CommonOps_DDRM.multAdd(-1.0, subspaceJacobianMatrix, privilegedJointVelocities, intermediateSubspaceSpatialVelocity);

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
      CommonOps_DDRM.multAdd(jacobianMatrixTransposed, intermediateResultInTaskspace, jointspaceVelocity);

      return success;
   }

   public boolean solveUsingNullspaceMethodWithoutSelectionMatrix(DMatrixRMaj spatialVelocity, DMatrixRMaj jacobianMatrix, DMatrixRMaj privilegedJointVelocities)
   {
      int numberOfConstraints = jacobianMatrix.getNumRows();
      subspaceJacobianMatrix.reshape(numberOfConstraints, jacobianMatrix.getNumCols());
      subspaceJacobianMatrix.set(jacobianMatrix);
      subspaceSpatialVelocity.reshape(spatialVelocity.getNumRows(), subspaceSpatialVelocity.getNumCols());
      subspaceSpatialVelocity.set(spatialVelocity);

      intermediateSubspaceSpatialVelocity.reshape(numberOfConstraints, 1);
      intermediateSubspaceSpatialVelocity.set(subspaceSpatialVelocity);
      // xDot - J qDot0
      CommonOps_DDRM.multAdd(-1.0, subspaceJacobianMatrix, privilegedJointVelocities, intermediateSubspaceSpatialVelocity);

      // J^T
      jacobianMatrixTransposed.reshape(numberOfDoF, numberOfConstraints);
      CommonOps_DDRM.transpose(subspaceJacobianMatrix, jacobianMatrixTransposed);

      // J J^T
      jacobianTimesJacobianTransposedMatrix.reshape(numberOfConstraints, numberOfConstraints);
      CommonOps_DDRM.multOuter(subspaceJacobianMatrix, jacobianTimesJacobianTransposedMatrix);

      intermediateResultInTaskspace.reshape(numberOfConstraints, 1);

      boolean success = linearAlgebraSolver.setA(jacobianTimesJacobianTransposedMatrix);

      // Solve J*J^T xDot = f
      if (success)
         linearAlgebraSolver.solve(intermediateSubspaceSpatialVelocity, intermediateResultInTaskspace);
      // qDot = J^T f + qDot0
      jointspaceVelocity.set(privilegedJointVelocities);
      CommonOps_DDRM.multAdd(jacobianMatrixTransposed, intermediateResultInTaskspace, jointspaceVelocity);

      return success;
   }

   private void computeJacobianTransposedTimesJacobian(DMatrixRMaj resultToPack, DMatrixRMaj jacobian)
   {
      resultToPack.reshape(numberOfDoF, numberOfDoF);
      CommonOps_DDRM.multInner(jacobian, resultToPack);
   }

   private void computeJacobianTimesJacobianTransposed(DMatrixRMaj resultToPack, DMatrixRMaj jacobian)
   {
      resultToPack.reshape(numberOfConstraints, numberOfConstraints);
      CommonOps_DDRM.multOuter(jacobian, resultToPack);
   }

   private void computeJacobianTransposed(DMatrixRMaj jacobianTransposedToPack, DMatrixRMaj jacobian)
   {
      jacobianTransposedToPack.reshape(numberOfDoF, numberOfConstraints);
      CommonOps_DDRM.transpose(jacobian, jacobianTransposedToPack);
   }

   private void computeSubspaceJacobian(DMatrixRMaj subspaceJacobianMatrixToPack, DMatrixRMaj jacobianMatrix)
   {
      subspaceJacobianMatrixToPack.reshape(numberOfConstraints, numberOfDoF);
      CommonOps_DDRM.mult(selectionMatrix, jacobianMatrix, subspaceJacobianMatrixToPack);
   }

   private void computeSubspaceSpatialVelocity(DMatrixRMaj subspaceSpatialVelocityToPack, DMatrixRMaj spatialVelocity)
   {
      subspaceSpatialVelocityToPack.reshape(numberOfConstraints, 1);
      CommonOps_DDRM.mult(selectionMatrix, spatialVelocity, subspaceSpatialVelocityToPack);
   }

   public int getNumberOfConstraints()
   {
      return numberOfConstraints;
   }

   public double computeDeterminant(DMatrixRMaj jacobianMatrix)
   {
      computeJacobianTimesJacobianTransposed(jacobianTimesJacobianTransposedMatrix, jacobianMatrix);
      return CommonOps_DDRM.det(jacobianTimesJacobianTransposedMatrix);
   }

   public double getLastComputedDeterminant()
   {
      double det = CommonOps_DDRM.det(jacobianTimesJacobianTransposedMatrix);
      return det;
   }

   public DMatrixRMaj getSelectionMatrix()
   {
      return selectionMatrix;
   }
}
