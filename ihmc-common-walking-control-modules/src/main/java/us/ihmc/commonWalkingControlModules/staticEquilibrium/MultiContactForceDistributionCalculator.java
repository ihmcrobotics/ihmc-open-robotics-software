package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTools;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.ContactPoint.basisVectorsPerContactPoint;

/**
 * Whole-body force distribution calculator.
 * Solves the QP:
 *
 * min_{f} f^2                         (min force)
 *    s.t.  mg + sum(f) = 0            (lin static equilibrium)
 *    s.t.  sum (r x f + r x mg) = 0   (ang static equilibrium)
 *          f is friction constrained
 *          tau_min <= G - J^T f <= tau_max  (f is actuation constrained)
 *
 * Notation taken from EoM:
 * M qdd + C qd + G = tau + J^T f
 */
public class MultiContactForceDistributionCalculator
{
   /* Maintain static equilibrium */
   private static final int staticEquilibriumConstraints = 6;

   private final DMatrixRMaj Aeq = new DMatrixRMaj(0);
   private final DMatrixRMaj beq = new DMatrixRMaj(0);
   private final DMatrixRMaj Ain = new DMatrixRMaj(0);
   private final DMatrixRMaj bin = new DMatrixRMaj(0);
   private final DMatrixRMaj rho = new DMatrixRMaj(0);
   private final DMatrixRMaj quadraticCost = new DMatrixRMaj(0);
   private final DMatrixRMaj linearCost = new DMatrixRMaj(0);

   private final FramePoint3D contactPointPosition = new FramePoint3D();
   private final SimpleEfficientActiveSetQPSolver qpSolver = new SimpleEfficientActiveSetQPSolver();

   /**
    * Returns whether an optimal solution was found
    */
   public boolean solve(WholeBodyContactState input)
   {
      clear();

      double mg = 9.81 * input.getRobotMass();

      int rhoSize = WholeBodyContactState.numberOfBasisVectors * input.getNumberOfContactPoints();
      int decisionVariables = rhoSize;

      rho.reshape(rhoSize, 1);
      Aeq.reshape(staticEquilibriumConstraints, decisionVariables);
      beq.reshape(staticEquilibriumConstraints, 1);
      Ain.reshape(rhoSize + 2 * input.getNumberOfJoints(), rhoSize);
      bin.reshape(rhoSize + 2 * input.getNumberOfJoints(), 1);
      quadraticCost.reshape(rhoSize, rhoSize);
      linearCost.reshape(rhoSize, 1);

      CommonOps_DDRM.setIdentity(quadraticCost);
      CommonOps_DDRM.fill(beq, 0.0);

      for (int i = 0; i < rhoSize; i++)
      { // Constraint for rho to be positive
         Ain.set(i, i, -1.0);
      }

      DMatrixRMaj constraintUpperBound = input.getConstraintUpperBound();
      DMatrixRMaj constraintLowerBound = input.getConstraintLowerBound();
      DMatrixRMaj graspMatrixJacobianTranspose = input.getGraspMatrixJacobianTranspose();

      MatrixTools.setMatrixBlock(Ain, rhoSize, 0, graspMatrixJacobianTranspose, 0, 0, graspMatrixJacobianTranspose.getNumRows(), graspMatrixJacobianTranspose.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(bin, rhoSize, 0, constraintUpperBound, 0, 0, constraintUpperBound.getNumRows(), constraintUpperBound.getNumCols(), 1.0);

      MatrixTools.setMatrixBlock(Ain, rhoSize + graspMatrixJacobianTranspose.getNumRows(), 0, graspMatrixJacobianTranspose, 0, 0, graspMatrixJacobianTranspose.getNumRows(), graspMatrixJacobianTranspose.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(bin, rhoSize + graspMatrixJacobianTranspose.getNumRows(), 0, constraintLowerBound, 0, 0, constraintLowerBound.getNumRows(), constraintLowerBound.getNumCols(), -1.0);

      for (int contactPointIndex = 0; contactPointIndex < input.getNumberOfContactPoints(); contactPointIndex++)
      {
         contactPointPosition.setToZero(input.getContactFrame(contactPointIndex));
         contactPointPosition.changeFrame(ReferenceFrame.getWorldFrame());

         for (int basisVectorIndex = 0; basisVectorIndex < WholeBodyContactState.numberOfBasisVectors; basisVectorIndex++)
         {
            FrameVector3D basisVector = input.getBasisVector(contactPointIndex, basisVectorIndex);
            int column = basisVectorsPerContactPoint * contactPointIndex + basisVectorIndex;

            Aeq.set(0, column, basisVector.getX());
            Aeq.set(1, column, basisVector.getY());
            Aeq.set(2, column, basisVector.getZ());

            // x-component of cross product
            double xMomentScale = contactPointPosition.getY() * basisVector.getZ() - contactPointPosition.getZ() * basisVector.getY();
            Aeq.set(3, column, xMomentScale);

            // y-component of cross product
            double yMomentScale = contactPointPosition.getZ() * basisVector.getX() - contactPointPosition.getX() * basisVector.getZ();
            Aeq.set(4, column, yMomentScale);

            // z-component of cross product
            double zMomentScale = contactPointPosition.getX() * basisVector.getY() - contactPointPosition.getY() * basisVector.getX();
            Aeq.set(5, column, zMomentScale);
         }
      }

      FramePoint3DReadOnly centerOfMass = input.getCenterOfMass();
      beq.set(2, 0, mg);
      beq.set(3, 0, mg * centerOfMass.getY());
      beq.set(4, 0, -mg * centerOfMass.getX());

      qpSolver.clear();
      qpSolver.resetActiveSet();

      qpSolver.setMaxNumberOfIterations(500);
      qpSolver.setConvergenceThreshold(1e-9);
      qpSolver.setQuadraticCostFunction(quadraticCost, linearCost, 0.0);
      qpSolver.setLinearEqualityConstraints(Aeq, beq);
      qpSolver.setLinearInequalityConstraints(Ain, bin);

      qpSolver.solve(rho);
      return !MatrixTools.containsNaN(rho);
   }

   public DMatrixRMaj getRho()
   {
      return rho;
   }

   private void clear()
   {
      Aeq.zero();
      beq.zero();
      Ain.zero();
      bin.zero();
      quadraticCost.zero();
      linearCost.zero();
      rho.zero();
   }

   public Vector3D getResolvedForce(int contactPointIndex, WholeBodyContactState input)
   {
      Vector3D resolvedForce = new Vector3D();
      for (int i = 0; i < WholeBodyContactState.numberOfBasisVectors; i++)
      {
         FrameVector3D basisVector = input.getBasisVector(contactPointIndex, i);
         basisVector.changeFrame(ReferenceFrame.getWorldFrame());

         double rho = this.rho.get(WholeBodyContactState.numberOfBasisVectors * contactPointIndex + i, 0);

         resolvedForce.addX(basisVector.getX() * rho);
         resolvedForce.addY(basisVector.getY() * rho);
         resolvedForce.addZ(basisVector.getZ() * rho);
      }

      return resolvedForce;
   }
}
