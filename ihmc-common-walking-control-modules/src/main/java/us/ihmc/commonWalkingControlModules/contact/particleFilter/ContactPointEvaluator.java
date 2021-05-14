package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.NativeCommonOps;

/**
 * QP-based force resolver for particle-filter-based contact estimator, based on the following:
 * http://groups.csail.mit.edu/robotics-center/public_papers/Manuelli16.pdf
 */
public class ContactPointEvaluator
{
   private final SimpleEfficientActiveSetQPSolver qpSolver = new SimpleEfficientActiveSetQPSolver();

   private final double convergenceThreshold = 1e-10;
   private final int maximumNumberOfIterations = 500;
   private final double dampingTerm = 1e-5;

   private final Vector3D[] polyhedraBasisVectors = new Vector3D[4];
   private final FrameVector3D[] polyhedraFrameBasisVectors = new FrameVector3D[4];

   private final DMatrixRMaj basisVectorMatrix = new DMatrixRMaj(3, 4);
   /** Shorthand for J^T * F */
   private final DMatrixRMaj JTF = new DMatrixRMaj(0);
   /** Shorthand for sigma^-1 * J^T * F */
   private final DMatrixRMaj SInvJTF = new DMatrixRMaj(0);
   private final DMatrixRMaj sigmaInv = new DMatrixRMaj(0);
   private final DMatrixRMaj diagonalCost = new DMatrixRMaj(0);
   private final DMatrixRMaj quadraticCost = new DMatrixRMaj(0);
   private final DMatrixRMaj linearCost = new DMatrixRMaj(0);
   private final DMatrixRMaj scalarCost = new DMatrixRMaj(0);
   /** Inequality terms */
   private final DMatrixRMaj Ain = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj bin = new DMatrixRMaj(4, 1);
   /** Equality terms - not used */
   private final DMatrixRMaj Aeq = new DMatrixRMaj(0, 4);
   private final DMatrixRMaj beq = new DMatrixRMaj(0, 1);
   /** Basis vector coefficients to solve */
   private final DMatrixRMaj rho = new DMatrixRMaj(4, 1);

   private final DMatrixRMaj estimatedForce = new DMatrixRMaj(3, 1);

   public ContactPointEvaluator()
   {
      for (int i = 0; i < polyhedraBasisVectors.length; i++)
      {
         polyhedraBasisVectors[i] = new Vector3D();
         polyhedraBasisVectors[i].setToNaN();

         polyhedraFrameBasisVectors[i] = new FrameVector3D();
      }

      CommonOps_DDRM.setIdentity(Ain);
      CommonOps_DDRM.scale(-1.0, Ain);
      CommonOps_DDRM.fill(beq, 0.0);
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      double angleFromNormal = Math.atan(coefficientOfFriction);

      for (int i = 0; i < polyhedraBasisVectors.length; i++)
      {
         Vector3D basisVector = polyhedraBasisVectors[i];
         basisVector.set(Axis3D.Z);

         double angleToRotateAroundInXY = i * Math.PI / 2.0;
         AxisAngle basisVectorRotation = new AxisAngle(Math.cos(angleToRotateAroundInXY), Math.sin(angleToRotateAroundInXY), 0.0, angleFromNormal);
         basisVectorRotation.transform(basisVector);

         // provided frame is "z out" at the surface and the basis vectors should point into the mesh
         basisVector.negate();
      }
   }

   /**
    * Solves the QP in eq 12 of the above paper. The returned value can be used to compute the probability associated with the given contact location
    *
    * @param jointspaceResidual gamma in the above paper (eq 12)
    * @param contactPointJacobian J_rt in the above paper (eq 12)
    * @param sigma modelled noise of the joint angles, i.e. relative expected joint torque std. dev (eq 10)
    * @param zInContactFrame reference frame of the contact point, with z pointing into the mesh
    * @return likelihood of the given contact point, QP in the above paper
    */
   public double computeMaximumLikelihoodForce(DMatrixRMaj jointspaceResidual, DMatrixRMaj contactPointJacobian, DMatrixRMaj sigma, ReferenceFrame zInContactFrame)
   {
      if (polyhedraBasisVectors[0].containsNaN())
      {
         LogTools.debug("Coefficient of friction has not been set yet");
         return 0.0;
      }

      for (int i = 0; i < polyhedraFrameBasisVectors.length; i++)
      {
         polyhedraFrameBasisVectors[i].setIncludingFrame(zInContactFrame, polyhedraBasisVectors[i]);
         polyhedraFrameBasisVectors[i].changeFrame(ReferenceFrame.getWorldFrame());
         polyhedraFrameBasisVectors[i].get(0, i, basisVectorMatrix);
      }

      CommonOps_DDRM.invert(sigma, sigmaInv);
      CommonOps_DDRM.multTransA(contactPointJacobian, basisVectorMatrix, JTF);
      NativeCommonOps.multQuad(JTF, sigmaInv, quadraticCost);

      diagonalCost.reshape(quadraticCost.getNumRows(), quadraticCost.getNumCols());
      CommonOps_DDRM.setIdentity(diagonalCost);
      CommonOps_DDRM.scale(dampingTerm, diagonalCost);
      CommonOps_DDRM.addEquals(quadraticCost, diagonalCost);

      CommonOps_DDRM.mult(sigmaInv, JTF, SInvJTF);
      CommonOps_DDRM.multTransA(SInvJTF, jointspaceResidual, linearCost);
      CommonOps_DDRM.scale(-1.0, linearCost);

      NativeCommonOps.multQuad(jointspaceResidual, sigmaInv, scalarCost);

      qpSolver.clear();
      qpSolver.resetActiveSet();

      qpSolver.setMaxNumberOfIterations(maximumNumberOfIterations);
      qpSolver.setConvergenceThreshold(convergenceThreshold);
      qpSolver.setQuadraticCostFunction(quadraticCost, linearCost, scalarCost.get(0, 0));
      qpSolver.setLinearInequalityConstraints(Aeq, beq);
      qpSolver.setLinearInequalityConstraints(Ain, bin);

      qpSolver.solve(rho);
      CommonOps_DDRM.mult(basisVectorMatrix, rho, estimatedForce);

      return qpSolver.getObjectiveCost(rho);
   }

   public DMatrixRMaj getEstimatedForce()
   {
      return estimatedForce;
   }
}
