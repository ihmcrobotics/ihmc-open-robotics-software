package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule.detector;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.exceptions.NoConvergenceException;
import us.ihmc.convexOptimization.quadraticProgram.QuadProgSolver;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;

/**
 * QP-based force resolver for particle-filter-based contact estimator, based on the following:
 * http://groups.csail.mit.edu/robotics-center/public_papers/Manuelli16.pdf
 */
public class ContactPointEvaluator
{
   private final QuadProgSolver qpSolver = new QuadProgSolver();

   private final Vector3D[] polyhedraBasisVectors = new Vector3D[4];
   private final FrameVector3D[] polyhedraFrameBasisVectors = new FrameVector3D[4];

   private final DMatrixRMaj basisVectorMatrix = new DMatrixRMaj(3, 4);
   private final DMatrixRMaj JTF = new DMatrixRMaj();
   /** Quadratic term - input to the qp */
   private final DMatrixRMaj Q = new DMatrixRMaj();
   /** Linear term - input to the qp */
   private final DMatrixRMaj f = new DMatrixRMaj();
   /** Inequality terms */
   private final DMatrixRMaj Ain = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj bin = new DMatrixRMaj(4, 1);
   /** Equality terms - not used */
   private final DMatrixRMaj Aeq = new DMatrixRMaj(4, 1);
   private final DMatrixRMaj beq = new DMatrixRMaj(1, 1);
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
      }
   }

   /**
    * Solves the QP in eq 12 of the above paper. The returned value is proportional to the probability associated with the given contact location
    *
    * @param jointspaceResidual gamma in the above paper (eq 12)
    * @param contactPointJacobian J_rt in the above paper (eq 12)
    * @param zOutContactFrame reference frame of the contact point, with z pointing away from the mesh
    * @return likelihood of the given contact point, QP in the above paper
    */
   public double evaluate(DMatrixRMaj jointspaceResidual, DMatrixRMaj contactPointJacobian, ReferenceFrame zOutContactFrame)
   {
      if (polyhedraBasisVectors[0].containsNaN())
      {
         LogTools.debug("Coefficient of friction has not been set yet");
         return 0.0;
      }

      for (int i = 0; i < polyhedraFrameBasisVectors.length; i++)
      {
         polyhedraFrameBasisVectors[i].setIncludingFrame(ReferenceFrame.getWorldFrame(), polyhedraBasisVectors[i]);
         polyhedraFrameBasisVectors[i].changeFrame(zOutContactFrame);
         polyhedraFrameBasisVectors[i].get(0, i, basisVectorMatrix);
      }

      CommonOps_DDRM.multTransA(contactPointJacobian, basisVectorMatrix, JTF);
      CommonOps_DDRM.multInner(JTF, Q);

      CommonOps_DDRM.multTransA(JTF, jointspaceResidual, f);
      CommonOps_DDRM.scale(-1.0, f);

      try
      {
         qpSolver.solve(Q, f, Aeq, beq, Ain, bin, rho, true);
      }
      catch (NoConvergenceException e)
      {
         LogTools.debug("No convergence exception");
         return Double.MAX_VALUE;
      }

      CommonOps_DDRM.mult(basisVectorMatrix, rho, estimatedForce);
      return qpSolver.getCost();
   }

   public DMatrixRMaj getEstimatedForce()
   {
      return estimatedForce;
   }
}
