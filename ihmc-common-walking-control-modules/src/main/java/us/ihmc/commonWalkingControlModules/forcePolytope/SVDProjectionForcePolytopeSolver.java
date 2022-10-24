package us.ihmc.commonWalkingControlModules.forcePolytope;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.linsol.svd.SolvePseudoInverseSvd_DDRM;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

class SVDProjectionForcePolytopeSolver implements ForcePolytopeSolver
{
   private final int dofs;
   private final DMatrixRMaj jacobianTranspose = new DMatrixRMaj(0);
   private final DMatrixRMaj jacobianTransposeInv = new DMatrixRMaj(0);
   private final DMatrixRMaj tau = new DMatrixRMaj(0);
   private final DMatrixRMaj f = new DMatrixRMaj(3, 1);
   private final SolvePseudoInverseSvd_DDRM psuedoInverseSolver = new SolvePseudoInverseSvd_DDRM();

   public SVDProjectionForcePolytopeSolver(int dofs)
   {
      this.dofs = dofs;
      jacobianTranspose.reshape(dofs, 3);
      jacobianTransposeInv.reshape(3, dofs);
      psuedoInverseSolver.setThreshold(singularValueThreshold);
      tau.reshape(dofs, 1);
   }

   @Override
   public void solve(DMatrixRMaj jacobian, DMatrixRMaj tauLowerLimit, DMatrixRMaj tauUpperLimit, ConvexPolytope3D polytopeToPack)
   {
      polytopeToPack.clear();
      CommonOps_DDRM.transpose(jacobian, jacobianTranspose);
      psuedoInverseSolver.setA(jacobianTranspose);
      psuedoInverseSolver.invert(jacobianTransposeInv);

      int permutationMax = 1 << dofs;
      for (int permutation = 0; permutation < permutationMax; permutation++)
      {
         for (int dof = 0; dof < dofs; dof++)
         {
            if ((1 << dof & permutation) > 0)
            {
               tau.set(dof, 0, tauLowerLimit.get(dof, 0));
            }
            else
            {
               tau.set(dof, 0, tauUpperLimit.get(dof, 0));
            }
         }

         CommonOps_DDRM.mult(jacobianTransposeInv, tau, f);

         Point3D vertex = new Point3D();
         vertex.set(f);
         polytopeToPack.addVertex(vertex);
      }
   }
}
