package us.ihmc.commonWalkingControlModules.forcePolytope;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.exceptions.NoConvergenceException;
import us.ihmc.convexOptimization.quadraticProgram.QuadProgSolver;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.matrixlib.MatrixTools;

import java.util.ArrayList;
import java.util.List;

class QPSupportingVertexForcePolytopeSolver implements ForcePolytopeSolver
{
   private final int dofs;

   private final DMatrixRMaj jacobianTranspose = new DMatrixRMaj(0);
   private final DMatrixRMaj qpQuadraticCost = new DMatrixRMaj(0);
   private final DMatrixRMaj qpLinearCost = new DMatrixRMaj(0);
   private final DMatrixRMaj qpInequalityA = new DMatrixRMaj(0);
   private final DMatrixRMaj qpInequalityB = new DMatrixRMaj(0);
   private final DMatrixRMaj qpSolution = new DMatrixRMaj(0);

   private final DMatrixRMaj identity = new DMatrixRMaj(0);
   private final DMatrixRMaj jacobianOuterProduct = new DMatrixRMaj(0);

   private final List<Point3D> vertices = new ArrayList<>();

   public QPSupportingVertexForcePolytopeSolver(int dofs)
   {
      this.dofs = dofs;

      jacobianTranspose.reshape(dofs, 3);
      int qpSize = 3 + dofs;
      qpQuadraticCost.reshape(qpSize, qpSize);
      qpLinearCost.reshape(qpSize, 1);
      qpInequalityA.reshape(2 * dofs, qpSize);
      qpInequalityB.reshape(2 * dofs, 1);
      qpSolution.reshape(qpSize, 1);
      identity.reshape(dofs, dofs);
      CommonOps_DDRM.setIdentity(identity);
      jacobianOuterProduct.reshape(3, 3);
   }

   @Override
   public void solve(DMatrixRMaj jacobian, DMatrixRMaj tauLowerLimit, DMatrixRMaj tauUpperLimit, ConvexPolytope3D polytopeToPack)
   {
      polytopeToPack.clear();

      // setup as quadratic program
      CommonOps_DDRM.multOuter(jacobian, jacobianOuterProduct);
      MatrixTools.setMatrixBlock(qpQuadraticCost, 0, 0, identity, 0, 0, dofs, dofs, 1.0);
      MatrixTools.setMatrixBlock(qpQuadraticCost, 0, dofs, jacobianTranspose, 0, 0, jacobianTranspose.getNumRows(), jacobianTranspose.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(qpQuadraticCost, dofs, 0, jacobian, 0, 0, jacobian.getNumRows(), jacobian.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(qpQuadraticCost, dofs, dofs, jacobianOuterProduct, 0, 0, jacobianOuterProduct.getNumRows(), jacobianOuterProduct.getNumCols(), 1.0);

      double quadCostScale = 100.0;
      CommonOps_DDRM.scale(quadCostScale, qpQuadraticCost);

      MatrixTools.setMatrixBlock(qpInequalityA, 0, 0, identity, 0, 0, identity.getNumRows(), identity.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(qpInequalityA, identity.getNumRows(), 0, identity, 0, 0, identity.getNumRows(), identity.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(qpInequalityB, 0, 0, tauLowerLimit, 0, 0, tauLowerLimit.getNumRows(), tauLowerLimit.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(qpInequalityB, identity.getNumRows(), 0, tauUpperLimit, 0, 0, tauUpperLimit.getNumRows(), tauUpperLimit.getNumCols(), 1.0);

      for (int i = 0; i < directionsToOptimize.length; i++)
      {
         Point3D direction = directionsToOptimize[i];

         direction.get(qpLinearCost);
         CommonOps_DDRM.scale(-1.0, qpLinearCost);

         for (int j = 0; j < 3; j++)
         {
            double forceDampingTerm = 1e-6;
            qpQuadraticCost.add(dofs + j, dofs + j, forceDampingTerm);
         }

         //            SimpleEfficientActiveSetQPSolver qpSolver = new SimpleEfficientActiveSetQPSolver();
         //            SimpleEfficientActiveSetQPSolverWithInactiveVariables qpSolver = new SimpleEfficientActiveSetQPSolverWithInactiveVariables();

         //            qpSolver.setUseWarmStart(false);
         //            qpSolver.setMaxNumberOfIterations(1000);
         //            qpSolver.setConvergenceThreshold(1e-5);
         //            qpSolver.clear();
         //            qpSolver.resetActiveSet();
         //            qpSolver.setQuadraticCostFunction(qpQuadraticCost, qpLinearCost);
         //            qpSolver.setLinearInequalityConstraints(qpInequalityA, qpInequalityB);
         //            int iterations = qpSolver.solve(qpSolution);

         QuadProgSolver qpSolver = new QuadProgSolver(3 + dofs, 0, 2 * dofs);

         try
         {
            qpSolver.solve(qpQuadraticCost, qpLinearCost, new DMatrixRMaj(0, 3 + dofs), new DMatrixRMaj(0, 0), qpInequalityA, qpInequalityB, qpSolution, false);
         }
         catch (NoConvergenceException e)
         {
            continue;
         }

         System.out.println(qpSolution);

         Point3D vertex = new Point3D();
         vertex.set(dofs, qpSolution);

         if (!vertex.containsNaN())
         {
            vertices.add(vertex);

            try
            {
               polytopeToPack.addVertex(vertex);
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }

      }
   }

   @Override
   public List<Point3D> getVertices()
   {
      return vertices;
   }
}
