package us.ihmc.commonWalkingControlModules.forcePolytope;

import gnu.trove.list.array.TDoubleArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import us.ihmc.convexOptimization.linearProgram.LinearProgramSolver;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;

class LPSupportingVertexForcePolytopeSolver implements ForcePolytopeSolver
{
   private final LinearProgramSolver lpSolver = new LinearProgramSolver();
   private final SvdImplicitQrDecompose_DDRM svdSolver = new SvdImplicitQrDecompose_DDRM(false, true, true, false);

   private final int dofs;
   private final TDoubleArrayList singularValues = new TDoubleArrayList();
   private final DMatrixRMaj V = new DMatrixRMaj(0);
   private final DMatrixRMaj alpha = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj tempDirectionToOptimize = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj jacobianTranspose = new DMatrixRMaj(0);
   private final DMatrixRMaj lpInequalityA = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj lpInequalityB = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj lpCost = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj lpSolution = new DMatrixRMaj(3, 1);

   public LPSupportingVertexForcePolytopeSolver(int dofs)
   {
      this.dofs = dofs;
      jacobianTranspose.reshape(dofs, 3);
      lpInequalityA.reshape(2 * dofs, 6);
      lpInequalityB.reshape(2 * dofs, 1);
   }

   @Override
   public void solve(DMatrixRMaj jacobian, DMatrixRMaj tauLowerLimit, DMatrixRMaj tauUpperLimit, ConvexPolytope3D polytopeToPack)
   {
      polytopeToPack.clear();
      CommonOps_DDRM.transpose(jacobian, jacobianTranspose);

      if (!svdSolver.decompose(jacobianTranspose))
      {
         return;
      }

      double[] singularValuesNonThresholded = svdSolver.getSingularValues();
      singularValues.clear();

      for (int i = 0; i < singularValuesNonThresholded.length; i++)
      {
         if (singularValuesNonThresholded[i] < singularValueThreshold)
            break;

         singularValues.add(singularValuesNonThresholded[i]);
      }

      int numSingularValues = singularValues.size();
      svdSolver.getV(V, true);
      V.reshape(numSingularValues, V.getNumCols());
      CommonOps_DDRM.transpose(V);

      // setup as linear program
      int rows = jacobianTranspose.getNumRows();
      int cols = jacobianTranspose.getNumCols();

      MatrixTools.setMatrixBlock(lpInequalityA, 0, 0, jacobianTranspose, 0, 0, rows, cols, 1.0);
      MatrixTools.setMatrixBlock(lpInequalityA, 0, cols, jacobianTranspose, 0, 0, rows, cols, -1.0);
      MatrixTools.setMatrixBlock(lpInequalityA, rows, 0, jacobianTranspose, 0, 0, rows, cols, -1.0);
      MatrixTools.setMatrixBlock(lpInequalityA, rows, cols, jacobianTranspose, 0, 0, rows, cols, 1.0);

      MatrixTools.setMatrixBlock(lpInequalityB, 0, 0, tauUpperLimit, 0, 0, dofs, 1, 1.0);
      MatrixTools.setMatrixBlock(lpInequalityB, rows, 0, tauLowerLimit, 0, 0, dofs, 1, -1.0);

      Tuple3DReadOnly[] directionsToOptimize;
      if (singularValues.size() == 3)
      {
         directionsToOptimize = ForcePolytopeSolver.directionsToOptimize;
      }
      else if (singularValues.size() == 2)
      {
         int numberOfPointsToGenerate = 50;
         directionsToOptimize = new Point3D[numberOfPointsToGenerate];
         for (int i = 0; i < numberOfPointsToGenerate; i++)
         {
            Point3D directionToOptimize = new Point3D();
            double theta = i * 2.0 * Math.PI / numberOfPointsToGenerate;
            alpha.set(0, 0, Math.cos(theta));
            alpha.set(1, 0, Math.sin(theta));
            CommonOps_DDRM.mult(V, alpha, tempDirectionToOptimize);
            directionToOptimize.set(tempDirectionToOptimize);
            directionsToOptimize[i] = directionToOptimize;
         }
      }
      else
      {
         return;
      }

      for (int i = 0; i < directionsToOptimize.length; i++)
      {
         Tuple3DReadOnly direction = directionsToOptimize[i];

         for (int j = 0; j < 3; j++)
         {
            lpCost.set(j, direction.getElement(j));
            lpCost.set(3 + j, -direction.getElement(j));
         }

         if (lpSolver.solve(lpCost, lpInequalityA, lpInequalityB, lpSolution))
         {
            Point3D vertex = new Point3D();
            vertex.set(lpSolution.get(0), lpSolution.get(1), lpSolution.get(2));
            vertex.sub(lpSolution.get(3), lpSolution.get(4), lpSolution.get(5));
            polytopeToPack.addVertex(vertex);
         }
      }
   }
}
