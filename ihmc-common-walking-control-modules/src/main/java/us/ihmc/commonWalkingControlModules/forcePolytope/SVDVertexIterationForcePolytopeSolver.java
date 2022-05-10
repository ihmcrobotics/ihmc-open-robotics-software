package us.ihmc.commonWalkingControlModules.forcePolytope;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.hash.TIntObjectHashMap;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.dense.row.linsol.svd.SolvePseudoInverseSvd_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;

/**
 * Solves for force polytope which is robust to singularities.
 * The SVD of the jacobian is computed and the non-singular columns of V are used as a base for the Image of J^T
 * The vertices of the force polytope are solved using these basis vectors along with torque limits
 *
 * See "Force polytope and force ellipsoid for redundant manipulators"
 * https://doi.org/10.1002/(SICI)1097-4563(199708)14:8<613::AID-ROB3>3.0.CO;2-P
 */
class SVDVertexIterationForcePolytopeSolver implements ForcePolytopeSolver
{
   // TODO tune
   private static final boolean debug = false;

   private final SvdImplicitQrDecompose_DDRM svdSolver = new SvdImplicitQrDecompose_DDRM(false, true, true, false);
   private final SolvePseudoInverseSvd_DDRM psuedoInverseSolver = new SolvePseudoInverseSvd_DDRM();
   private final LinearSolverDense<DMatrixRMaj> solver;

   private final TDoubleArrayList singularValues = new TDoubleArrayList();
   private final int dofs;
   private final DMatrixRMaj A = new DMatrixRMaj(0);
   private final DMatrixRMaj Asub = new DMatrixRMaj(0);
   private final DMatrixRMaj b = new DMatrixRMaj(0);
   private final DMatrixRMaj V = new DMatrixRMaj(0);
   private final DMatrixRMaj I;
   private final DMatrixRMaj x = new DMatrixRMaj(0);
   private final DMatrixRMaj jacobianTranspose = new DMatrixRMaj(0);
   private final DMatrixRMaj jacobianTransposeInv = new DMatrixRMaj(0);
   private final DMatrixRMaj tau = new DMatrixRMaj(0);
   private final DMatrixRMaj f = new DMatrixRMaj(0);
   private final DMatrixRMaj alpha = new DMatrixRMaj(0);

   private final TIntObjectHashMap<TIntArrayList> permutationMap = new TIntObjectHashMap<>();
   private static final int nullIndex = -1;
   private int[] AtoASub;
   private int[] ASubToA;

   public SVDVertexIterationForcePolytopeSolver(int dofs)
   {
      this.dofs = dofs;
      jacobianTranspose.reshape(dofs, 3);
      jacobianTransposeInv.reshape(3, dofs);
      Asub.reshape(2 * dofs, 2 * dofs);
      b.reshape(2 * dofs, 1);
      I = CommonOps_DDRM.identity(dofs);
      solver = LinearSolverFactory_DDRM.linear(2 * dofs);
      tau.reshape(dofs, 1);
      f.reshape(3, 1);
      psuedoInverseSolver.setThreshold(singularValueThreshold);
   }

   @Override
   public void solve(DMatrixRMaj jacobian, DMatrixRMaj tauLowerLimit, DMatrixRMaj tauUpperLimit, ConvexPolytope3D polytopeToPack)
   {
      polytopeToPack.clear();

      if (!svdSolver.decompose(jacobian))
      {
         return;
      }

      CommonOps_DDRM.transpose(jacobianTranspose, jacobian);
      psuedoInverseSolver.setA(jacobianTranspose);
      psuedoInverseSolver.invert(jacobianTransposeInv);

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

      A.zero();
      A.reshape(2 * dofs, 2 * dofs + numSingularValues);
      x.reshape(2 * dofs, 1);
      MatrixTools.setMatrixBlock(A, 0, 0, V, 0, 0, V.getNumRows(), numSingularValues, 1.0);
      MatrixTools.setMatrixBlock(A, V.getNumRows(), 0, V, 0, 0, V.getNumRows(), numSingularValues, -1.0);
      MatrixTools.setMatrixBlock(A, 0, numSingularValues, I, 0, 0, I.getNumRows(), I.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(A, V.getNumRows(), dofs + numSingularValues, I, 0, 0, I.getNumRows(), I.getNumCols(), 1.0);

      Asub.zero();
      MatrixTools.setMatrixBlock(Asub, 0, 0, V, 0, 0, V.getNumRows(), numSingularValues, 1.0);
      MatrixTools.setMatrixBlock(Asub, V.getNumRows(), 0, V, 0, 0, V.getNumRows(), numSingularValues, -1.0);

      for (int i = 0; i < dofs; i++)
      {
         b.set(i, 0, tauLowerLimit.get(i, 0));
         b.set(i + dofs, 0, tauUpperLimit.get(i, 0));
      }

      TIntArrayList permutations;
      if (permutationMap.containsKey(numSingularValues))
      {
         permutations = permutationMap.get(numSingularValues);
      }
      else
      {
         permutations = computePermutations(numSingularValues);
         permutationMap.put(numSingularValues, permutations);
      }

      AtoASub = new int[A.getNumCols()];
      ASubToA = new int[A.getNumCols() - numSingularValues];
      alpha.reshape(numSingularValues, 1);

      for (int i = 0; i < numSingularValues; i++)
      {
         AtoASub[i] = i;
         ASubToA[i] = i;
      }

      for (int i = 0; i < permutations.size(); i++)
      {
         int permutation = permutations.get(i);
         packIndices(numSingularValues, permutation);

         for (int j = 0; j < 2 * dofs; j++)
         {
            int aSubIndex = AtoASub[numSingularValues + j];
            if (aSubIndex == nullIndex)
               continue;

            MatrixTools.setMatrixBlock(Asub, 0, aSubIndex, A, 0, numSingularValues + j, A.getNumRows(), 1, 1.0);
         }

         solver.setA(Asub);
         solver.solve(b, x);

         boolean isValid = true;
         for (int k = numSingularValues; k < 2 * dofs; k++)
         {
            if (x.get(k) < 0.0)
            {
               isValid = false;
               break;
            }
         }

         if (isValid)
         {
            tau.zero();
            for (int k = 0; k < numSingularValues; k++)
            {
               alpha.set(k, 0, x.get(k, 0));
            }

            CommonOps_DDRM.mult(V, alpha, tau);
            CommonOps_DDRM.mult(jacobianTransposeInv, tau, f);

            Point3D vertex = new Point3D();
            vertex.set(f);
            if (polytopeToPack.signedDistance(vertex) > 1e-3)
            {
               polytopeToPack.addVertex(vertex);

               if (debug)
                  LogTools.info("Adding vertex \t" + vertex);
            }
         }
      }
   }

   private void packIndices(int numSingularValues, int permutation)
   {
      int counter = 0;
      for (int j = 0; j < 2 * dofs; j++)
      {
         if ((1 << j & permutation) > 0)
         {
            counter++;
            AtoASub[numSingularValues + j] = nullIndex;
         }
         else
         {
            AtoASub[numSingularValues + j] = numSingularValues + j - counter;
            ASubToA[numSingularValues + j - counter] = numSingularValues + j;
         }
      }
   }

   private TIntArrayList computePermutations(int m)
   {
      TIntArrayList permutations = new TIntArrayList();
      int maxIndex = 1 << (2 * dofs);
      int counter = 1;

      while (counter < maxIndex)
      {
         if (countOnesInBinary(counter, 2 * dofs) == m)
            permutations.add(counter);
         counter++;
      }

      return permutations;
   }

   private static int countOnesInBinary(int n, int maxDecimalsToCheck)
   {
      int digits = 0;
      for (int i = 0; i < 2 * maxDecimalsToCheck; i++)
      {
         if (((1 << i) & n) > 0)
            digits++;
      }
      return digits;
   }
}
