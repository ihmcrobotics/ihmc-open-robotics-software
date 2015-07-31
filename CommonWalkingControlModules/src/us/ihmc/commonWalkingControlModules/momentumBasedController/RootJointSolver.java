package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.LinkedHashMap;

import org.ejml.alg.dense.mult.MatrixDimensionException;
import org.ejml.data.D1Matrix64F;
import org.ejml.data.DenseMatrix64F;
import org.ejml.data.RowD1Matrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;

import us.ihmc.utilities.math.linearAlgebra.MatrixTools;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

public class RootJointSolver
{
   private final int size = Momentum.SIZE;
   private final DenseMatrix64F bHat = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F aHatRootN = new DenseMatrix64F(size, size);    // will reshape later
   private final DenseMatrix64F f = new DenseMatrix64F(size, size);
   private final DenseMatrix64F alpha2Beta2 = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F alpha2 = new DenseMatrix64F(size, 1);    // will reshape later
   private final DenseMatrix64F beta2 = new DenseMatrix64F(size, 1);    // will reshape later
   private final DenseMatrix64F aVdotRoot1 = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F hdot = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F orthogonalCheck = new DenseMatrix64F(size / 2, size / 2);    // oversized unless momentumSubspace and accelerationSubspace have 3 columns each
   private final LinearSolver<DenseMatrix64F> alpha2Beta2Solver = LinearSolverFactory.linear(size);

   public void solveAndSetRootJointAcceleration(DenseMatrix64F vdotRoot, DenseMatrix64F aHatRoot, DenseMatrix64F b, DenseMatrix64F T, DenseMatrix64F alpha1,
           DenseMatrix64F N, DenseMatrix64F beta1, SixDoFJoint rootJoint, LinkedHashMap<InverseDynamicsJoint, Boolean> jointAccelerationValidMap)
   {
      double epsilonOrthogonal = 1e-10;
      orthogonalCheck.reshape(N.getNumCols(), T.getNumCols());
      CommonOps.multTransA(N, T, orthogonalCheck);
      if (!MatrixFeatures.isConstantVal(orthogonalCheck, 0.0, epsilonOrthogonal))
         throw new RuntimeException("subspaces not orthogonal");

      int momentumSubspaceRank = N.getNumCols();
      int accelerationSubspaceRank = T.getNumCols();

      // aHatRootN
      aHatRootN.reshape(aHatRoot.getNumRows(), N.getNumCols());
      CommonOps.mult(aHatRoot, N, aHatRootN);

      // f
      CommonOps.insert(aHatRootN, f, 0, 0);
      MatrixTools.setMatrixBlock(f, 0, momentumSubspaceRank, T, 0, 0, T.numRows, T.numCols, -1.0);

      // vdot1
      checkDimensions(T, alpha1, vdotRoot);
      if (accelerationSubspaceRank == 0)    // handle EJML stupidity
         CommonOps.fill(vdotRoot, 0.0);
      else
         CommonOps.mult(T, alpha1, vdotRoot);

      // hdot1
      checkDimensions(N, beta1, hdot);
      if (momentumSubspaceRank == 0)    // handle EJML stupidity
         CommonOps.fill(hdot, 0.0);
      else
         CommonOps.mult(N, beta1, hdot);

      // aVdot1
      CommonOps.mult(aHatRoot, vdotRoot, aVdotRoot1);

      // bHat
      bHat.set(b);
      CommonOps.addEquals(bHat, hdot);
      CommonOps.subtractEquals(bHat, aVdotRoot1);

      // alpha2Beta2
      alpha2Beta2Solver.setA(f);
      alpha2Beta2Solver.solve(bHat, alpha2Beta2);

      // alpha2
      alpha2.reshape(momentumSubspaceRank, 1);
      CommonOps.extract(alpha2Beta2, 0, momentumSubspaceRank, 0, 1, alpha2, 0, 0);

      // beta2
      beta2.reshape(accelerationSubspaceRank, 1);
      CommonOps.extract(alpha2Beta2, momentumSubspaceRank, alpha2Beta2.getNumRows(), 0, 1, beta2, 0, 0);

      // vdotRoot
      if (momentumSubspaceRank > 0)    // handle EJML stupidity
         CommonOps.multAdd(N, alpha2, vdotRoot);

      // hdot
      if (accelerationSubspaceRank > 0)    // handle EJML stupidity
         CommonOps.multAdd(T, beta2, hdot);

      assert (areAccelerationsOK(aHatRoot, vdotRoot, b));

      rootJoint.setDesiredAcceleration(vdotRoot, 0);
      jointAccelerationValidMap.put(rootJoint, true);
   }

   private boolean areAccelerationsOK(RowD1Matrix64F aHatRoot, RowD1Matrix64F vdotRoot, D1Matrix64F b)
   {
      DenseMatrix64F check = new DenseMatrix64F(6, 1);
      CommonOps.mult(aHatRoot, vdotRoot, check);
      CommonOps.subtractEquals(check, hdot);
      CommonOps.subtractEquals(check, b);
      boolean ret = MatrixFeatures.isConstantVal(check, 0.0, 1e-7);
      return ret;
   }

   public DenseMatrix64F getRateOfChangeOfMomentum()
   {
      return hdot;
   }
   
   private static void checkDimensions(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c)
   {
      if (a.numCols != b.numRows)
         throw new MatrixDimensionException("The 'a' and 'b' matrices do not have compatible dimensions");
      else if ((a.numRows != c.numRows) || (b.numCols != c.numCols))
         throw new MatrixDimensionException("The results matrix does not have the desired dimensions");
   }
}
