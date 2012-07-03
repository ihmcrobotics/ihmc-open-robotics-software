package us.ihmc.commonWalkingControlModules.kinematics;

import org.ejml.UtilEjml;
import org.ejml.alg.dense.decomposition.DecompositionFactory;
import org.ejml.alg.dense.decomposition.SingularValueDecomposition;
import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;
import org.ejml.ops.SingularOps;

import us.ihmc.utilities.screwTheory.JacobianSolver;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class SingularityRobustJacobianSolver implements JacobianSolver
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable switchingDeterminant;
   private final DoubleYoVariable jacobianDeterminant;
   private final DoubleYoVariable nullspaceGain;
   private final SingularValueDecomposition<DenseMatrix64F> svd;
   private final DenseMatrix64F u;
   private final int degenerateRank;
   private final int matrixSize;
   private final DenseMatrix64F columnSpace;
   private final DenseMatrix64F nullspace;
   private final DenseMatrix64F vStar;
   private final DenseMatrix64F uStar;

   private final DenseMatrix64F projectedAcceleration;
   private final DenseMatrix64F vStarTaskAcceleration;

   private final DenseMatrix64F iMinusNNT;
   private final SolvePseudoInverseSvd iMinusNNTSolver;
   private final DenseMatrix64F taskJointAcceleration;
   private final DenseMatrix64F projectedUnachievableTaskAcceleration;
   private final DenseMatrix64F nullspaceJointAcceleration;

   public SingularityRobustJacobianSolver(String namePrefix, int matrixSize)
   {
      this.matrixSize = matrixSize;
      this.registry = new YoVariableRegistry(namePrefix + "JacobianSolver");
      this.switchingDeterminant = new DoubleYoVariable("switchingDeterminant", registry);
      this.jacobianDeterminant = new DoubleYoVariable("jacobianDeterminant", registry);
      this.nullspaceGain = new DoubleYoVariable("nullspaceGain", registry);
      this.svd = DecompositionFactory.svd(matrixSize, matrixSize);
      this.u = new DenseMatrix64F(matrixSize, matrixSize);
      this.degenerateRank = matrixSize - 1;

      columnSpace = new DenseMatrix64F(matrixSize, degenerateRank);
      nullspace = new DenseMatrix64F(matrixSize, matrixSize - degenerateRank);
      vStar = new DenseMatrix64F(matrixSize, degenerateRank);
      uStar = new DenseMatrix64F(matrixSize, matrixSize - degenerateRank);
      projectedAcceleration = new DenseMatrix64F(degenerateRank, 1);
      vStarTaskAcceleration = new DenseMatrix64F(matrixSize, 1);
      iMinusNNT = new DenseMatrix64F(matrixSize, matrixSize);
      iMinusNNTSolver = new SolvePseudoInverseSvd(matrixSize, matrixSize);
      taskJointAcceleration = new DenseMatrix64F(matrixSize, 1);
      projectedUnachievableTaskAcceleration = new DenseMatrix64F(matrixSize - degenerateRank, 1);
      nullspaceJointAcceleration = new DenseMatrix64F(matrixSize, 1);
   }

   public void solve(DenseMatrix64F solutionToPack, DenseMatrix64F jacobianMatrix, DenseMatrix64F vector)
   {
      double determinant = CommonOps.det(jacobianMatrix);
      double switchingDeterminant = this.switchingDeterminant.getDoubleValue();
      double nullspaceMultiplier = nullspaceGain.getDoubleValue() * (switchingDeterminant - determinant);

      if (Math.abs(determinant) > switchingDeterminant)
      {
         double previousDeterminant = jacobianDeterminant.getDoubleValue();
         if (Math.abs(previousDeterminant) <= switchingDeterminant)
         {
            computeDegenerate(solutionToPack, jacobianMatrix, vector, nullspaceMultiplier);

            // TODO:
            // reset trajectory generator acc to current desired acc
            // reset trajectory generator pos, vel to have no component along singular direction
         }
         else
         {
            CommonOps.solve(jacobianMatrix, vector, solutionToPack);
         }
      }
      else
      {
         computeDegenerate(solutionToPack, jacobianMatrix, vector, nullspaceMultiplier);
      }

      jacobianDeterminant.set(determinant);
   }
   
   public void setSwitchingDeterminant(double switchingDeterminant)
   {
      this.switchingDeterminant.set(switchingDeterminant);
   }

   public void setNullspaceGain(double nullspaceGain)
   {
      this.nullspaceGain.set(nullspaceGain);
   }
   
   private void computeDegenerate(DenseMatrix64F jointAccelerations, DenseMatrix64F jacobian, DenseMatrix64F taskAcceleration, double nullspaceMultiplier)
   {
      svd.decompose(jacobian);
      DenseMatrix64F uTranspose = svd.getU(true);
      DenseMatrix64F sigma = svd.getW(null);
      DenseMatrix64F v = svd.getV(false);
      CommonOps.transpose(uTranspose, u);
      SingularOps.descendingOrder(u, false, sigma, v, false);

      CommonOps.extract(u, 0, matrixSize, 0, degenerateRank, columnSpace, 0, 0);
      CommonOps.extract(v, 0, matrixSize, degenerateRank, matrixSize, nullspace, 0, 0);
      CommonOps.extract(v, 0, matrixSize, 0, degenerateRank, vStar, 0, 0);
      CommonOps.extract(u, 0, matrixSize, degenerateRank, matrixSize, uStar, 0, 0);

      DenseMatrix64F augmentedTaskJointAcceleration = computeAugmentedTaskJointAccelerations(jacobian, taskAcceleration, sigma, columnSpace, nullspace, vStar);
      DenseMatrix64F nullspaceJointAcceleration = computeNullspaceJointAccelerations(taskAcceleration, nullspaceMultiplier, nullspace, uStar);
      CommonOps.add(augmentedTaskJointAcceleration, nullspaceJointAcceleration, jointAccelerations);
   }

   private DenseMatrix64F computeAugmentedTaskJointAccelerations(DenseMatrix64F jacobian, DenseMatrix64F taskAcceleration, DenseMatrix64F sigma,
           DenseMatrix64F columnSpace, DenseMatrix64F nullspace, DenseMatrix64F vStar)
   {
      CommonOps.multTransA(columnSpace, taskAcceleration, projectedAcceleration);

      for (int i = 0; i < degenerateRank; i++)
      {
         projectedAcceleration.times(i, 1.0 / sigma.get(i, i));
      }

      CommonOps.mult(vStar, projectedAcceleration, vStarTaskAcceleration);

      CommonOps.multOuter(nullspace, iMinusNNT);
      CommonOps.scale(-1.0, iMinusNNT);
      addDiagonal(iMinusNNT, 1.0);

      double oldEps = UtilEjml.EPS;
      UtilEjml.EPS = 0.5;    // this is OK because singular values should be either 0 or 1 anyway; f numerical issues
      iMinusNNTSolver.setA(iMinusNNT);
      iMinusNNTSolver.solve(vStarTaskAcceleration, taskJointAcceleration);
      UtilEjml.EPS = oldEps;

      return taskJointAcceleration;
   }

   private DenseMatrix64F computeNullspaceJointAccelerations(DenseMatrix64F taskAcceleration, double nullspaceMultiplier, DenseMatrix64F nullspace,
           DenseMatrix64F uStar)
   {
      CommonOps.multTransA(uStar, taskAcceleration, projectedUnachievableTaskAcceleration);

      CommonOps.mult(nullspace, projectedUnachievableTaskAcceleration, nullspaceJointAcceleration);
      double norm = NormOps.normP2(nullspaceJointAcceleration);
      double normEpsilon = 1e-3;
      if (norm > normEpsilon)
         CommonOps.scale(1.0 / norm, nullspaceJointAcceleration);
      CommonOps.scale(nullspaceMultiplier, nullspaceJointAcceleration);

      return nullspaceJointAcceleration;
   }

   private void addDiagonal(DenseMatrix64F mat, double s)
   {
      int n = Math.max(mat.getNumRows(), mat.getNumCols());
      for (int i = 0; i < n; i++)
      {
         mat.add(i, i, s);
      }
   }
}
