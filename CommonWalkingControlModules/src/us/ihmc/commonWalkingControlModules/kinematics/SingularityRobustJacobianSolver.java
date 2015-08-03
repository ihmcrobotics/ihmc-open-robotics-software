package us.ihmc.commonWalkingControlModules.kinematics;

import org.ejml.UtilEjml;
import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.SingularOps;


import us.ihmc.utilities.math.MathTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.JacobianSolver;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


/*
 * Use DampedLeastSquaresJacobianSolver in combination with NullspaceCalculator instead
 */
@Deprecated
public class SingularityRobustJacobianSolver implements JacobianSolver
{
   private final YoVariableRegistry registry;
   private final DampedLeastSquaresJacobianSolver dampedLeastSquaresJacobianSolver;
   private final DoubleYoVariable switchingDeterminant;
   private final DoubleYoVariable jacobianDeterminant;
   private final DoubleYoVariable nullspaceMultiplier;
   private final BooleanYoVariable leavingSingularRegion;
   private final SingularValueDecomposition<DenseMatrix64F> svd;
   private final int degenerateRank;
   private final DenseMatrix64F nullspace;

   private final DenseMatrix64F taskJointAcceleration;
   private final DenseMatrix64F nullspaceJointAcceleration;
   private final int indexToUseForSign;
   private final int sign;

   private final DenseMatrix64F vStarTaskAcceleration;
   private final DenseMatrix64F iMinusNNT;
   private final SolvePseudoInverseSvd iMinusNNTSolver;

   public SingularityRobustJacobianSolver(String namePrefix, int matrixSize, int indexToUseForSign, int sign, YoVariableRegistry parentRegistry)
   {
      MathTools.checkIfInRange(matrixSize, 1, Integer.MAX_VALUE);
      MathTools.checkIfInRange(indexToUseForSign, 0, matrixSize - 1);
      MathTools.checkIfInRange(sign, -1, 1);
      if (sign == 0)
         throw new RuntimeException("sign == 0");

      this.registry = new YoVariableRegistry(namePrefix + "JacobianSolver");
      this.dampedLeastSquaresJacobianSolver = new DampedLeastSquaresJacobianSolver(namePrefix + "DampedLS", matrixSize, registry);
      this.switchingDeterminant = new DoubleYoVariable("switchingDeterminant", registry);
      this.jacobianDeterminant = new DoubleYoVariable("jacobianDeterminant", registry);
      this.nullspaceMultiplier = new DoubleYoVariable("nullspaceMultiplier", registry);
      this.leavingSingularRegion = new BooleanYoVariable("leavingSingularRegion", registry);
      this.svd = DecompositionFactory.svd(matrixSize, matrixSize, false, true, false);
      this.degenerateRank = matrixSize - 1;
      this.indexToUseForSign = indexToUseForSign;
      this.sign = sign;

      vStarTaskAcceleration = new DenseMatrix64F(matrixSize, 1);
      iMinusNNT = new DenseMatrix64F(matrixSize, matrixSize);
      iMinusNNTSolver = new SolvePseudoInverseSvd(matrixSize, matrixSize);

      nullspace = new DenseMatrix64F(matrixSize, matrixSize - degenerateRank);
      taskJointAcceleration = new DenseMatrix64F(matrixSize, 1);
      nullspaceJointAcceleration = new DenseMatrix64F(matrixSize, 1);

      parentRegistry.addChild(registry);
   }

   public void setNullspaceMultiplier(double nullspaceMultiplier)
   {
      this.nullspaceMultiplier.set(nullspaceMultiplier);
   }

   public boolean leavingSingularRegion()
   {
      return leavingSingularRegion.getBooleanValue();
   }

   public boolean inSingularRegion()
   {
      return Math.abs(jacobianDeterminant.getDoubleValue()) < switchingDeterminant.getDoubleValue();
   }

   public void inverseSolveNonDegenerate(DenseMatrix64F taskSpaceVectorToPack, DenseMatrix64F jointSpaceVector)
   {
      dampedLeastSquaresJacobianSolver.inverseSolve(taskSpaceVectorToPack, jointSpaceVector);
   }

   public void solve(DenseMatrix64F solutionToPack, DenseMatrix64F vector)
   {
      if (inSingularRegion() || leavingSingularRegion())
      {
         computeDegenerate(solutionToPack, vector, nullspaceMultiplier.getDoubleValue());
      }
      else
      {
         dampedLeastSquaresJacobianSolver.solve(solutionToPack, vector);
      }
   }

   public void setSwitchingDeterminant(double switchingDeterminant)
   {
      this.switchingDeterminant.set(switchingDeterminant);
   }

   public void setAlpha(double alpha)
   {
      dampedLeastSquaresJacobianSolver.setAlpha(alpha);
   }

   private void computeDegenerate(DenseMatrix64F jointAccelerations, DenseMatrix64F taskAcceleration, double nullspaceMultiplier)
   {
      DenseMatrix64F jacobian = dampedLeastSquaresJacobianSolver.getJacobianMatrix();
      svd.decompose(jacobian);
      SingularOps.nullSpace(svd, nullspace, UtilEjml.EPS);

      DenseMatrix64F augmentedTaskJointAccelerations = computeAugmentedTaskJointAccelerations(jacobian, taskAcceleration, nullspace,
                                                          nullspaceMultiplier != 0.0);
      DenseMatrix64F nullspaceJointAccelerations = computeNullspaceJointAccelerations(nullspaceMultiplier, nullspace);

      CommonOps.add(augmentedTaskJointAccelerations, nullspaceJointAccelerations, jointAccelerations);
   }

   private DenseMatrix64F computeAugmentedTaskJointAccelerations(DenseMatrix64F jacobian, DenseMatrix64F taskAcceleration, DenseMatrix64F nullspace,
           boolean removeNullspaceComponent)
   {
      dampedLeastSquaresJacobianSolver.solve(vStarTaskAcceleration, taskAcceleration);

      if (removeNullspaceComponent)
      {
         CommonOps.multOuter(nullspace, iMinusNNT);
         CommonOps.scale(-1.0, iMinusNNT);
         MatrixTools.addDiagonal(iMinusNNT, 1.0);

         double oldEps = UtilEjml.EPS;
         UtilEjml.EPS = 0.5 / Math.max(jacobian.getNumRows(), jacobian.getNumCols());    // this is OK because singular values should be either 0 or 1 anyway, since columns of nullspace are orthonormal; fixes numerical issues
         iMinusNNTSolver.setA(iMinusNNT);
         iMinusNNTSolver.solve(vStarTaskAcceleration, taskJointAcceleration);
         UtilEjml.EPS = oldEps;
      }
      else
      {
         taskJointAcceleration.set(vStarTaskAcceleration);
      }

      return taskJointAcceleration;
   }

   private DenseMatrix64F computeNullspaceJointAccelerations(double nullspaceMultiplier, DenseMatrix64F nullspace)
   {
      nullspaceJointAcceleration.set(nullspace);
      double sign = this.sign * Math.signum(nullspace.get(indexToUseForSign, 0));
      CommonOps.scale(nullspaceMultiplier * sign, nullspaceJointAcceleration);

      return nullspaceJointAcceleration;
   }

   public void setJacobian(DenseMatrix64F jacobianMatrix)
   {
      dampedLeastSquaresJacobianSolver.setJacobian(jacobianMatrix);
      double determinant = CommonOps.det(jacobianMatrix);
      double switchingDeterminant = this.switchingDeterminant.getDoubleValue();
      double previousDeterminant = jacobianDeterminant.getDoubleValue();
      leavingSingularRegion.set(Math.abs(determinant) > switchingDeterminant && Math.abs(previousDeterminant) <= switchingDeterminant);

      jacobianDeterminant.set(determinant);
   }
}
