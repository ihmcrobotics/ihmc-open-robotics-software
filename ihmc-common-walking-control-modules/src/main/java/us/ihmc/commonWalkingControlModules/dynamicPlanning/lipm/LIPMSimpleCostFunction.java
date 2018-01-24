package us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.trajectoryOptimization.DefaultDiscreteState;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.LIPMDynamics.stateVectorSize;
import static us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.LIPMDynamics.controlVectorSize;

public class LIPMSimpleCostFunction implements LQTrackingCostFunction<DefaultDiscreteState>
{
   static final double qX = 1e-6;
   static final double qY = 1e-6;
   static final double qZ = 1e1;
   static final double qXDot = 1e-6;
   static final double qYDot = 1e-6;
   static final double qZDot = 1e-6;

   static final double rXf = 1e2;
   static final double rYf = 1e2;
   static final double rFz = 1e-6;

   private final DenseMatrix64F Q = new DenseMatrix64F(stateVectorSize, stateVectorSize);
   private final DenseMatrix64F R = new DenseMatrix64F(controlVectorSize, controlVectorSize);

   /**
    * This is a cost of the form 0.5 (X - X_d)^T Q (X - X_d) + 0.5 (U - U_d)^T R (U - U_d)
    */

   public LIPMSimpleCostFunction()
   {
      Q.set(0, 0, qX);
      Q.set(1, 1, qY);
      Q.set(2, 2, qZ);
      Q.set(3, 3, qXDot);
      Q.set(4, 4, qYDot);
      Q.set(5, 5, qZDot);

      R.set(0, 0, rXf);
      R.set(1, 1, rYf);
      R.set(2, 2, rFz);
   }

   private DenseMatrix64F tempStateMatrix = new DenseMatrix64F(stateVectorSize, 1);
   private DenseMatrix64F tempControlMatrix = new DenseMatrix64F(controlVectorSize, 1);
   private DenseMatrix64F tempWX = new DenseMatrix64F(stateVectorSize, 1);
   private DenseMatrix64F tempWU = new DenseMatrix64F(controlVectorSize, 1);

   @Override
   public double getCost(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                         DenseMatrix64F desiredStateVector, DenseMatrix64F constants)
   {
      CommonOps.subtract(controlVector, desiredControlVector, tempControlMatrix);
      CommonOps.subtract(stateVector, desiredStateVector, tempStateMatrix);

      DiagonalMatrixTools.preMult(Q, tempStateMatrix, tempWX);
      DiagonalMatrixTools.preMult(R, tempControlMatrix, tempWU);

      return CommonOps.dot(tempControlMatrix, tempWU) + CommonOps.dot(tempStateMatrix, tempWX);
   }

   /** L_x(X_k, U_k) */
   @Override
   public void getCostStateGradient(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                                    DenseMatrix64F desiredStateVector, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      CommonOps.subtract(stateVector, desiredStateVector, tempStateMatrix);
      DiagonalMatrixTools.preMult(Q, tempStateMatrix, matrixToPack);
      CommonOps.scale(2.0, matrixToPack);
   }

   /** L_u(X_k, U_k) */
   @Override
   public void getCostControlGradient(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                                      DenseMatrix64F desiredStateVector, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      CommonOps.subtract(controlVector, desiredControlVector, tempControlMatrix);
      DiagonalMatrixTools.preMult(R, tempControlMatrix, matrixToPack);
      CommonOps.scale(2.0, matrixToPack);
   }

   /** L_xx(X_k, U_k) */
   @Override
   public void getCostStateHessian(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      CommonOps.scale(2.0, Q, matrixToPack);
   }

   /** L_uu(X_k, U_k) */
   @Override
   public void getCostControlHessian(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      CommonOps.scale(2.0, R, matrixToPack);
   }

   /** L_ux(X_k, U_k) */
   @Override
   public void getCostStateGradientOfControlGradient(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector,
                                                     DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(controlVectorSize, stateVectorSize);
   }

   /** L_xu(X_k, U_k) */
   @Override
   public void getCostControlGradientOfStateGradient(DefaultDiscreteState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector,
                                                     DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(stateVectorSize, controlVectorSize);
   }
}
