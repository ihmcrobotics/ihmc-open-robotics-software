package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SLIPTerminalCost implements LQTrackingCostFunction<SLIPState>
{
   private final DenseMatrix64F Q = new DenseMatrix64F(stateVectorSize, stateVectorSize);
   private final DenseMatrix64F R = new DenseMatrix64F(controlVectorSize, controlVectorSize);

   public SLIPTerminalCost()
   {
      Q.set(x, x, 1e10);
      Q.set(y, y, 1e10);
      Q.set(z, z, 1e10);
      Q.set(thetaX, thetaX, 1e10);
      Q.set(thetaY, thetaY, 1e10);
      Q.set(thetaZ, thetaZ, 1e10);
      Q.set(xDot, xDot, 1e10);
      Q.set(yDot, yDot, 1e10);
      Q.set(zDot, zDot, 1e10);
      Q.set(thetaXDot, thetaXDot, 1e10);
      Q.set(thetaYDot, thetaYDot, 1e10);
      Q.set(thetaZDot, thetaZDot, 1e10);

      R.set(fx, fx, 1e10);
      R.set(fy, fy, 1e10);
      R.set(fz, fz, 1e10);
      R.set(tauX, tauX, 1e10);
      R.set(tauY, tauY, 1e10);
      R.set(tauZ, tauZ, 1e10);
      R.set(xF, xF, 0e0);
      R.set(yF, yF, 0e0);
      R.set(k, k, 0e0);
   }

   private DenseMatrix64F tempStateMatrix = new DenseMatrix64F(stateVectorSize, 1);
   private DenseMatrix64F tempControlMatrix = new DenseMatrix64F(controlVectorSize, 1);
   private DenseMatrix64F tempWX = new DenseMatrix64F(stateVectorSize, 1);
   private DenseMatrix64F tempWU = new DenseMatrix64F(controlVectorSize, 1);

   @Override
   public double getCost(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                         DenseMatrix64F desiredStateVector)
   {
      CommonOps.subtract(controlVector, desiredControlVector, tempControlMatrix);
      CommonOps.subtract(stateVector, desiredStateVector, tempStateMatrix);

      DiagonalMatrixTools.preMult(Q, tempStateMatrix, tempWX);
      DiagonalMatrixTools.preMult(R, tempControlMatrix, tempWU);

      return CommonOps.dot(tempControlMatrix, tempWU) + CommonOps.dot(tempStateMatrix, tempWX);
   }

   /** L_x(X_k, U_k) */
   @Override
   public void getCostStateGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                                    DenseMatrix64F desiredStateVector, DenseMatrix64F matrixToPack)
   {
      CommonOps.subtract(stateVector, desiredStateVector, tempStateMatrix);
      DiagonalMatrixTools.preMult(Q, tempStateMatrix, matrixToPack);
   }

   /** L_u(X_k, U_k) */
   @Override
   public void getCostControlGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVecotr,
                                      DenseMatrix64F desiredStateVector, DenseMatrix64F matrixToPack)
   {
      CommonOps.subtract(controlVector, desiredControlVecotr, tempControlMatrix);
      DiagonalMatrixTools.preMult(R, tempControlMatrix, matrixToPack);
   }

   /** L_xx(X_k, U_k) */
   @Override
   public void getCostStateHessian(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(Q);
   }

   /** L_uu(X_k, U_k) */
   @Override
   public void getCostControlHessian(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(R);
   }

   /** L_ux(X_k, U_k) */
   @Override
   public void getCostStateGradientOfControlGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector,
                                                     DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(controlVectorSize, stateVectorSize);
   }

   /** L_xu(X_k, U_k) */
   @Override
   public void getCostControlGradientOfStateGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector,
                                                     DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(stateVectorSize, controlVectorSize);
   }
}
