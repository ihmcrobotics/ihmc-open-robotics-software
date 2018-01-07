package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.trajectoryOptimization.LQCostFunction;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SLIPRegularizationCostFunction implements LQCostFunction<SLIPState>
{
   private final DenseMatrix64F Q = new DenseMatrix64F(stateVectorSize, stateVectorSize);
   private final DenseMatrix64F R = new DenseMatrix64F(controlVectorSize, controlVectorSize);

   public SLIPRegularizationCostFunction()
   {
      Q.set(x, x, 5e0);
      Q.set(y, y, 5e0);
      Q.set(z, z, 5e0);
      Q.set(thetaX, thetaX, 5e0);
      Q.set(thetaY, thetaY, 5e0);
      Q.set(thetaZ, thetaZ, 5e0);
      Q.set(xDot, xDot, 5e0);
      Q.set(yDot, yDot, 5e0);
      Q.set(zDot, zDot, 5e0);
      Q.set(thetaXDot, thetaXDot, 5e0);
      Q.set(thetaYDot, thetaYDot, 5e0);
      Q.set(thetaZDot, thetaZDot, 5e0);

      R.set(fx, fx, 0.0);
      R.set(fy, fy, 0.0);
      R.set(fz, fz, 0.0);
      R.set(tauX, tauX, 0.0);
      R.set(tauY, tauY, 0.0);
      R.set(tauZ, tauZ, 0.0);
      R.set(xF, xF, 0.0);
      R.set(yF, yF, 0.0);
      R.set(k, k, 0.0);
   }

   private DenseMatrix64F tempWX = new DenseMatrix64F(stateVectorSize, 1);
   private DenseMatrix64F tempWU = new DenseMatrix64F(controlVectorSize, 1);

   @Override
   public double getCost(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector)
   {
      DiagonalMatrixTools.preMult(Q, stateVector, tempWX);
      DiagonalMatrixTools.preMult(R, controlVector, tempWU);

      return CommonOps.dot(controlVector, tempWU) + CommonOps.dot(stateVector, tempWX);
   }

   /** L_x(X_k, U_k) */
   @Override
   public void getCostStateGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      DiagonalMatrixTools.preMult(Q, stateVector, matrixToPack);
   }

   /** L_u(X_k, U_k) */
   @Override
   public void getCostControlGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      DiagonalMatrixTools.preMult(R, controlVector, matrixToPack);
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
      matrixToPack.zero();
   }

   /** L_xu(X_k, U_k) */
   @Override
   public void getCostControlGradientOfStateGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector,
                                                     DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(stateVectorSize, controlVectorSize);
      matrixToPack.zero();
   }
}
