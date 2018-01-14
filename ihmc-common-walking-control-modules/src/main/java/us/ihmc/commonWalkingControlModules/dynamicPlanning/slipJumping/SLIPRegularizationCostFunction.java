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
      Q.set(x, x, 1e-5);
      Q.set(y, y, 1e-5);
      Q.set(z, z, 1e-5);
      Q.set(thetaX, thetaX, 1e-5);
      Q.set(thetaY, thetaY, 1e-5);
      Q.set(thetaZ, thetaZ, 1e-5);
      Q.set(xDot, xDot, 1e-5);
      Q.set(yDot, yDot, 1e-5);
      Q.set(zDot, zDot, 1e-5);
      Q.set(thetaXDot, thetaXDot, 1e-5);
      Q.set(thetaYDot, thetaYDot, 1e-5);
      Q.set(thetaZDot, thetaZDot, 1e-5);

      R.set(fx, fx, 1e-5);
      R.set(fy, fy, 1e-5);
      R.set(fz, fz, 1e-8);
      R.set(tauX, tauX, 1e1);
      R.set(tauY, tauY, 1e1);
      R.set(tauZ, tauZ, 1e1);
      R.set(xF, xF, 1e-5);
      R.set(yF, yF, 1e-5);
      R.set(k, k, 1e-8);
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
