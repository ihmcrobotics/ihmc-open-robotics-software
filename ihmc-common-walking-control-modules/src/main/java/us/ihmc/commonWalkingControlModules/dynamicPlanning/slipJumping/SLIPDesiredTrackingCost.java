package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SLIPDesiredTrackingCost implements LQTrackingCostFunction<SLIPState>
{
   static double qX = 1e-1;
   static double qY = 1e-1;
   static double qZ = 1e-1;
   static double qThetaX = 1.0;
   static double qThetaY = 1.0;
   static double qThetaZ = 1.0;
   static double qXDot = 0.0;
   static double qYDot = 0.0;
   static double qZDot = 0.0;
   static double qThetaDotX = 0.0;
   static double qThetaDotY = 0.0;
   static double qThetaDotZ = 0.0;

   static double rFx = 0.0;
   static double rFy = 0.0;
   static double rFz = 1.0e2;
   static double rTauX = 0.0;
   static double rTauY = 0.0;
   static double rTauZ = 0.0;
   static double rXf = 1.0e2;
   static double rYf = 1.0e2;
   static double rK = 1.0e5;

   private final DenseMatrix64F Q = new DenseMatrix64F(stateVectorSize, stateVectorSize);
   private final DenseMatrix64F R = new DenseMatrix64F(controlVectorSize, controlVectorSize);

   public SLIPDesiredTrackingCost()
   {
      Q.set(x, x, qX);
      Q.set(y, y, qY);
      Q.set(z, z, qZ);
      Q.set(thetaX, thetaX, qThetaX);
      Q.set(thetaY, thetaY, qThetaY);
      Q.set(thetaZ, thetaZ, qThetaZ);
      Q.set(xDot, xDot, qXDot);
      Q.set(yDot, yDot, qYDot);
      Q.set(zDot, zDot, qZDot);
      Q.set(thetaXDot, thetaXDot, qThetaDotX);
      Q.set(thetaYDot, thetaYDot, qThetaDotY);
      Q.set(thetaZDot, thetaZDot, qThetaDotZ);

      R.set(fx, fx, rFx);
      R.set(fy, fy, rFy);
      R.set(fz, fz, rFz);
      R.set(tauX, tauX, rTauX);
      R.set(tauY, tauY, rTauY);
      R.set(tauZ, tauZ, rTauZ);
      R.set(xF, xF, rXf);
      R.set(yF, yF, rYf);
      R.set(k, k, rK);
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
      CommonOps.scale(2.0, matrixToPack);
   }

   /** L_u(X_k, U_k) */
   @Override
   public void getCostControlGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVecotr,
                                      DenseMatrix64F desiredStateVector, DenseMatrix64F matrixToPack)
   {
      CommonOps.subtract(controlVector, desiredControlVecotr, tempControlMatrix);
      DiagonalMatrixTools.preMult(R, tempControlMatrix, matrixToPack);
      CommonOps.scale(2.0, matrixToPack);
   }

   /** L_xx(X_k, U_k) */
   @Override
   public void getCostStateHessian(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      CommonOps.scale(2.0, Q, matrixToPack);
   }

   /** L_uu(X_k, U_k) */
   @Override
   public void getCostControlHessian(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      CommonOps.scale(2.0, R, matrixToPack);
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
