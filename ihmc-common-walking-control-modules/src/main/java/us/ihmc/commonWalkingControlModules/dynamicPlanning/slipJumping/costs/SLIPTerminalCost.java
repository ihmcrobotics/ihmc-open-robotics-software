package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.costs;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState;
import us.ihmc.matrixlib.DiagonalMatrixTools;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

public class SLIPTerminalCost implements LQTrackingCostFunction<SLIPState>
{
   static double qX = 1e10;
   static double qY = 1e10;
   static double qZ = 1e12;
   static double qThetaX = 1e10;
   static double qThetaY = 1e10;
   static double qThetaZ = 1e10;
   static double qXDot = 1e15;
   static double qYDot = 1e15;
   static double qZDot = 1e15;
   static double qThetaDotX = 1e10;
   static double qThetaDotY = 1e10;
   static double qThetaDotZ = 1e10;

   static double rFx = 0e0;
   static double rFy = 0e0;
   static double rFz = 0e0;
   static double rTauX = 0e0;
   static double rTauY = 0e0;
   static double rTauZ = 0e0;
   static double rXf = 1e5;
   static double rYf = 1e5;
   static double rK = 0e0;

   private final DMatrixRMaj Q = new DMatrixRMaj(stateVectorSize, stateVectorSize);
   private final DMatrixRMaj R = new DMatrixRMaj(controlVectorSize, controlVectorSize);

   public SLIPTerminalCost()
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

   private DMatrixRMaj tempStateMatrix = new DMatrixRMaj(stateVectorSize, 1);
   private DMatrixRMaj tempControlMatrix = new DMatrixRMaj(controlVectorSize, 1);
   private DMatrixRMaj tempWX = new DMatrixRMaj(stateVectorSize, 1);
   private DMatrixRMaj tempWU = new DMatrixRMaj(controlVectorSize, 1);

   @Override
   public double getCost(SLIPState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector,
                         DMatrixRMaj desiredStateVector, DMatrixRMaj constants)
   {
      CommonOps_DDRM.subtract(controlVector, desiredControlVector, tempControlMatrix);
      CommonOps_DDRM.subtract(stateVector, desiredStateVector, tempStateMatrix);

      DiagonalMatrixTools.preMult(Q, tempStateMatrix, tempWX);
      DiagonalMatrixTools.preMult(R, tempControlMatrix, tempWU);

      return CommonOps_DDRM.dot(tempControlMatrix, tempWU) + CommonOps_DDRM.dot(tempStateMatrix, tempWX);
   }

   /** L_x(X_k, U_k) */
   @Override
   public void getCostStateGradient(SLIPState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVector,
                                    DMatrixRMaj desiredStateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      CommonOps_DDRM.subtract(stateVector, desiredStateVector, tempStateMatrix);
      DiagonalMatrixTools.preMult(Q, tempStateMatrix, matrixToPack);
      CommonOps_DDRM.scale(2.0, matrixToPack);
   }

   /** L_u(X_k, U_k) */
   @Override
   public void getCostControlGradient(SLIPState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj desiredControlVecotr,
                                      DMatrixRMaj desiredStateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      CommonOps_DDRM.subtract(controlVector, desiredControlVecotr, tempControlMatrix);
      DiagonalMatrixTools.preMult(R, tempControlMatrix, matrixToPack);
      CommonOps_DDRM.scale(2.0, matrixToPack);
   }

   /** L_xx(X_k, U_k) */
   @Override
   public void getCostStateHessian(SLIPState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      CommonOps_DDRM.scale(2.0, Q, matrixToPack);
   }

   /** L_uu(X_k, U_k) */
   @Override
   public void getCostControlHessian(SLIPState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      CommonOps_DDRM.scale(2.0, R, matrixToPack);
   }

   /** L_ux(X_k, U_k) */
   @Override
   public void getCostStateGradientOfControlGradient(SLIPState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants,
                                                     DMatrixRMaj matrixToPack)
   {
      matrixToPack.reshape(controlVectorSize, stateVectorSize);
   }

   /** L_xu(X_k, U_k) */
   @Override
   public void getCostControlGradientOfStateGradient(SLIPState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants,
                                                     DMatrixRMaj matrixToPack)
   {
      matrixToPack.reshape(stateVectorSize, controlVectorSize);
   }
}
