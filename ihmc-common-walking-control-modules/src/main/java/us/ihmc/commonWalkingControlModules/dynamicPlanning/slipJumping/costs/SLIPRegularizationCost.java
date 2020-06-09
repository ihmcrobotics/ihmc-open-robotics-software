package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.costs;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState;
import us.ihmc.matrixlib.DiagonalMatrixTools;
import us.ihmc.trajectoryOptimization.LQCostFunction;

public class SLIPRegularizationCost implements LQCostFunction<SLIPState>
{
   static double qX = 0.0;
   static double qY = 0.0;
   static double qZ = 0.0;
   static double qThetaX = 0.0;
   static double qThetaY = 0.0;
   static double qThetaZ = 0.0;
   static double qXDot = 1e-6;
   static double qYDot = 1e-6;
   static double qZDot = 1e-6;
   static double qThetaDotX = 1e-5;
   static double qThetaDotY = 1e-5;
   static double qThetaDotZ = 1e-5;

   static double rFx = 1e-7;
   static double rFy = 1e-7;
   static double rFz = 1e-8;
   static double rTauX = 1e0;
   static double rTauY = 1e0;
   static double rTauZ = 1e0;
   static double rXf = 0.0;
   static double rYf = 0.0;
   static double rK = 1e-12;

   private final DMatrixRMaj Q = new DMatrixRMaj(stateVectorSize, stateVectorSize);
   private final DMatrixRMaj R = new DMatrixRMaj(controlVectorSize, controlVectorSize);

   public SLIPRegularizationCost()
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

   private DMatrixRMaj tempWX = new DMatrixRMaj(stateVectorSize, 1);
   private DMatrixRMaj tempWU = new DMatrixRMaj(controlVectorSize, 1);

   @Override
   public double getCost(SLIPState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants)
   {
      DiagonalMatrixTools.preMult(Q, stateVector, tempWX);
      DiagonalMatrixTools.preMult(R, controlVector, tempWU);

      return CommonOps_DDRM.dot(controlVector, tempWU) + CommonOps_DDRM.dot(stateVector, tempWX);
   }

   /** L_x(X_k, U_k) */
   @Override
   public void getCostStateGradient(SLIPState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      DiagonalMatrixTools.preMult(Q, stateVector, matrixToPack);
      CommonOps_DDRM.scale(2.0, matrixToPack);
   }

   /** L_u(X_k, U_k) */
   @Override
   public void getCostControlGradient(SLIPState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants, DMatrixRMaj matrixToPack)
   {
      DiagonalMatrixTools.preMult(R, controlVector, matrixToPack);
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
      matrixToPack.zero();
   }

   /** L_xu(X_k, U_k) */
   @Override
   public void getCostControlGradientOfStateGradient(SLIPState state, DMatrixRMaj controlVector, DMatrixRMaj stateVector, DMatrixRMaj constants,
                                                     DMatrixRMaj matrixToPack)
   {
      matrixToPack.reshape(stateVectorSize, controlVectorSize);
      matrixToPack.zero();
   }
}
