package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.costs;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SLIPDesiredTrackingCost implements LQTrackingCostFunction<SLIPState>
{
   static double qXFlight = 1e-3;
   static double qYFlight = 1e-3;
   static double qZFlight = 1e-1;
   static double qThetaXFlight = 1.0e-1;
   static double qThetaYFlight = 1.0e-1;
   static double qThetaZFlight = 1.0e-1;
   static double qXDotFlight = 0.0;
   static double qYDotFlight = 0.0;
   static double qZDotFlight = 0.0;
   static double qThetaDotXFlight = 1e2;
   static double qThetaDotYFlight = 1e2;
   static double qThetaDotZFlight = 1e2;

   static double rFxFlight = 0.0;
   static double rFyFlight = 0.0;
   static double rFzFlight = 0.0;
   static double rTauXFlight = 0.0;
   static double rTauYFlight = 0.0;
   static double rTauZFlight = 0.0;
   static double rXfFlight = 0.0;
   static double rYfFlight = 0.0;
   static double rKFlight = 0.0;

   static double qXStance = 1e-1;
   static double qYStance = 1e-1;
   static double qZStance = 1e-1;
   static double qThetaXStance = 1.0;
   static double qThetaYStance = 1.0;
   static double qThetaZStance = 1.0;
   static double qXDotStance = 0.0;
   static double qYDotStance = 0.0;
   static double qZDotStance = 0.0;
   static double qThetaDotXStance = 0.0;
   static double qThetaDotYStance = 0.0;
   static double qThetaDotZStance = 0.0;

   static double rFxStance = 0.0;
   static double rFyStance = 0.0;
   static double rFzStance = 1.0e2;
   static double rTauXStance = 0.0;
   static double rTauYStance = 0.0;
   static double rTauZStance = 0.0;
   static double rXfStance = 5.0e6;
   static double rYfStance = 5.0e6;
   static double rKStance = 1.0e5;

   private final DenseMatrix64F QFlight = new DenseMatrix64F(stateVectorSize, stateVectorSize);
   private final DenseMatrix64F RFlight = new DenseMatrix64F(controlVectorSize, controlVectorSize);
   private final DenseMatrix64F QStance = new DenseMatrix64F(stateVectorSize, stateVectorSize);
   private final DenseMatrix64F RStance = new DenseMatrix64F(controlVectorSize, controlVectorSize);

   public SLIPDesiredTrackingCost()
   {
      QStance.set(x, x, qXStance);
      QStance.set(y, y, qYStance);
      QStance.set(z, z, qZStance);
      QStance.set(thetaX, thetaX, qThetaXStance);
      QStance.set(thetaY, thetaY, qThetaYStance);
      QStance.set(thetaZ, thetaZ, qThetaZStance);
      QStance.set(xDot, xDot, qXDotStance);
      QStance.set(yDot, yDot, qYDotStance);
      QStance.set(zDot, zDot, qZDotStance);
      QStance.set(thetaXDot, thetaXDot, qThetaDotXStance);
      QStance.set(thetaYDot, thetaYDot, qThetaDotYStance);
      QStance.set(thetaZDot, thetaZDot, qThetaDotZStance);

      RStance.set(fx, fx, rFxStance);
      RStance.set(fy, fy, rFyStance);
      RStance.set(fz, fz, rFzStance);
      RStance.set(tauX, tauX, rTauXStance);
      RStance.set(tauY, tauY, rTauYStance);
      RStance.set(tauZ, tauZ, rTauZStance);
      RStance.set(xF, xF, rXfStance);
      RStance.set(yF, yF, rYfStance);
      RStance.set(k, k, rKStance);

      QFlight.set(x, x, qXFlight);
      QFlight.set(y, y, qYFlight);
      QFlight.set(z, z, qZFlight);
      QFlight.set(thetaX, thetaX, qThetaXFlight);
      QFlight.set(thetaY, thetaY, qThetaYFlight);
      QFlight.set(thetaZ, thetaZ, qThetaZFlight);
      QFlight.set(xDot, xDot, qXDotFlight);
      QFlight.set(yDot, yDot, qYDotFlight);
      QFlight.set(zDot, zDot, qZDotFlight);
      QFlight.set(thetaXDot, thetaXDot, qThetaDotXFlight);
      QFlight.set(thetaYDot, thetaYDot, qThetaDotYFlight);
      QFlight.set(thetaZDot, thetaZDot, qThetaDotZFlight);

      RFlight.set(fx, fx, rFxFlight);
      RFlight.set(fy, fy, rFyFlight);
      RFlight.set(fz, fz, rFzFlight);
      RFlight.set(tauX, tauX, rTauXFlight);
      RFlight.set(tauY, tauY, rTauYFlight);
      RFlight.set(tauZ, tauZ, rTauZFlight);
      RFlight.set(xF, xF, rXfFlight);
      RFlight.set(yF, yF, rYfFlight);
      RFlight.set(k, k, rKFlight);
   }

   private DenseMatrix64F tempStateMatrix = new DenseMatrix64F(stateVectorSize, 1);
   private DenseMatrix64F tempControlMatrix = new DenseMatrix64F(controlVectorSize, 1);
   private DenseMatrix64F tempWX = new DenseMatrix64F(stateVectorSize, 1);
   private DenseMatrix64F tempWU = new DenseMatrix64F(controlVectorSize, 1);

   @Override
   public double getCost(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                         DenseMatrix64F desiredStateVector, DenseMatrix64F constants)
   {
      CommonOps.subtract(controlVector, desiredControlVector, tempControlMatrix);
      CommonOps.subtract(stateVector, desiredStateVector, tempStateMatrix);

      switch (state)
      {
      case FLIGHT:
         DiagonalMatrixTools.preMult(QFlight, tempStateMatrix, tempWX);
         DiagonalMatrixTools.preMult(RFlight, tempControlMatrix, tempWU);
         break;
      case STANCE:
         DiagonalMatrixTools.preMult(QStance, tempStateMatrix, tempWX);
         DiagonalMatrixTools.preMult(RStance, tempControlMatrix, tempWU);
         break;
      }

      return CommonOps.dot(tempControlMatrix, tempWU) + CommonOps.dot(tempStateMatrix, tempWX);
   }

   /** L_x(X_k, U_k) */
   @Override
   public void getCostStateGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                                    DenseMatrix64F desiredStateVector, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      CommonOps.subtract(stateVector, desiredStateVector, tempStateMatrix);
      switch (state)
      {
      case FLIGHT:
         DiagonalMatrixTools.preMult(QFlight, tempStateMatrix, matrixToPack);
         break;
      case STANCE:
         DiagonalMatrixTools.preMult(QStance, tempStateMatrix, matrixToPack);
         break;
      }
      CommonOps.scale(2.0, matrixToPack);
   }

   /** L_u(X_k, U_k) */
   @Override
   public void getCostControlGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVecotr,
                                      DenseMatrix64F desiredStateVector, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      CommonOps.subtract(controlVector, desiredControlVecotr, tempControlMatrix);
      switch (state)
      {
      case FLIGHT:
         DiagonalMatrixTools.preMult(RFlight, tempControlMatrix, matrixToPack);
         break;
      case STANCE:
         DiagonalMatrixTools.preMult(RStance, tempControlMatrix, matrixToPack);
         break;
      }
      CommonOps.scale(2.0, matrixToPack);
   }

   /** L_xx(X_k, U_k) */
   @Override
   public void getCostStateHessian(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      switch (state)
      {
      case FLIGHT:
         CommonOps.scale(2.0, QFlight, matrixToPack);
         break;
      case STANCE:
         CommonOps.scale(2.0, QStance, matrixToPack);
         break;
      }
   }

   /** L_uu(X_k, U_k) */
   @Override
   public void getCostControlHessian(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants, DenseMatrix64F matrixToPack)
   {
      switch (state)
      {
      case FLIGHT:
         CommonOps.scale(2.0, RFlight, matrixToPack);
         break;
      case STANCE:
         CommonOps.scale(2.0, RStance, matrixToPack);
         break;
      }
   }

   /** L_ux(X_k, U_k) */
   @Override
   public void getCostStateGradientOfControlGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants,
                                                     DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(controlVectorSize, stateVectorSize);
   }

   /** L_xu(X_k, U_k) */
   @Override
   public void getCostControlGradientOfStateGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F constants,
                                                     DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(stateVectorSize, controlVectorSize);
   }
}
