package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.trajectoryOptimization.LQTrackingCostFunction;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SLIPDesiredTrackingCostFunction implements LQTrackingCostFunction<SLIPState>
{
   private final DenseMatrix64F R = new DenseMatrix64F(controlVectorSize, controlVectorSize);

   public SLIPDesiredTrackingCostFunction()
   {
      R.set(fx, fx, 0.0);
      R.set(fy, fy, 0.0);
      R.set(fz, fz, 0.0);
      R.set(tauX, tauX, 0.0);
      R.set(tauY, tauY, 0.0);
      R.set(tauZ, tauZ, 0.0);
      R.set(xF, xF, 1e1);
      R.set(yF, yF, 1e1);
      R.set(k, k, 1e3);
   }

   private DenseMatrix64F tempControlMatrix = new DenseMatrix64F(controlVectorSize, 1);
   private DenseMatrix64F tempWU = new DenseMatrix64F(controlVectorSize, 1);

   @Override
   public double getCost(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                         DenseMatrix64F desiredStateVector)
   {
      switch (state)
      {
      case STANCE:
         CommonOps.subtract(controlVector, desiredControlVector, tempControlMatrix);
         DiagonalMatrixTools.preMult(R, tempControlMatrix, tempWU);
         return CommonOps.dot(tempControlMatrix, tempWU);
      }

      return 0.0;
   }

   /** L_x(X_k, U_k) */
   @Override
   public void getCostStateGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                                    DenseMatrix64F desiredStateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.zero();
   }

   /** L_u(X_k, U_k) */
   @Override
   public void getCostControlGradient(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVecotr,
                                      DenseMatrix64F desiredStateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.zero();

      switch (state)
      {
      case STANCE:
         CommonOps.subtract(controlVector, desiredControlVecotr, tempControlMatrix);
         DiagonalMatrixTools.preMult(R, tempControlMatrix, matrixToPack);
         break;
      }
   }

   /** L_xx(X_k, U_k) */
   @Override
   public void getCostStateHessian(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      matrixToPack.zero();
   }

   /** L_uu(X_k, U_k) */
   @Override
   public void getCostControlHessian(SLIPState state, DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {
      switch (state)
      {
      case STANCE:
         matrixToPack.set(R);
         break;
      case FLIGHT:
         matrixToPack.zero();
         break;
      }
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
