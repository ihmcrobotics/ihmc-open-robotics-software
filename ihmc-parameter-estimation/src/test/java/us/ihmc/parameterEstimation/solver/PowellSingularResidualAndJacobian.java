package us.ihmc.parameterEstimation.solver;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;
import us.ihmc.parameterEstimation.solver.GarbageFreeResidualAndJacobian;

/**
 * See More, Garbow, and Hillstrom (1981): "Testing Unconstrained Optimization Software".
 * <p>
 * This is the Powell singular function problem for nonlinear least squares solvers.
 * </p>
 */
public class PowellSingularResidualAndJacobian implements GarbageFreeResidualAndJacobian
{
   int parameterSize = 4;
   int residualSize = 4;

   @Override
   public void calculateResidual(DMatrixRMaj x, DMatrixRMaj residualToPack)
   {
      residualToPack.unsafe_set(0, 0, x.get(0) + 10.0 * x.get(1));
      residualToPack.unsafe_set(1, 0, Math.sqrt(5) * (x.get(2) - x.get(3)));
      residualToPack.unsafe_set(2, 0, MathTools.square(x.get(1) - 2.0 * x.get(2)));
      residualToPack.unsafe_set(3, 0, Math.sqrt(10) * MathTools.square(x.get(0) - x.get(3)));
   }

   @Override
   public void calculateJacobian(DMatrixRMaj x, DMatrixRMaj jacobianToPack)
   {
      jacobianToPack.zero();
      // Only some Jacobian entries are non-zero
      // First row
      jacobianToPack.unsafe_set(0, 0, 1.0);
      jacobianToPack.unsafe_set(0, 1, 10.0);
      // Second row
      jacobianToPack.unsafe_set(1, 2, Math.sqrt(5));
      jacobianToPack.unsafe_set(1, 3, -Math.sqrt(5));
      // Third row
      jacobianToPack.unsafe_set(2, 1, 2.0 * (x.get(1) - 2.0 * x.get(2)));
      jacobianToPack.unsafe_set(2, 2, 8.0 * x.get(2) - 4.0 * x.get(1));
      // Fourth row
      jacobianToPack.unsafe_set(3, 0, Math.sqrt(10.0) * 2.0 * (x.get(0) - x.get(3)));
      jacobianToPack.unsafe_set(3, 3, Math.sqrt(10.0) * -2.0 * (x.get(0) - x.get(3)));
   }

   @Override
   public int getParameterSize()
   {
      return parameterSize;
   }

   @Override
   public int getResidualSize()
   {
      return residualSize;
   }
}
