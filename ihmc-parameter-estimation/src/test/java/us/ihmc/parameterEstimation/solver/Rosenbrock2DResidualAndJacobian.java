package us.ihmc.parameterEstimation.solver;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;
import us.ihmc.parameterEstimation.solver.GarbageFreeResidualAndJacobian;

public class Rosenbrock2DResidualAndJacobian implements GarbageFreeResidualAndJacobian
{
   int parameterSize = 2;
   int residualSize = 2;

   Rosenbrock2DResidualAndJacobian()
   {

   }

   @Override
   public void calculateResidual(DMatrixRMaj x, DMatrixRMaj residualToPack)
   {
      double x1 = x.get(0);
      double x2 = x.get(1);
      residualToPack.unsafe_set(0, 0, 10.0 * (x2 - MathTools.square(x1)));
      residualToPack.unsafe_set(1, 0, 1.0 - x1);
   }

   @Override
   public void calculateJacobian(DMatrixRMaj x, DMatrixRMaj jacobianToPack)
   {
      double x1 = x.get(0);
      double x2 = x.get(1);
      jacobianToPack.unsafe_set(0, 0, -20.0 * x1);
      jacobianToPack.unsafe_set(0, 1, 10.0);
      jacobianToPack.unsafe_set(1, 0, -1.0);
      jacobianToPack.unsafe_set(1, 1, 0.0);
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
