package us.ihmc.robotics.controllers.pidGains;

import us.ihmc.euclid.matrix.Matrix3D;

public interface PID3DGainsReadOnly
{
   public abstract double[] getProportionalGains();

   public abstract double[] getDerivativeGains();

   public abstract double[] getIntegralGains();

   public abstract double getMaximumIntegralError();

   public abstract double getMaximumDerivativeError();

   public abstract double getMaximumProportionalError();

   public abstract double getMaximumFeedback();

   public abstract double getMaximumFeedbackRate();

   public default GainCoupling getGainCoupling()
   {
      return GainCoupling.NONE;
   }

   public default void getProportionalGainMatrix(Matrix3D proportialGainMatrixToPack)
   {
      setMatrixDiagonal(getProportionalGains(), proportialGainMatrixToPack);
   }

   public default void getDerivativeGainMatrix(Matrix3D derivativeGainMatrixToPack)
   {
      setMatrixDiagonal(getDerivativeGains(), derivativeGainMatrixToPack);
   }

   public default void getIntegralGainMatrix(Matrix3D integralGainMatrixToPack)
   {
      setMatrixDiagonal(getIntegralGains(), integralGainMatrixToPack);
   }

   static void setMatrixDiagonal(double[] diagonalElements, Matrix3D matrixToFill)
   {
      matrixToFill.fill(0.0);
      for (int i = 0; i < 3; i++)
      {
         matrixToFill.setElement(i, i, diagonalElements[i]);
      }
   }
}
