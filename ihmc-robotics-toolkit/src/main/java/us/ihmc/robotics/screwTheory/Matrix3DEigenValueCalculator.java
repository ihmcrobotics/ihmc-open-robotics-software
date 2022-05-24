package us.ihmc.robotics.screwTheory;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tools.Matrix3DFeatures;
import us.ihmc.matrixlib.MatrixTools;

public class Matrix3DEigenValueCalculator
{
   private final Matrix3D B = new Matrix3D();

   private double eig1 = Double.NaN;
   private double eig2 = Double.NaN;
   private double eig3 = Double.NaN;

   public void compute(Matrix3DReadOnly matrix)
   {
      if (!matrix.isMatrixSymmetric())
         throw new RuntimeException("Only works for symmetric matrices.");

      double p1 = MathTools.square(matrix.getM01()) + MathTools.square(matrix.getM02()) + MathTools.square(matrix.getM12());
      double q = trace(matrix) / 3.0;
      double p2 = MathTools.square(matrix.getM00() - q) + MathTools.square(matrix.getM11() - q) + MathTools.square(matrix.getM22() - q) + 2.0 * p1;
      double p = Math.sqrt(p2 / 6.0);
      B.set(matrix);
      B.subM00(q);
      B.subM11(q);
      B.subM22(q);
      B.scale(1.0 / p);

      double r = B.determinant() / 2.0;

      double phi;
      if (r <= -1.0)
         phi = Math.PI / 3.0;
      else if (r >= 1)
         phi = 0.0;
      else
         phi = Math.acos(r) / 3.0;

      eig1 = q + 2.0 * p * Math.cos(phi);
      eig3 = q + 2.0 * p * Math.cos(phi + (2.0 * Math.PI / 3.0));
      eig2 = 3.0 * q - eig1 - eig3;
   }

   public double getFirstEigenValue()
   {
      return eig1;
   }

   public double getSecondEigenValue()
   {
      return eig2;
   }

   public double getThirdEigenValue()
   {
      return eig3;
   }

   private static double trace(Matrix3DReadOnly matrix)
   {
      return matrix.getM00() + matrix.getM11() + matrix.getM22();
   }
}
