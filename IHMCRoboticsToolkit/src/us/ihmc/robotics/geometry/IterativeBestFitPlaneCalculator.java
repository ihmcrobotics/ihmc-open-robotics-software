package us.ihmc.robotics.geometry;

import javax.vecmath.Matrix3d;
import javax.vecmath.SingularMatrixException;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.MathTools;

public class IterativeBestFitPlaneCalculator
{
   private Matrix3d matrix = new Matrix3d();
   private Matrix3d matrixInverse = new Matrix3d();
   private Vector3d vector = new Vector3d();
   private Vector3d output = new Vector3d();
   private double[] ret = new double[3];

   public double[] updatePlane(BestFitPlaneDataAccumulator data)
   {
      data.populateSquareMatrixToInvert(matrix);
      data.populateMomentVector(vector);
//      System.out.println("matrix.determinant()= "+matrix.determinant());
      if (Math.abs(matrix.determinant())<1e-6)
         return solveAsDegenerate1Case(data);
      try
      {
         
         matrixInverse.invert(matrix);
         matrixInverse.transform(vector, output);
      }
      catch (SingularMatrixException e)
      {
         return solveAsDegenerate1Case(data);
      }
      output.get(ret);

      return ret;
   }
//   java.lang.AssertionError: component 1 expected:<0.18061607089663576> but was:<0.18061507000311394>
//   java.lang.AssertionError: component 1 expected:<0.18061607089663576> but was:<0.1806125239030579>




   private DenseMatrix64F matrixXCase = new DenseMatrix64F(2, 2);
   private DenseMatrix64F matrixYCase = new DenseMatrix64F(2, 2);
   private DenseMatrix64F vectorXCase = new DenseMatrix64F(2, 1);
   private DenseMatrix64F vectorYCase = new DenseMatrix64F(2, 1);
   private DenseMatrix64F retXCase = new DenseMatrix64F(2, 1);
   private DenseMatrix64F retYCase = new DenseMatrix64F(2, 1);

   private double[] solveAsDegenerate1Case(BestFitPlaneDataAccumulator data)
   {
      boolean xSuccess = true;
      boolean ySuccess = true;
      data.populateDegenerate1XCase(matrixXCase, vectorXCase);

      try
      {
         CommonOps.invert(matrixXCase);
         CommonOps.mult(matrixXCase, vectorXCase, retXCase);
      }
      catch (SingularMatrixException e)
      {
         xSuccess = false;
      }

      data.populateDegenerate1YCase(matrixYCase, vectorYCase);

      try
      {
         CommonOps.invert(matrixYCase);
         CommonOps.mult(matrixYCase, vectorYCase, retYCase);
      }
      catch (Exception e)
      {
         ySuccess = false;
      }
//      System.out.println("IterativeBestFitPlaneCalculator xSuccess = "+xSuccess+" and ySuccess = "+ySuccess+".");

      double[] ret = null;
      if (xSuccess && ySuccess)
      {
         double kx = retXCase.get(0, 0);
         double ky = retYCase.get(0, 0);
//         System.out.println("IterativeBestFitPlaneCalculator kx= "+kx+" and ky= "+ky);
         double kySquared = ky*ky;
         double kxSquared = kx*kx;
         double sharedFactor = kx * ky / (kxSquared + kySquared);
         if (!MathTools.isFinite(sharedFactor))
            sharedFactor=0.0;
         ret = new double[] {sharedFactor * ky, sharedFactor * kx, retXCase.get(1, 0)};
      }
      else if (xSuccess &&!ySuccess)
      {
         ret = new double[] {retXCase.get(0, 0), 0.0, retXCase.get(1, 0)};
      }
      else if (!xSuccess && ySuccess)
      {
         ret = new double[] {0.0, retYCase.get(0, 0), retYCase.get(1, 0)};
      }
      else
      {
         double average = data.solveDegenerate0Case();
         if (MathTools.isFinite(average))
            ret = new double[] {0.0, 0.0, average};
      }

      return ret;
   }
}
