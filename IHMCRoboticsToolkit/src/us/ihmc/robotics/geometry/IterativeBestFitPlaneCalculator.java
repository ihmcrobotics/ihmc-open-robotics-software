package us.ihmc.robotics.geometry;

import us.ihmc.robotics.MathTools;

import javax.vecmath.GMatrix;
import javax.vecmath.Matrix3d;
import javax.vecmath.SingularMatrixException;
import javax.vecmath.Vector3d;

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




   private GMatrix matrixXCase = new GMatrix(2, 2);
   private GMatrix matrixYCase = new GMatrix(2, 2);
   private GMatrix vectorXCase = new GMatrix(2, 1);
   private GMatrix vectorYCase = new GMatrix(2, 1);
   private GMatrix retXCase = new GMatrix(2, 1);
   private GMatrix retYCase = new GMatrix(2, 1);

   private double[] solveAsDegenerate1Case(BestFitPlaneDataAccumulator data)
   {
      boolean xSuccess = true;
      boolean ySuccess = true;
      data.populateDegenerate1XCase(matrixXCase, vectorXCase);

      try
      {
         matrixXCase.invert();
         retXCase.mul(matrixXCase, vectorXCase);
      }
      catch (SingularMatrixException e)
      {
         xSuccess = false;
      }

      data.populateDegenerate1YCase(matrixYCase, vectorYCase);

      try
      {
         matrixYCase.invert();
         retYCase.mul(matrixYCase, vectorYCase);
      }
      catch (Exception e)
      {
         ySuccess = false;
      }
//      System.out.println("IterativeBestFitPlaneCalculator xSuccess = "+xSuccess+" and ySuccess = "+ySuccess+".");

      double[] ret = null;
      if (xSuccess && ySuccess)
      {
         double kx = retXCase.getElement(0, 0);
         double ky = retYCase.getElement(0, 0);
//         System.out.println("IterativeBestFitPlaneCalculator kx= "+kx+" and ky= "+ky);
         double kySquared = ky*ky;
         double kxSquared = kx*kx;
         double sharedFactor = kx * ky / (kxSquared + kySquared);
         if (!MathTools.isFinite(sharedFactor))
            sharedFactor=0.0;
         ret = new double[] {sharedFactor * ky, sharedFactor * kx, retXCase.getElement(1, 0)};
      }
      else if (xSuccess &&!ySuccess)
      {
         ret = new double[] {retXCase.getElement(0, 0), 0.0, retXCase.getElement(1, 0)};
      }
      else if (!xSuccess && ySuccess)
      {
         ret = new double[] {0.0, retYCase.getElement(0, 0), retYCase.getElement(1, 0)};
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
