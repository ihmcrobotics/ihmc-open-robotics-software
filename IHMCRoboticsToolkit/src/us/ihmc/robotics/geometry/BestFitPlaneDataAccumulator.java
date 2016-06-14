package us.ihmc.robotics.geometry;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

public class BestFitPlaneDataAccumulator
{
   private double sumXsquared = 0;
   private double sumYsquared = 0;
   private double sumXY = 0;
   private double sumX = 0;
   private double sumY = 0;
   private int sumUnity = 0;
   private double sumZ = 0;
   private double sumZX = 0;
   private double sumZY = 0;

   public void addPoint(double x, double y, double z)
   {
      sumXsquared += x * x;
      sumYsquared += y * y;
      sumXY += x * y;
      sumX += x;
      sumY += y;
      sumUnity++;
      sumZ += z;
      sumZX += z * x;
      sumZY += z * y;
   }

   public void populateSquareMatrixToInvert(Matrix3d matrix)
   {
      matrix.m00 = sumXsquared;
      matrix.m01 = sumXY;
      matrix.m10 = sumXY;
      matrix.m02 = sumX;
      matrix.m20 = sumX;
      matrix.m11 = sumYsquared;
      matrix.m12 = sumY;
      matrix.m21 = sumY;
      matrix.m22 = sumUnity;
   }

   public int getCount()
   {
      return sumUnity;
   }

   public void populateMomentVector(Vector3d vectorToPopulate)
   {
      vectorToPopulate.x = sumZX;
      vectorToPopulate.y = sumZY;
      vectorToPopulate.z = sumZ;
   }

   public void populateDegenerate1XCase(DenseMatrix64F matrixXCase, DenseMatrix64F vectorXCase)
   {
      matrixXCase.set(0, 0, sumXsquared);
      matrixXCase.set(0, 1, sumX);
      matrixXCase.set(1, 0, sumX);
      matrixXCase.set(1, 1, sumUnity);
      vectorXCase.set(0, 0, sumZX);
      vectorXCase.set(1, 0, sumZ);
   }
   public void populateDegenerate1YCase(DenseMatrix64F matrixYCase, DenseMatrix64F vectorYCase)
   {
      matrixYCase.set(0, 0, sumYsquared);
      matrixYCase.set(0, 1, sumY);
      matrixYCase.set(1, 0, sumY);
      matrixYCase.set(1, 1, sumUnity);
      vectorYCase.set(0, 0, sumZY);
      vectorYCase.set(1, 0, sumZ);
   }
   public double solveDegenerate0Case()
   {
      return (sumZ/sumUnity);
   }
}
