package us.ihmc.robotics.geometry;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.shapes.Plane3d;

/**
 * Created by agrabertilton on 11/13/14.
 */
public class LeastSquaresZPlaneFitter implements PlaneFitter
{
   private final DenseMatrix64F covarianceMatrix = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F zVarianceVector = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F coefficients = new DenseMatrix64F(3, 1);

   public LeastSquaresZPlaneFitter()
   {
   }

   public static boolean checkDistanceThreshold(List<Point3D> point3dList, Plane3d plane3d, double threshold)
   {
      if (plane3d.containsNaN())
         return false;

      for (Point3D point3d : point3dList)
      {
         double distance = plane3d.distance(point3d);
         if (distance > threshold)
            return false;
      }

      return true;
   }


   public static boolean checkZDistanceThreshold(List<Point3D> point3dList, double[] xyCCoefficients, double threshold)
   {
      // Checks to make sure no point is beyond the distance threshold to the plane in the z direction
      if (xyCCoefficients.length != 3)
      {
         return false;
      }

      for (Point3D point3d : point3dList)
      {
         if (Math.abs(point3d.getZ() + xyCCoefficients[0] * point3d.getX() + xyCCoefficients[1] * point3d.getY() + xyCCoefficients[2]) > threshold)
         {
            return false;
         }
      }

      return true;
   }

   public double fitPlaneToPoints(Point2D center, List<Point3D> pointList, Plane3d planeToPack)
   {
      double squareError = fitPlaneToPoints(pointList, planeToPack);

      double centerZ = planeToPack.getZOnPlane(center.getX(), center.getY());
      Point3D planePoint = new Point3D(center.getX(), center.getY(), centerZ);
      planeToPack.setPoint(planePoint);
      return squareError;
   }
   
   @Override
   public double fitPlaneToPoints(List<Point3D> pointList, Plane3d planeToPack)
   {
      // Given plane equation Ax+By+z +C = 0
      // find coefficients of plane that best fits the points using least squared in the z direction
      // coefficients returned (A,B,C)
      //pack the plane, and return the squared error
      double n = 0;
      double x = 0;
      double y = 0;
      double z = 0;
      double xx = 0;
      double xy = 0;
      double yy = 0;
      double xz = 0;
      double yz = 0;
      double zz = 0;

      if (pointList.size() < 3)
      {
         planeToPack.setToNaN();
         return Double.POSITIVE_INFINITY;
      }

      for (int i = 0; i < pointList.size(); i++)
      {
         Point3D point3d = pointList.get(i);
         n++;
         x += point3d.getX();
         y += point3d.getY();
         z += point3d.getZ();
         xx += Math.pow(point3d.getX(), 2);
         xy += point3d.getX() * point3d.getY();
         xz += point3d.getX() * point3d.getZ();
         yy += Math.pow(point3d.getY(), 2);
         yz += point3d.getY() * point3d.getZ();
         zz += point3d.getZ() * point3d.getZ();
      }

      covarianceMatrix.set(0, 0, xx);
      covarianceMatrix.set(0, 1, xy);
      covarianceMatrix.set(0, 2, x);
      
      covarianceMatrix.set(1, 0, xy);
      covarianceMatrix.set(1, 1, yy);
      covarianceMatrix.set(1, 2, y);
      
      covarianceMatrix.set(2, 0, x);
      covarianceMatrix.set(2, 1, y);
      covarianceMatrix.set(2, 2, n);
      
      zVarianceVector.set(0, 0, -1.0 * xz);
      zVarianceVector.set(1, 0, -1.0 * yz);
      zVarianceVector.set(2, 0, -1.0 * z);
      
      CommonOps.invert(covarianceMatrix);
      CommonOps.mult(covarianceMatrix, zVarianceVector, coefficients);

      double xSolution = x / n;
      double ySolution = y / n;
      double zSolution = -coefficients.get(0) * xSolution - coefficients.get(1) * ySolution - coefficients.get(2);
      
      if (Double.isNaN(zSolution))
      {
         planeToPack.setToNaN();
         return Double.POSITIVE_INFINITY;
      }
      
      planeToPack.setPoint(xSolution, ySolution, zSolution);
      planeToPack.setNormal(coefficients.get(0), coefficients.get(1), 1.0);

      double squaredError = 0;
      double A = coefficients.get(0);
      double B = coefficients.get(1);
      double C = coefficients.get(2);

      squaredError = A*A * xx + 2 * A*B*xy + 2*A*xz + 2*A*C*x + B*B*yy + 2*B*yz + 2*B*C*y + zz + 2* C*z + C*C;
      return squaredError/n;
   }



}
