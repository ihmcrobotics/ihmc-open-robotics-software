package us.ihmc.commonWalkingControlModules.trajectories;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.LineSegment2d;

public class ThreePointDoubleSplines2D
{
   private Point3d previousQueryPosition = null;
   private double previousQuerySlope = 0.0;
   private double previousQuerySecondDerivative = 0.0;
   
//   private Line2d lineBetweenFlatPoints;
   private LineSegment2d lineSegmentBetweenFlatPoints;
   
//   private Point3d[] colinearPoints = new Point3d[3];
   
   private ThreePointDoubleSplines1D threePointDoubleSplines1D;

   public void setPoints(Point3d existingFlatPoint, Point3d newFlatPoint)
   {
      Point2d existingFlatPoint2d = convertPointTo2d(existingFlatPoint);
      Point2d newFlatPoint2d = convertPointTo2d(newFlatPoint);
      
      if (previousQueryPosition == null)
      {
         previousQueryPosition = new Point3d(existingFlatPoint);
         previousQueryPosition.add(newFlatPoint);
         previousQueryPosition.scale(0.5);
         previousQuerySlope = 2.0 * (newFlatPoint.getZ() - existingFlatPoint.getZ())/(existingFlatPoint2d.distance(newFlatPoint2d));
         previousQuerySecondDerivative = 0.0;
      }
      
      Point2d newNonFlatPoint2d = convertPointTo2d(previousQueryPosition);
      
//      lineBetweenFlatPoints = new Line2d(existingFlatPoint2d, newFlatPoint2d);
      lineSegmentBetweenFlatPoints = new LineSegment2d(existingFlatPoint2d, newFlatPoint2d);
      
//      Point2d nonflatPointProjection2d = lineBetweenFlatPoints.orthogonalProjectionCopy(newNonFlatPoint2d);
//      Point3d nonflatPointProjection3d = new Point3d(nonflatPointProjection2d.getX(), nonflatPointProjection2d.getY(), previousQueryPosition.getZ());
      
      double distanceBetweenFlatPoints = lineSegmentBetweenFlatPoints.length();
      double nonFlatPercentageAlong = lineSegmentBetweenFlatPoints.percentageAlongLineSegment(newNonFlatPoint2d);
      
      Point2d[] pointsIn;
      double[] slopes;
      double[] secondDerivatives;
      
      threePointDoubleSplines1D = new ThreePointDoubleSplines1D();

      Point2d nonFlat1D = new Point2d(nonFlatPercentageAlong*distanceBetweenFlatPoints, previousQueryPosition.getZ());
      Point2d existingFlat1D = new Point2d(0.0, existingFlatPoint.getZ());
      Point2d newFlat1D = new Point2d(distanceBetweenFlatPoints, newFlatPoint.getZ());
      
      if (nonFlatPercentageAlong < 0.0)
      {
//         colinearPoints = new Point3d[]{nonflatPointProjection3d, existingFlatPoint, newFlatPoint};
         pointsIn = new Point2d[]{nonFlat1D, existingFlat1D, newFlat1D};
         slopes = new double[]{previousQuerySlope, 0.0, 0.0};
         secondDerivatives = new double[]{previousQuerySecondDerivative, 0.0, 0.0};
      }
      else if (nonFlatPercentageAlong < 1.0)
      {
//         colinearPoints = new Point3d[]{existingFlatPoint, nonflatPointProjection3d, newFlatPoint};
         pointsIn = new Point2d[]{existingFlat1D, nonFlat1D, newFlat1D};
         slopes = new double[]{0.0, previousQuerySlope, 0.0};
         secondDerivatives = new double[]{0.0, previousQuerySecondDerivative, 0.0};
      }
      else
      {
//         colinearPoints = new Point3d[]{existingFlatPoint, newFlatPoint, nonflatPointProjection3d};
         pointsIn = new Point2d[]{nonFlat1D, newFlat1D, existingFlat1D};
         slopes = new double[]{0.0, 0.0, previousQuerySlope};
         secondDerivatives = new double[]{0.0, 0.0, previousQuerySecondDerivative};
      }
      
      threePointDoubleSplines1D.setPoints(pointsIn, slopes, secondDerivatives);
  
   }


   public double[] getZSlopeAndSecondDerivative(Point2d queryPoint)
   {
      double percentageAlong = lineSegmentBetweenFlatPoints.percentageAlongLineSegment(queryPoint);
      double distanceAlong = percentageAlong * lineSegmentBetweenFlatPoints.length();
      
      double[] zSlopeAndSecondDerivative = threePointDoubleSplines1D.getZSlopeAndSecondDerivative(distanceAlong);
      
      return zSlopeAndSecondDerivative;
   }
   
   private Point2d convertPointTo2d(Point3d pointToConvert)
   {
      return new Point2d(pointToConvert.getX(), pointToConvert.getY());
   }
   


}
