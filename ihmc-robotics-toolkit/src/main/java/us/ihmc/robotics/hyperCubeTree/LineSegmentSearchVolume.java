package us.ihmc.robotics.hyperCubeTree;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.tuple2D.Point2D;

public class LineSegmentSearchVolume extends HyperVolume
{
   final int dim;
   final private double[] pointA;
   final private double[] pointB;
   final private double[] scale;
   public LineSegmentSearchVolume(LineSegment3D lineSegment)
   {
      super(3, computeOuterBounds(lineSegment), null, true);
      dim = 3;
      pointA = new double[dim];
      pointB = new double[dim];
      scale = new double[dim];
      lineSegment.getFirstEndpoint().get(pointA);
      lineSegment.getSecondEndpoint().get(pointB);
      for (int i=0;i<dim;i++)
      {
         scale[i]=1.0/(pointB[i]-pointA[i]);
      }  
   }
   private static OneDimensionalBounds[] computeOuterBounds(LineSegment3D lineSegment)
   {
      double[] aVals = new double[3];
      lineSegment.getFirstEndpoint().get(aVals);
      double[] bVals = new double[3];
      lineSegment.getSecondEndpoint().get(bVals);
      OneDimensionalBounds[] ret = new OneDimensionalBounds[3];
      for (int i = 0; i < 3; i++)
      {
         ret[i] = new OneDimensionalBounds(Math.min(aVals[i], bVals[i]), Math.max(aVals[i], bVals[i]));
      }
      return ret;
   }
   public LineSegmentSearchVolume(Point2D point1, Point2D point2)
   {
      this(new double[]{point1.getX(),point1.getY()},new double[]{point2.getX(),point2.getY()});
   }
   
   public LineSegmentSearchVolume(double[] point1, double[] point2)
   {
      super(point1.length,calculateBounds(point1, point2), null,true);
      dim = point1.length;
      pointA=point1;
      pointB=point2;
      scale = new double[dim];
      for (int i=0;i<dim;i++)
      {
         scale[i]=1.0/(pointB[i]-pointA[i]);
      }  
   }
   private static OneDimensionalBounds[] calculateBounds(double[] point1, double[] point2)
   {
      if (point1.length != point2.length)
         throw new DimensionalityMismatchException();
      int dim = point1.length;
      OneDimensionalBounds[] bounds = new OneDimensionalBounds[dim];
      for (int i=0;i<dim;i++)
         bounds[i]=new OneDimensionalBounds(point1[i], point2[i]);
      return bounds;
   }

   
   protected boolean complexBoundsIntersect(OneDimensionalBounds[] bounds)
   {
      OneDimensionalBounds boundaryRepresentedInLineParameterSpace;
      OneDimensionalBounds accumulatorBounds = new OneDimensionalBounds(0.0,1.0);
      for (int i=0;i<dim;i++)
      {
         boundaryRepresentedInLineParameterSpace = transformOneDimensionalBoundsToLineParameterSpace(bounds, i);
         accumulatorBounds=accumulatorBounds.intersectionWith(boundaryRepresentedInLineParameterSpace);
         if (null == accumulatorBounds)
            return false;
      }
      return true;
   }

   private OneDimensionalBounds transformOneDimensionalBoundsToLineParameterSpace(OneDimensionalBounds[] bounds, int i)
   {
      double scale = this.scale[i];
      double translation = pointA[i];
      if (Double.isInfinite(scale))
         return OneDimensionalBounds.unbounded();
      return new OneDimensionalBounds(((bounds[i].min()-translation)*scale),((bounds[i].max()-translation)*scale));
   }

   protected boolean containsBoundsIfWithinOuterBounds(OneDimensionalBounds[] bounds)
   {
      return false;
   }

   protected boolean containsPointIfWithinOuterBounds(double[] point)
   {
      return false;
   }


   @Override
   protected double[] pointWithin(OneDimensionalBounds[] bounds)
   {
      OneDimensionalBounds sBounds = sBoundsWithin(bounds);
      double sValue = sBounds.midpoint();
      double[] ret = new double[dim];
      for (int i = 0; i<dim ; i++ )
      {
         ret[i]=pointA[i]*(1-sValue)+pointB[i]*sValue;
      }
      return ret;
   }
   
   private OneDimensionalBounds sBoundsWithin(OneDimensionalBounds[] bounds)
   {
      OneDimensionalBounds boundaryRepresentedInLineParameterSpace;
      OneDimensionalBounds accumulatorBounds = new OneDimensionalBounds(0.0,1.0);
      for (int i=0;i<dim;i++)
      {
         boundaryRepresentedInLineParameterSpace = transformOneDimensionalBoundsToLineParameterSpace(bounds, i);
         accumulatorBounds=accumulatorBounds.intersectionWith(boundaryRepresentedInLineParameterSpace);
         if (null == accumulatorBounds)
            return null;
      }
      return accumulatorBounds;
   }

}
