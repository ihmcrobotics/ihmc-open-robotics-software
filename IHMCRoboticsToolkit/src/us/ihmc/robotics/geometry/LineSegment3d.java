package us.ihmc.robotics.geometry;

import us.ihmc.robotics.hyperCubeTree.OneDimensionalBounds;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class LineSegment3d
{
   private Point3d pointA;
   private Point3d pointB;

   public LineSegment3d(Point3d pointA, Point3d pointB)
   {
      this.pointA = pointA;
      this.pointB = pointB;
   }

   public double length()
   {
      return Math.sqrt((pointB.getX() - pointA.getX()) * (pointB.getX() - pointA.getX()) + (pointB.getY() - pointA.getY()) * (pointB.getY() - pointA.getY())
                       + (pointB.getZ() - pointA.getZ()) * (pointB.getZ() - pointA.getZ()));
   }

   public double distanceToAPoint(Point3d pointC)
   {
      Vector3d projectionOfpointC = new Vector3d();
      projectionOfpointC.set(projection(pointC));

      if (projectionOfpointC.epsilonEquals(pointA, 1e-14))    // pointC projection is below pointA
      {
         return pointC.distance(pointA);
      }
      else if (projectionOfpointC.epsilonEquals(pointB, 1e-14))    // pointC projection is up pointB
      {
         return pointC.distance(pointB);
      }

      return pointC.distance(new Point3d(projectionOfpointC));    // pointC projection is between pointA and pointB
   }

   public Point3d projection(Point3d pointC)
   {
      Vector3d vectorAB = new Vector3d(pointB);
      vectorAB.sub(pointA);
      Vector3d vectorAC = new Vector3d(pointC);
      vectorAC.sub(pointA);
      double temporaryDoubleA;
      if ((temporaryDoubleA = vectorAC.dot(vectorAB)) <= 0.0)    // pointC projection is below pointA
      {
         return new Point3d(pointA);
      }

      double temporaryDoubleB;
      if ((temporaryDoubleB = vectorAB.dot(vectorAB)) <= temporaryDoubleA)    // pointC projection is up pointB
      {
         return new Point3d(pointB);
      }

      double temporaryRoubleRatio;
      temporaryRoubleRatio = temporaryDoubleA / temporaryDoubleB;    // pointC projection is between pointA and pointB
      Vector3d projectionOfPointC = new Vector3d(pointA);
      vectorAB.scale(temporaryRoubleRatio);
      projectionOfPointC.add(vectorAB);

      return new Point3d(projectionOfPointC);
   }
   public OneDimensionalBounds[] getOuterBounds()
   {
      double[] aVals = new double[3];
      pointA.get(aVals);
      double[] bVals = new double[3];
      pointB.get(bVals);
      OneDimensionalBounds[] ret = new OneDimensionalBounds[3];
      for (int i=0;i<3;i++)
      {
         ret[i]=new OneDimensionalBounds(Math.min(aVals[i],bVals[i]), Math.max(aVals[i], bVals[i]));
      }
      return ret;
   }

   public void packPoints(double[] aVals, double[] bVals)
   {
      pointA.get(aVals);
      pointB.get(bVals);

   }

   public Vector3d getDirection()
   {
      Vector3d direction = new Vector3d(pointB);
      direction.sub(pointA);
      direction.normalize();

      return direction;
   }

   public Point3d getPointA()
   {
      return pointA;
   }

   public Point3d getPointB()
   {
      return pointB;
   }
}
