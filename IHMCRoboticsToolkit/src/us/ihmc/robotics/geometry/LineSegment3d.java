package us.ihmc.robotics.geometry;

import us.ihmc.robotics.hyperCubeTree.OneDimensionalBounds;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class LineSegment3d
{
   private final Point3d pointA = new Point3d();
   private final Point3d pointB = new Point3d();

   private final Vector3d tempVector3dOne = new Vector3d();
   private final Vector3d tempVector3dTwo = new Vector3d();

   public LineSegment3d()
   {
   }

   public LineSegment3d(Point3d pointA, Point3d pointB)
   {
      set(pointA, pointB);
   }

   public void set(LineSegment3d lineSegment)
   {
      set(lineSegment.pointA, lineSegment.pointB);
   }

   public void set(Point3d pointA, Point3d pointB)
   {
      setPointA(pointA);
      setPointB(pointB);
   }

   public void set(double xA, double yA, double zA, double xB, double yB, double zB)
   {
      setPointA(xA, yA, zA);
      setPointB(xB, yB, zB);
   }

   public void setPointA(Point3d pointA)
   {
      this.pointA.set(pointA);
   }

   public void setPointA(double xA, double yA, double zA)
   {
      pointA.set(xA, yA, zA);
   }

   public void setPointB(Point3d pointB)
   {
      this.pointB.set(pointB);
   }

   public void setPointB(double xB, double yB, double zB)
   {
      pointB.set(xB, yB, zB);
   }

   public void applyTransform(RigidBodyTransform transform)
   {
      transform.transform(pointA);
      transform.transform(pointB);
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

      if (projectionOfpointC.epsilonEquals(pointA, 1e-14)) // pointC projection is below pointA
      {
         return pointC.distance(pointA);
      }
      else if (projectionOfpointC.epsilonEquals(pointB, 1e-14)) // pointC projection is up pointB
      {
         return pointC.distance(pointB);
      }

      return pointC.distance(new Point3d(projectionOfpointC)); // pointC projection is between pointA and pointB
   }

   public Point3d projection(Point3d pointC)
   {
      Point3d projection = new Point3d();

      getProjectionOntoLineSegment(pointC, projection);

      return projection;
   }

   public void getProjectionOntoLineSegment(Point3d pointC, Point3d projectedPointToPack)
   {
      Vector3d vectorAB = tempVector3dOne;
      vectorAB.sub(pointB, pointA);
      Vector3d vectorAC = tempVector3dTwo;
      vectorAC.sub(pointC, pointA);
      double temporaryDoubleA;
      if ((temporaryDoubleA = vectorAC.dot(vectorAB)) <= 0.0) // pointC projection is below pointA
      {
         projectedPointToPack.set(pointA.getX(), pointA.getY(), pointA.getZ());
         return;
      }

      double temporaryDoubleB;
      if ((temporaryDoubleB = vectorAB.dot(vectorAB)) <= temporaryDoubleA) // pointC projection is up pointB
      {
         projectedPointToPack.set(pointB.getX(), pointB.getY(), pointB.getZ());
         return;
      }

      double temporaryRoubleRatio;
      temporaryRoubleRatio = temporaryDoubleA / temporaryDoubleB; // pointC projection is between pointA and pointB
      Vector3d projectionOfPointC = tempVector3dTwo;
      projectionOfPointC.set(pointA);
      vectorAB.scale(temporaryRoubleRatio);
      projectionOfPointC.add(vectorAB);

      projectedPointToPack.set(projectionOfPointC.getX(), projectionOfPointC.getY(), projectionOfPointC.getZ());
   }

   public OneDimensionalBounds[] getOuterBounds()
   {
      double[] aVals = new double[3];
      pointA.get(aVals);
      double[] bVals = new double[3];
      pointB.get(bVals);
      OneDimensionalBounds[] ret = new OneDimensionalBounds[3];
      for (int i = 0; i < 3; i++)
      {
         ret[i] = new OneDimensionalBounds(Math.min(aVals[i], bVals[i]), Math.max(aVals[i], bVals[i]));
      }
      return ret;
   }

   public void getPoints(double[] aVals, double[] bVals)
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
