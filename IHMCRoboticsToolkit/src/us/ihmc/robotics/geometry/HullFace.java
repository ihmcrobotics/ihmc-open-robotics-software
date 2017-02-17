package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.shapes.Plane3d;
import us.ihmc.robotics.geometry.transformables.Pose;

/**
 * @author agrabertilton
 */
public class HullFace
{
   private Plane3d plane = new Plane3d();
   private List<Point3D> facePoints = new ArrayList<Point3D>();
   private double slopeAngle;
   private double area;

   public HullFace(List<Point3D> points)
   {
      setPoints(points);
      computeAndUpdate();
   }

   public HullFace(Point3D[] points)
   {
      setPoints(Arrays.asList(points));
      computeAndUpdate();
   }

   public HullFace(Point3D[] points, int[] vertexIndices)
   {
      setPoints(points, vertexIndices);
      computeAndUpdate();
   }

   public HullFace(List<Point3D> points, int[] vertexIndices)
   {
      setPoints(points, vertexIndices);
      computeAndUpdate();
   }

   private void computeAndUpdate()
   {
      if (facePoints == null || facePoints.size() < 3)
      {
         return;
      }

      plane.setToNaN();
      Point3D faceCenter = new Point3D();
      Vector3D planeNormal = new Vector3D();

      //find the fan of the points, which should be ordered
      Point3D p0 = facePoints.get(0);
      Vector3D edge01 = new Vector3D();
      Vector3D edge02 = new Vector3D();
      Vector3D singleCross = new Vector3D();
      int numPoints = facePoints.size();
      for (int i = 1; i < numPoints - 1; i++)
      {
         edge01.set(facePoints.get(i));
         edge02.set(facePoints.get(i + 1));
         edge01.sub(p0);
         edge02.sub(p0);
         singleCross.cross(edge01, edge02);
         planeNormal.add(singleCross);
      }

      planeNormal.scale(-1.0);
      area = planeNormal.length() / 2;
      planeNormal.normalize();
      slopeAngle = Math.acos(planeNormal.getZ());

      for (int i = 0; i < numPoints; i++)
      {
         faceCenter.add(facePoints.get(i));
      }

      faceCenter.scale(1.0 / numPoints);
      plane.setPoint(faceCenter);
      plane.setNormal(planeNormal);
   }

   public double getSlopeAngle()
   {
      return slopeAngle;
   }

   public double getArea()
   {
      return area;
   }

   public void getPlane(Plane3d planeToPack)
   {
      planeToPack.set(plane);
   }

   public List<Point3D> getPoints()
   {
      return facePoints;
   }

   private void setPoints(List<Point3D> points)
   {
      this.facePoints = points;
   }

   public void setPoints(Point3D[] points, int[] vertexIndecies)
   {
      int index;
      for (int i = 0; i < vertexIndecies.length; i++)
      {
         index = vertexIndecies[i];
         if (index >= points.length)
         {
            System.err.println(this.getClass().getSimpleName() + " Index out of bounds");
            facePoints.clear();
            return;
         }
         facePoints.add(points[index]);
      }
   }

   public void setPoints(List<Point3D> points, int[] vertexIndecies)
   {
      int index;
      int maxIndex = points.size();
      for (int i = 0; i < vertexIndecies.length; i++)
      {
         index = vertexIndecies[i];
         if (index >= maxIndex)
         {
            System.err.println(this.getClass().getSimpleName() + " Index out of bounds");
            facePoints.clear();
            return;
         }
         facePoints.add(points.get(index));
      }
   }

   public void get2DPolygonAndPose(ConvexPolygon2d polygonToPack, Pose polygonPose)
   {
      if (facePoints.isEmpty())
      {
         return;
      }

      List<Point3D> projectedPoints = new ArrayList<>();
      Point3D projectedPoint;
      Point3D averagePoint = new Point3D();
      for (Point3D point3d : facePoints)
      {
         projectedPoint = plane.orthogonalProjectionCopy(point3d);
         averagePoint.add(projectedPoint);
         projectedPoints.add(projectedPoint);
      }
      averagePoint.scale(1.0 / facePoints.size());
      polygonPose.setPosition(averagePoint);

      Vector3D xVec = new Vector3D(projectedPoints.get(0));
      xVec.sub(averagePoint);
      xVec.normalize();

      Vector3D zVec = plane.getNormalCopy();
      Vector3D yVec = new Vector3D();
      yVec.cross(zVec, xVec);
      yVec.normalize();

      RotationMatrix rotationMatrix = new RotationMatrix();
      rotationMatrix.setColumns(xVec, yVec, zVec);
      polygonPose.setOrientation(rotationMatrix);

      rotationMatrix.transpose();
      polygonToPack.clear();
      for (Point3D point : projectedPoints)
      {
         point.sub(averagePoint);
         rotationMatrix.transform(point);
         if (Math.abs(point.getZ()) > 1e-14)
         {
            System.out.println("Error in HullFace class, failed to get polygon");
         }
         polygonToPack.addVertex(point.getX(), point.getY());
      }
      polygonToPack.update();
   }
}
