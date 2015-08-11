package us.ihmc.robotics.geometry;

import us.ihmc.robotics.dataStructures.Pose;
import us.ihmc.robotics.geometry.shapes.Plane3d;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * @author agrabertilton
 */
public class HullFace
{
   private Plane3d plane = new Plane3d();
   private List<Point3d> facePoints = new ArrayList<Point3d>();
   private double slopeAngle;
   private double area;

   public HullFace(List<Point3d> points)
   {
      setPoints(points);
      computeAndUpdate();
   }

   public HullFace(Point3d[] points)
   {
      setPoints(Arrays.asList(points));
      computeAndUpdate();
   }

   public HullFace(Point3d[] points, int[] vertexIndices)
   {
      setPoints(points, vertexIndices);
      computeAndUpdate();
   }

   public HullFace(List<Point3d> points, int[] vertexIndices)
   {
      setPoints(points, vertexIndices);
      computeAndUpdate();
   }

   private void computeAndUpdate()
   {
      if (facePoints == null || facePoints.size() < 3) { return; }

      plane.setToNaN();
      Point3d faceCenter = new Point3d();
      Vector3d planeNormal = new Vector3d();

      //find the fan of the points, which should be ordered
      Point3d p0 = facePoints.get(0);
      Vector3d edge01 = new Vector3d();
      Vector3d edge02 = new Vector3d();
      Vector3d singleCross = new Vector3d();
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
      slopeAngle = Math.acos(planeNormal.z);

      for (int i = 0; i < numPoints; i++)
      {
         faceCenter.add(facePoints.get(i));
      }

      faceCenter.scale(1.0 / numPoints);
      plane.setPoint(faceCenter);
      plane.setNormal(planeNormal);
   }

   public double getSlopeAngle() {return slopeAngle;}

   public double getArea() {return area;}

   public void getPlane(Plane3d planeToPack) {planeToPack.set(plane);}

   public List<Point3d> getPoints() {return facePoints;}

   private void setPoints(List<Point3d> points) { this.facePoints = points;}

   public void setPoints(Point3d[] points, int[] vertexIndecies)
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

   public void setPoints(List<Point3d> points, int[] vertexIndecies)
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

      List<Point3d> projectedPoints = new ArrayList<>();
      Point3d projectedPoint;
      Point3d averagePoint = new Point3d();
      for (Point3d point3d : facePoints)
      {
         projectedPoint = plane.orthogonalProjectionCopy(point3d);
         averagePoint.add(projectedPoint);
         projectedPoints.add(projectedPoint);
      }
      averagePoint.scale(1.0 / facePoints.size());
      polygonPose.getPoint().set(averagePoint);

      Vector3d xVec = new Vector3d(projectedPoints.get(0));
      xVec.sub(averagePoint);
      xVec.normalize();

      Vector3d zVec = plane.getNormalCopy();
      Vector3d yVec = new Vector3d();
      yVec.cross(zVec, xVec);
      yVec.normalize();

      Matrix3d rotationMatrix = new Matrix3d();
      rotationMatrix.setColumn(0, xVec);
      rotationMatrix.setColumn(1, yVec);
      rotationMatrix.setColumn(2, zVec);
      polygonPose.getOrientation().set(rotationMatrix);

      rotationMatrix.transpose();
      polygonToPack.clear();
      for (Point3d point : projectedPoints)
      {
         point.sub(averagePoint);
         rotationMatrix.transform(point);
         if (Math.abs(point.z) > 1e-14)
         {
            System.out.println("Error in HullFace class, failed to get polygon");
         }
         polygonToPack.addVertex(point.x, point.y);
      }
      polygonToPack.update();
   }
}
