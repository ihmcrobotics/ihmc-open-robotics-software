package us.ihmc.perception.elements;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

public class CameraModel
{
   private RigidBodyTransform transform;

   private final DMatrixRMaj homogenousPoint = new DMatrixRMaj(4,1);
   private final DMatrixRMaj projectionMatrix = new DMatrixRMaj(3,4);
   private final DMatrixRMaj transformMatrix = new DMatrixRMaj(4,4);

   public CameraModel(double fx, double fy, double cx, double cy, Pose3D pose)
   {
      projectionMatrix.set(0,0, fx);
      projectionMatrix.set(1,1, fy);
      projectionMatrix.set(0,3, cx);
      projectionMatrix.set(1,3, cy);
      projectionMatrix.set(2,2, 1.0);

      transform = new RigidBodyTransform(pose);
      transform.invert();
      transform.get(transformMatrix);

      LogTools.info("Transform: {}", transformMatrix);
   }

   public Point2D project(Point3D point)
   {
      point.applyTransform(transform);

      homogenousPoint.set(0,0, point.getX());
      homogenousPoint.set(1,0, point.getY());
      homogenousPoint.set(2,0, point.getZ());
      homogenousPoint.set(3,0, 1.0);

      Point2D projection = new Point2D();


      DMatrixRMaj result = new DMatrixRMaj(3,1);
      CommonOps_DDRM.mult(projectionMatrix, homogenousPoint, result);

      LogTools.info("Projection: {}", result);

      projection.setX(result.get(0,0) / result.get(2,0));
      projection.setY(result.get(1,0) / result.get(2,0));



      return projection;
   }

   public static void main(String[] args)
   {
      ArrayList<Point3D> points = new ArrayList<>();
      points.add(new Point3D(10.0,10.0,10.0));
      points.add(new Point3D(-10.0,10.0,10.0));
      points.add(new Point3D(-10.0,-10.0,10.0));
      points.add(new Point3D(10.0,-10.0,10.0));
      points.add(new Point3D(10.0,10.0,-10.0));
      points.add(new Point3D(-10.0,10.0,-10.0));
      points.add(new Point3D(-10.0,-10.0,-10.0));
      points.add(new Point3D(10.0,-10.0,-10.0));

      int i = 0;
      double theta = 0;
      double radius = 30.0;

      Pose3D pose = new Pose3D();

      for (; i < 8; ++i, theta += 2 * Math.PI / 8)
      {

         Point3D position = new Point3D(radius * Math.cos(theta), radius * Math.sin(theta), 0.0);

         pose.setRotationYawAndZeroTranslation(theta);
         pose.appendTranslation(position);

         CameraModel camera = new CameraModel(50.0f,50.0f,50.0f,50.0f, pose);

         for(int j = 0; j<points.size(); j++)
         {
            Point2D measurement = camera.project(points.get(j));

            LogTools.info("Point: {} -> Projection: {}", points.get(j), measurement);
         }
      }
   }
}
