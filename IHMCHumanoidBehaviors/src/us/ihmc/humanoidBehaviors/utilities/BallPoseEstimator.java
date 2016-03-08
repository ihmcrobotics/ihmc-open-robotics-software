package us.ihmc.humanoidBehaviors.utilities;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javax.vecmath.Point3f;

import bubo.clouds.FactoryPointCloudShape;
import bubo.clouds.detect.CloudShapeTypes;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.clouds.detect.PointCloudShapeFinder.Shape;
import bubo.clouds.detect.wrapper.ConfigMultiShapeRansac;
import bubo.clouds.detect.wrapper.ConfigSurfaceNormals;
import georegression.struct.point.Point3D_F64;
import georegression.struct.shapes.Sphere3D_F64;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModelFactory;
import us.ihmc.ihmcPerception.depthData.PointCloudDataReceiver;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.io.printing.PrintTools;

public class BallPoseEstimator
{
   private final static boolean DEBUG = false;
   ExecutorService executorService = Executors.newFixedThreadPool(2);
   SDFFullHumanoidRobotModelFactory modelFactory;
   final int pointDropFactor = 4;
   final float searchRadius = 2.0f;

   public BallPoseEstimator()
   {
   }

   public ArrayList<Sphere3D_F64> detectBalls(Point3f[] fullPoints)
   {

      ArrayList<Sphere3D_F64> foundBalls = new ArrayList<Sphere3D_F64>();
      // filter points
      ArrayList<Point3D_F64> pointsNearBy = new ArrayList<Point3D_F64>();
      for (Point3f tmpPoint : fullPoints)
      {
         pointsNearBy.add(new Point3D_F64(tmpPoint.x, tmpPoint.y, tmpPoint.z));
      }

      // find plane
      ConfigMultiShapeRansac configRansac = ConfigMultiShapeRansac.createDefault(30, 1.57, 0.010, CloudShapeTypes.SPHERE);
      configRansac.minimumPoints = 15;
      PointCloudShapeFinder findSpheres = FactoryPointCloudShape.ransacSingleAll( new ConfigSurfaceNormals(20, 0.10), configRansac);

      PrintStream out = System.out;
      System.setOut(new PrintStream(new OutputStream()
      {
         @Override
         public void write(int b) throws IOException
         {
         }
      }));
      try
      {
         findSpheres.process(pointsNearBy, null);
      } finally
      {
         System.setOut(out);
      }

      // sort large to small
      List<Shape> spheres = findSpheres.getFound();
      Collections.sort(spheres, new Comparator<Shape>()
      {
         
         @Override
         public int compare(Shape o1, Shape o2)
         {
            return Integer.compare(o1.points.size(), o2.points.size());
         };
      });

      if (spheres.size() > 0)
      {
         PrintTools.debug(DEBUG, this, "spheres.size() " + spheres.size());
      }
      for (Shape sphere : spheres)
      {
         Sphere3D_F64 sphereParams = (Sphere3D_F64) sphere.getParameters();
         PrintTools.debug(DEBUG, this, "sphere radius" + sphereParams.getRadius() + " center " + sphereParams.getCenter());

         if (sphereParams.getRadius() < 0.254)// soccer ball -
         {
            foundBalls.add(sphereParams);
            PrintTools.debug(DEBUG, this, "------Found Soccer Ball radius" + sphereParams.getRadius() + " center " + sphereParams.getCenter());

            RigidBodyTransform t = new RigidBodyTransform();
            t.setTranslation(sphereParams.getCenter().x, sphereParams.getCenter().y, sphereParams.getCenter().z);
         }

      }
      return foundBalls;

   }

}
