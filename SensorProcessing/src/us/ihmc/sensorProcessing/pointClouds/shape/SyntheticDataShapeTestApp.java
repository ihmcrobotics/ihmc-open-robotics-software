package us.ihmc.sensorProcessing.pointClouds.shape;

import bubo.ptcloud.FactoryPointCloudShape;
import bubo.ptcloud.PointCloudShapeFinder;
import bubo.ptcloud.alg.ConfigSchnabel2007;
import bubo.ptcloud.tools.PointCloudShapeTools;
import bubo.ptcloud.wrapper.ConfigMergeShapes;
import bubo.ptcloud.wrapper.ConfigSurfaceNormals;
import com.jme3.app.SimpleApplication;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.shapes.Cylinder3D_F64;
import georegression.struct.shapes.Sphere3D_F64;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * @author Peter Abeles
 */
public class SyntheticDataShapeTestApp extends SimpleApplication
{
   Random rand = new Random(234);

   Sphere3D_F64 truthSphere = new Sphere3D_F64(2, 2, 10, 3);
   Cylinder3D_F64 truthCylinder = new Cylinder3D_F64(0, 1, 3, -1, 1, 0.5, 1);
   PlaneNormal3D_F64 truthPlane = new PlaneNormal3D_F64(2, 2, 10, 0, 1, 0);


   public static void main(String[] args)
   {
      SyntheticDataShapeTestApp test1 = new SyntheticDataShapeTestApp();
      test1.start();
   }

   @Override
   public void simpleInitApp()
   {
      List<Point3D_F64> cloud = createCloudOfPoints();

      ConfigSchnabel2007 configRansac = ConfigSchnabel2007.createDefault(100, 0.3, 0.1, 0.05);
      configRansac.minModelAccept = 100;
      configRansac.octreeSplit = 100;

      ConfigSurfaceNormals configSurface = new ConfigSurfaceNormals(6, 20, 3);
      ConfigMergeShapes configMerge = new ConfigMergeShapes(0.6, 0.9);

      PointCloudShapeFinder shapeFinder = FactoryPointCloudShape.ransacOctree(configSurface, configRansac, configMerge);

      shapeFinder.process(cloud, null);

      List<PointCloudShapeFinder.Shape> found = shapeFinder.getFound();

      List<Point3D_F64> unmatched = new ArrayList<Point3D_F64>();
      shapeFinder.getUnmatched(unmatched);

      System.out.println("Unmatched points " + unmatched.size());
      System.out.println("total shapes found: " + found.size());
      int total = 0;
      for (PointCloudShapeFinder.Shape s : found)
      {
         System.out.println("  " + s.type + "  points = " + s.points.size());
         System.out.println("           " + s.parameters);
         total += s.points.size();
      }

      Vector3f[] points = new Vector3f[total];
      ColorRGBA[] colors = new ColorRGBA[total];

      int index = 0;
      for (PointCloudShapeFinder.Shape s : found)
      {
         float r = rand.nextFloat();
         float g = rand.nextFloat();
         float b = rand.nextFloat();

         // make sure it is bright enough to see
         float n = (r + g + b) / 3.0f;

         if (n < 0.5f)
         {
            r *= 0.5f / n;
            g *= 0.5f / n;
            b *= 0.5f / n;
         }


         ColorRGBA color = new ColorRGBA(r, g, b, 1.0f);
         System.out.println(" color " + r + " " + g + " " + b);

         for (Point3D_F64 p : s.points)
         {
            points[index] = new Vector3f((float) p.x, (float) p.y, (float) p.z);
            colors[index] = color;
            index++;
         }
      }


      PointCloud generator = new PointCloud(assetManager);


      try
      {
         rootNode.attachChild(generator.generatePointCloudGraph(points, colors));
      }
      catch (Exception e)
      {
         this.handleError(e.getMessage(), e);
      }

      cam.setFrustumPerspective(45.0f, ((float) cam.getWidth()) / ((float) cam.getHeight()), 0.05f, 100.0f);
      cam.setLocation(new Vector3f(0, 0, -5));
      cam.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);
      cam.update();
      flyCam.setMoveSpeed(200);
   }

   private List<Point3D_F64> createCloudOfPoints()
   {
      List<Point3D_F64> cloud = new ArrayList<Point3D_F64>();

      for (int i = 0; i < 300; i++)
      {
         double phi = 2.0 * Math.PI * rand.nextDouble();
         double theta = 2.0 * Math.PI * rand.nextDouble();

         Point3D_F64 p = PointCloudShapeTools.createPt(truthSphere, phi, theta);

         cloud.add(p);
      }

      for (int i = 0; i < 300; i++)
      {
         double z = 2.0 * rand.nextDouble();
         double theta = 2.0 * Math.PI * rand.nextDouble();

         Point3D_F64 p = PointCloudShapeTools.createPt(truthCylinder, z, theta);

         cloud.add(p);
      }

      for (int i = 0; i < 1000; i++)
      {
         double x = 7.0 * (rand.nextDouble() - 0.5);
         double y = 7.0 * (rand.nextDouble() - 0.5);

         Point3D_F64 p = PointCloudShapeTools.createPt(truthPlane, x, y);

         cloud.add(p);
      }

      // noise
      for (int i = 0; i < 1000; i++)
      {
         double x = 14.0 * (rand.nextDouble() - 0.5);
         double y = 14.0 * (rand.nextDouble() - 0.5);
         double z = 14.0 * (rand.nextDouble() - 0.5);

         cloud.add(new Point3D_F64(x, y, z));
      }


      return cloud;
   }
}
