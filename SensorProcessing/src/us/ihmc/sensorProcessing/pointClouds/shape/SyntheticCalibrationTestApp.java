package us.ihmc.sensorProcessing.pointClouds.shape;

import georegression.metric.Intersection3D_F64;
import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.shapes.Cylinder3D_F64;
import georegression.struct.shapes.Sphere3D_F64;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import bubo.ptcloud.CloudShapeTypes;
import bubo.ptcloud.FactoryPointCloudShape;
import bubo.ptcloud.PointCloudShapeFinder;
import bubo.ptcloud.PointCloudShapeFinder.Shape;
import bubo.ptcloud.alg.ConfigSchnabel2007;
import bubo.ptcloud.wrapper.ConfigMergeShapes;
import bubo.ptcloud.wrapper.ConfigSurfaceNormals;

import com.jme3.app.SimpleApplication;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;

/**
 * @author Alex Lesman
 */
public class SyntheticCalibrationTestApp extends SimpleApplication
{
   private static Random rand = new Random(System.currentTimeMillis());

   Sphere3D_F64 truthSphere = new Sphere3D_F64(2, 2, 10, 3);
   Cylinder3D_F64 truthCylinder = new Cylinder3D_F64(0, 1, 3, -1, 1, 0.5, 1);
   PlaneNormal3D_F64 truthPlane = new PlaneNormal3D_F64(2, 2, 10, 0, 1, 0);

   Node zUpNode = new Node();

   public static void main(String[] args)
   {
      SyntheticCalibrationTestApp test1 = new SyntheticCalibrationTestApp();

      test1.start();

   }

   private static List<Point3D_F64> createBoxCloud(Point3D_F64 center, double size, double noise)
   {
      List<Point3D_F64> cloud = new ArrayList<Point3D_F64>();
      noise /=2; //gives 95% of gaussian noise within range

      Vector3D_F64[] sides = new Vector3D_F64[] { new Vector3D_F64(1, 1, 0), new Vector3D_F64(1, 0, 1), new Vector3D_F64(0, 1, 1) };
      Vector3D_F64[] noises = new Vector3D_F64[] { new Vector3D_F64(0, 0, noise), new Vector3D_F64(0, noise, 0), new Vector3D_F64(noise, 0, 0) };

      for (int side = 0; side < sides.length; side++)
      {
         Vector3D_F64 sideVector = sides[side];
         Vector3D_F64 noiseVector = noises[side];
         for (int i = 0; i < 500; i++)
         {
            double x = (size * (rand.nextDouble() - .5) * sideVector.x) - (center.x - (sideVector.x * size / 2));
            double y = (size * (rand.nextDouble() - .5) * sideVector.y) - (center.y - (sideVector.y * size / 2));
            double z = (size * (rand.nextDouble() - .5) * sideVector.z) - (center.z - (sideVector.z * size / 2));            
            
            
            x += ((rand.nextGaussian() - .5) * noiseVector.x) - (center.x);
            y += ((rand.nextGaussian() - .5) * noiseVector.y) - (center.y);
            z += ((rand.nextGaussian() - .5) * noiseVector.z) - (center.z);

            cloud.add(new Point3D_F64(x, y, z));
         }
      }
      System.out.println(cloud);
      return cloud;
   }

   private List<Point3D_F64> createCloudOfPoints()
   {
      List<Point3D_F64> cloud = new ArrayList<Point3D_F64>();

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

   @Override
   public void simpleInitApp()
   {
      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());

      List<Point3D_F64> cloud = createBoxCloud(new Point3D_F64(0, 0, 0), -1, .05);

      ConfigSchnabel2007 configRansac = ConfigSchnabel2007.createDefault(100, 0.5, 0.1, 0.05, CloudShapeTypes.PLANE);
      configRansac.minModelAccept = 300;
      configRansac.octreeSplit = 300;
      configRansac.ransacExtension = 200;
      configRansac.maximumAllowedIterations *= 2;

      ConfigSurfaceNormals configSurface = new ConfigSurfaceNormals(6, 20, 3);
      ConfigMergeShapes configMerge = new ConfigMergeShapes(0.6, 0.9);

      PointCloudShapeFinder shapeFinder = FactoryPointCloudShape.ransacOctree(configSurface, configRansac, configMerge);

      shapeFinder.process(cloud, null);


      orient(shapeFinder);
      render(shapeFinder);

   }
   
   private void orient(PointCloudShapeFinder shapeFinder) {
      ArrayList<PlaneGeneral3D_F64> planes = new ArrayList<PlaneGeneral3D_F64>();
      
      for (Shape shape : shapeFinder.getFound()) {
         System.out.println(shape.type);
         if (shape.type == CloudShapeTypes.PLANE) 
            planes.add((PlaneGeneral3D_F64)shape.parameters);
         
      }

      System.out.println("Num Planes: " + planes.size());
      if (planes.size() < 3)
         return;
      
      LineParametric3D_F64 line = new LineParametric3D_F64();
      Point3D_F64 point = new Point3D_F64();
      Intersection3D_F64.intersect(planes.get(0), planes.get(1), line);
      Intersection3D_F64.intersect(planes.get(2), line, point);
      
      System.out.println(point);
   }

   private void render(PointCloudShapeFinder shapeFinder)
   {
      List<PointCloudShapeFinder.Shape> found = shapeFinder.getFound();
      List<Point3D_F64> unmatched = new ArrayList<Point3D_F64>();
      shapeFinder.getUnmatched(unmatched);

      System.out.println("Unmatched points " + unmatched.size());
      System.out.println("total shapes found: " + found.size());
      int total = 0;

      Shape unShape = new Shape();
      unShape.points = unmatched;
      found.add(unShape);

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
         rootNode.attachChild(zUpNode);
         zUpNode.attachChild(generator.generatePointCloudGraph(points, colors));
      }
      catch (Exception e)
      {
         this.handleError(e.getMessage(), e);
      }

      cam.setFrustumPerspective(45.0f, ((float) cam.getWidth()) / ((float) cam.getHeight()), 0.05f, 100.0f);
      cam.setLocation(new Vector3f(0, 0, -5));
      cam.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);
      cam.update();
      flyCam.setMoveSpeed(25);
   }
}
