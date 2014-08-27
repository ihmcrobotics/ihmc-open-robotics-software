package us.ihmc.sensorProcessing.pointClouds.shape;

import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.shapes.Cylinder3D_F64;
import georegression.struct.shapes.Sphere3D_F64;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.sensorProcessing.pointClouds.shape.ExpectationMaximizationFitter.ScoringFunction;
import bubo.clouds.FactoryPointCloudShape;
import bubo.clouds.detect.CloudShapeTypes;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.clouds.detect.PointCloudShapeFinder.Shape;
import bubo.clouds.detect.alg.ConfigSchnabel2007;
import bubo.clouds.detect.wrapper.ConfigRemoveFalseShapes;
import bubo.clouds.detect.wrapper.ConfigSurfaceNormals;

import com.jme3.app.SimpleApplication;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;

/**
 * @author Alex Lesman
 */
public class SyntheticCalibrationTestApp extends SimpleApplication
{
   private static Random rand = new Random(123);

   private ShapeTranslator translator = new ShapeTranslator(this);

   Sphere3D_F64 truthSphere = new Sphere3D_F64(2, 2, 10, 3);
   Cylinder3D_F64 truthCylinder = new Cylinder3D_F64(0, 1, 3, -1, 1, 0.5, 1);
   PlaneNormal3D_F64 truthPlane = new PlaneNormal3D_F64(2, 2, 10, 0, 1, 0);

   Node zUpNode = new Node();

   public static void main(String[] args)
   {
      SyntheticCalibrationTestApp test1 = new SyntheticCalibrationTestApp();

      test1.start();

   }

   public static List<Point3D_F64> createBoxCloud(Point3D_F64 center, int numPoints, double size, double noise)
   {
      List<Point3D_F64> cloud = new ArrayList<Point3D_F64>();
      noise /= 3; //gives 98% of gaussian noise within range

      Vector3D_F64[] sides = new Vector3D_F64[] { new Vector3D_F64(1, 1, 0), new Vector3D_F64(1, 0, 1), new Vector3D_F64(0, 1, 1) };

      for (int side = 0; side < sides.length; side++)
      {
         Vector3D_F64 sideVector = sides[side];
         for (int i = 0; i < numPoints; i++)
         {
            double x = (size * (rand.nextDouble() - .5) * sideVector.x) + (center.x - (sideVector.x * size / 2));
            double y = (size * (rand.nextDouble() - .5) * sideVector.y) + (center.y - (sideVector.y * size / 2));
            double z = (size * (rand.nextDouble() - .5) * sideVector.z) + (center.z - (sideVector.z * size / 2));

            x += ((rand.nextGaussian() - .5) * noise);
            y += ((rand.nextGaussian() - .5) * noise);
            z += ((rand.nextGaussian() - .5) * noise);

            cloud.add(new Point3D_F64(x, y, z));
         }
      }
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

      List<Point3D_F64> cloud = createBoxCloud(new Point3D_F64(1, 2, 3), 1000, 1, 0.05);
      PointCloudShapeFinder shapeFinder = applyRansac(cloud);

      List<PlaneGeneral3D_F64> planes = new ArrayList<PlaneGeneral3D_F64>();
      HashSet<Point3D_F64> pointSet = new HashSet<Point3D_F64>();

      for (PointCloudShapeFinder.Shape shape : shapeFinder.getFound())
      {
         if (shape.type == CloudShapeTypes.PLANE)
         {
            planes.add((PlaneGeneral3D_F64) shape.parameters);
            pointSet.addAll(shape.points);
         }
      }

      List<Point3D_F64> allPoints = new LinkedList<Point3D_F64>(pointSet);


      ScoringFunction sf = ExpectationMaximizationFitter.getGaussianSqauresMixedError(.05 / 2);
      CubeCalibration.orient(planes = ExpectationMaximizationFitter.fit(3, rand, allPoints, sf, 16));
      double[][] weights = ExpectationMaximizationFitter.getWeights(null, planes, cloud, sf);
      List<Shape> emFitShapes = new ArrayList<PointCloudShapeFinder.Shape>();

      for (int i = 0; i < planes.size(); i++)
      {
         Shape s = new Shape();
         s.type = CloudShapeTypes.PLANE;
         s.points = new ArrayList<Point3D_F64>();
         s.parameters = planes.get(i);
         emFitShapes.add(s);
         for (int j = 0; j < cloud.size(); j++)
         {
            if (weights[i][j] >= .5)
               s.points.add(cloud.get(j));
         }
      }

      render(emFitShapes);

      //render(fitShapes);

      //render(shapeFinder.getFound());
   }

   private PointCloudShapeFinder applyRansac(List<Point3D_F64> cloud)
   {
      CloudShapeTypes shapeTypes[] = new CloudShapeTypes[] { CloudShapeTypes.PLANE };//, CloudShapeTypes.CYLINDER, CloudShapeTypes.SPHERE};

      ConfigSchnabel2007 configRansac = ConfigSchnabel2007.createDefault(20, 0.8, 0.02, shapeTypes);
      configRansac.randomSeed = 2342342;

      configRansac.minModelAccept = 100;
      configRansac.octreeSplit = 25;

      configRansac.maximumAllowedIterations = 1000;
      configRansac.ransacExtension = 25;

      //configRansac.models.get(1).modelCheck = new CheckShapeCylinderRadius(0.2);
      //configRansac.models.get(2).modelCheck = new CheckShapeSphere3DRadius(0.2);

      ConfigSurfaceNormals configSurface = new ConfigSurfaceNormals(30, Double.MAX_VALUE);
      ConfigRemoveFalseShapes configMerge = new ConfigRemoveFalseShapes(0.7);

      PointCloudShapeFinder shapeFinder = FactoryPointCloudShape.ransacOctree(configSurface, configRansac, configMerge);

      shapeFinder.process(cloud, null);
      return shapeFinder;
   }

   private void render(List<Shape> found)
   {
      List<Point3D_F64> unmatched = new ArrayList<Point3D_F64>();

      int total = 0;

      PointCloudShapeFinder.Shape unShape = new PointCloudShapeFinder.Shape();
      unShape.points = unmatched;
      found.add(unShape);

      for (PointCloudShapeFinder.Shape s : found)
      {
         total += s.points.size();
      }

      Vector3f[] points = new Vector3f[total];
      ColorRGBA[] colors = new ColorRGBA[total];

      int index = 0;
      float hue = 0.0f;
      for (PointCloudShapeFinder.Shape s : found)
      {
         int c = Color.HSBtoRGB(hue += (1.0 / found.size()), 1.0f, 1.0f);
         ColorRGBA color = new ColorRGBA(((c >> 16) & 0xFF) / 256.0f, ((c >> 8) & 0xFF) / 256.0f, ((c >> 0) & 0xFF) / 256.0f, 1.0f);

         if (s.type != null && s.points.size() > 0)
         {
            translator.translateShape(s, color, zUpNode);
         }

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
         zUpNode.attachChild(generator.generatePointCloudGraph(points, colors, 0.75f));
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
