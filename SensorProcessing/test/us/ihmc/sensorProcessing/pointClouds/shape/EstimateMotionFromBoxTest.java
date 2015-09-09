package us.ihmc.sensorProcessing.pointClouds.shape;

import static junit.framework.Assert.assertEquals;
import georegression.geometry.GeometryMath_F64;
import georegression.geometry.RotationMatrixGenerator;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.sensorProcessing.pointClouds.shape.ExpectationMaximizationFitter.ScoringFunction;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import bubo.clouds.FactoryPointCloudShape;
import bubo.clouds.detect.CloudShapeTypes;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.clouds.detect.PointCloudShapeFinder.Shape;
import bubo.clouds.detect.alg.ConfigSchnabel2007;
import bubo.clouds.detect.wrapper.ConfigRemoveFalseShapes;
import bubo.clouds.detect.wrapper.ConfigSurfaceNormals;

/**
 * @author Peter Abeles
 */
public class EstimateMotionFromBoxTest
{
   private static Random rand = new Random(154213l);
   private static ConfigSurfaceNormals configNormal = new ConfigSurfaceNormals(30, .25);

   // planes in canonical frame
   PlaneNormal3D_F64 left = new PlaneNormal3D_F64(0,0,0,1,0,0);
   PlaneNormal3D_F64 right = new PlaneNormal3D_F64(0,0,0,0,1,0);
   PlaneNormal3D_F64 top = new PlaneNormal3D_F64(0,0,1.5,0,0,1);

   Point3D_F64 testP = new Point3D_F64(3,-5,6);
   Point3D_F64 foundP = new Point3D_F64();
   Point3D_F64 expectedP = new Point3D_F64();

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void translation() {
      Se3_F64 canonicalToA =  new Se3_F64();
      Se3_F64 canonicalToB =  new Se3_F64();

      canonicalToA.T.set(1,-0.5,-0.23);
      canonicalToB.T.set(2,0.05,-0.4);

      checkCase(new EstimateMotionFromBox(), canonicalToA, canonicalToB);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void rotation() {
      Se3_F64 canonicalToA =  new Se3_F64();
      Se3_F64 canonicalToB =  new Se3_F64();

      RotationMatrixGenerator.eulerXYZ(0.1,-0.5,0.05,canonicalToA.getR());
      RotationMatrixGenerator.eulerXYZ(0.3,-0.7,-0.12,canonicalToB.getR());

      checkCase(new EstimateMotionFromBox(), canonicalToA, canonicalToB);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void both() {
      Se3_F64 canonicalToA =  new Se3_F64();
      Se3_F64 canonicalToB =  new Se3_F64();

      canonicalToA.T.set(1,-0.5,-0.23);
      canonicalToB.T.set(2,0.05,-0.4);
      RotationMatrixGenerator.eulerXYZ(0.1,-0.5,0.05,canonicalToA.getR());
      RotationMatrixGenerator.eulerXYZ(0.3,-0.7,-0.12,canonicalToB.getR());

      checkCase(new EstimateMotionFromBox(), canonicalToA, canonicalToB);
   }

	@DeployableTestMethod(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void randomTransforms()
   {
      EstimateMotionFromBox alg = new EstimateMotionFromBox();

      for( int i = 0; i < 100; i++ ) {
         Se3_F64 canonicalToA =  randomTran();
         Se3_F64 canonicalToB =  randomTran();

         checkCase(alg, canonicalToA, canonicalToB);
      }
   }

   private void checkCase(EstimateMotionFromBox alg, Se3_F64 canonicalToA, Se3_F64 canonicalToB)
   {
      Se3_F64 aToB = canonicalToA.invert(null).concat(canonicalToB,null);
      Se3_F64 found;

      PlaneNormal3D_F64 leftA = create(left,canonicalToA);
      PlaneNormal3D_F64 rightA = create(right,canonicalToA);
      PlaneNormal3D_F64 topA = create(top,canonicalToA);

      PlaneNormal3D_F64 leftB = create(left,canonicalToB);
      PlaneNormal3D_F64 rightB = create(right,canonicalToB);
      PlaneNormal3D_F64 topB = create(top,canonicalToB);

      alg.setSrc(leftA,rightA,topA);
      alg.computeMotion(leftB,rightB,topB);

      found = alg.getMotionSrcToDst();

      SePointOps_F64.transform(aToB, testP, expectedP);
      SePointOps_F64.transform(found,testP,foundP);

      assertEquals(aToB.T.x,found.T.x,1e-8);
      assertEquals(aToB.T.y,found.T.y,1e-8);
      assertEquals(aToB.T.z,found.T.z,1e-8);

      assertEquals(expectedP.x,foundP.x,1e-8);
      assertEquals(expectedP.y,foundP.y,1e-8);
      assertEquals(expectedP.z,foundP.z,1e-8);
   }

   private PlaneNormal3D_F64 create( PlaneNormal3D_F64 plane , Se3_F64 tran ) {
      PlaneNormal3D_F64 b = new PlaneNormal3D_F64();
      SePointOps_F64.transform(tran,plane.p,b.p);
      GeometryMath_F64.mult(tran.R,plane.n,b.n);

      return b;
   }

   private Se3_F64 randomTran() {
      Se3_F64 a = new Se3_F64();

      a.T.x = rand.nextGaussian()*2;
      a.T.y = rand.nextGaussian()*2;
      a.T.z = rand.nextGaussian()*2;

      RotationMatrixGenerator.eulerXYZ(rand.nextDouble(),rand.nextDouble(),rand.nextDouble(),a.R);

      return a;
   }

	@DeployableTestMethod(estimatedDuration = 1.7)
	@Test(timeout = 30000)
   public void testMovingBox()
   {
      EstimateMotionFromBox boxMotion = new EstimateMotionFromBox();
      
      List<Point3D_F64> cloud = PointCloudTools.boundSphere(PointCloudTools.readPointCloud("../SensorProcessing/box_5s.txt", 1000000), new Point3D_F64(4.1, -0.55, -0.75), 1);
      List<PlaneNormal3D_F64> cube = emFitCube(cloud);

      boxMotion.setSrc(cube.get(0), cube.get(1), cube.get(2));
      
      Se3_F64 transform = new Se3_F64(new DenseMatrix64F(new double[][]{{1,0,0}, {0,1,0}, {0,0,1}}), new Vector3D_F64(0,0,1));
      PointCloudTools.transformCloud(cloud, transform);
      cube = emFitCube(cloud);

      boxMotion.computeMotion(cube.get(0), cube.get(1), cube.get(2));

      System.out.println(boxMotion.centerA);
      System.out.println(boxMotion.centerB);
   }
   
   


   private static List<PlaneNormal3D_F64> emFitCube(final List<Point3D_F64> cloud)
   {
      PointCloudShapeFinder shapeFinder = applyRansac(cloud);
      List<Shape> found = new ArrayList<Shape>(shapeFinder.getFound());
      filter(found, .25, 4);

      List<PlaneGeneral3D_F64> planes = new ArrayList<PlaneGeneral3D_F64>();
      for (Shape s : found)
         planes.add((PlaneGeneral3D_F64) s.parameters);

      List<Point3D_F64> allPoints = new ArrayList<Point3D_F64>();
      for (Shape s : found)
         allPoints.addAll(s.points);
      
      ScoringFunction<PlaneGeneral3D_F64, Point3D_F64> scorer = ExpectationMaximizationFitter.getGaussianSqauresMixedError(.01 / 2);
      List<Shape> emFitShapes = getEMFitShapes(planes, cloud, scorer, 50, .9999);
      //List<Shape> emFitShapes = getEMFitShapes(planes, allPoints, scorer, 50, .9999);

      List<PlaneNormal3D_F64> normalPlanes = new ArrayList<PlaneNormal3D_F64>();
      for (Shape s : emFitShapes)
         normalPlanes.add(IhmcPointCloudOps.convert((PlaneGeneral3D_F64) s.parameters, s.points, null));

      List<PlaneNormal3D_F64> cubePlanes = new ArrayList<PlaneNormal3D_F64>();
      cubePlanes.add(selectMax(normalPlanes, new Vector3D_F64(0, -1, 0)));
      cubePlanes.add(selectMax(normalPlanes, new Vector3D_F64(0, 1, 0)));
      cubePlanes.add(selectMax(normalPlanes, new Vector3D_F64(0, 0, 1)));

      IhmcPointCloudOps.adjustBoxNormals(cubePlanes.get(0), cubePlanes.get(1), cubePlanes.get(1));

      return cubePlanes;
   }
   

   
   public static PlaneNormal3D_F64 selectMax(List<PlaneNormal3D_F64> planes, Vector3D_F64 direction)
   {
      double max = Double.NEGATIVE_INFINITY;
      double dot = 0;
      PlaneNormal3D_F64 maxPlane = null;
      for (PlaneNormal3D_F64 p : planes)
      {
         if ((dot = direction.dot(new Vector3D_F64(p.p.x, p.p.y, p.p.z))) > max)
         {
            maxPlane = p;
            max = dot;
         }
      }

      return maxPlane;
   }
   


   private static List<Shape> getEMFitShapes(List<PlaneGeneral3D_F64> startPlanes, List<Point3D_F64> cloud,
         ScoringFunction<PlaneGeneral3D_F64, Point3D_F64> scorer, int rounds, double filter)
   {
      List<PlaneGeneral3D_F64> planes = ExpectationMaximizationFitter.fit(startPlanes, cloud, scorer, rounds);
      double[][] weights = ExpectationMaximizationFitter.getWeights(null, planes, cloud, scorer);

      List<Shape> emFitShapes = new ArrayList<Shape>();

      for (int i = 0; i < planes.size(); i++)
      {
         Shape s = new Shape();
         s.type = CloudShapeTypes.PLANE;
         s.points = new ArrayList<Point3D_F64>();
         s.parameters = planes.get(i);
         emFitShapes.add(s);
         for (int j = 0; j < cloud.size(); j++)
         {
            if (weights[i][j] > filter)
               s.points.add(cloud.get(j));
         }
      }

      return emFitShapes;
   }

   private static void filter(List<Shape> shapes, double alpha, double max)
   {
      Collections.sort(shapes, new Comparator<Shape>()
      {
         public int compare(Shape o1, Shape o2)
         {
            return o1.points.size() - o2.points.size();
         }
      });

      double eps = 3e-2;
      for (int i = 0; i < shapes.size(); i++)
      {
         List<Point3D_F64> s1 = shapes.get(i).points;
         for (int j = i + 1; j < shapes.size(); j++)
         {
            List<Point3D_F64> s2 = shapes.get(j).points;
            int overlap = 0;
            for (int k = 0; k < s2.size(); k++)
            {
               for (int l = 0; l < s1.size(); l++)
               {
                  if (s1.get(l).distance(s2.get(k)) < eps)
                     overlap++;
               }
            }

            System.out.println("overlap: " + overlap + " of " + s2.size() + " with limit:" + s2.size() * alpha);
            if (overlap > s2.size() * alpha)
            {
               shapes.remove(j);
               j--;
            }
         }

         while (shapes.size() > max)
         {
            shapes.remove(0);
         }
      }
   }

   private static PointCloudShapeFinder applyRansac(List<Point3D_F64> cloud)
   {
      CloudShapeTypes shapeTypes[] = new CloudShapeTypes[] { CloudShapeTypes.PLANE };//, CloudShapeTypes.CYLINDER, CloudShapeTypes.SPHERE};

      ConfigSchnabel2007 configRansac = ConfigSchnabel2007.createDefault(20, 0.8, 0.02, shapeTypes);
      configRansac.randomSeed = rand.nextLong();

      configRansac.minModelAccept = 50;
      configRansac.octreeSplit = 25;

      configRansac.maximumAllowedIterations = 1000;
      configRansac.ransacExtension = 25;

      ConfigRemoveFalseShapes configMerge = new ConfigRemoveFalseShapes(0.7);

      PointCloudShapeFinder shapeFinder = FactoryPointCloudShape.ransacOctree(configNormal, configRansac, configMerge);

      shapeFinder.process(cloud, null);
      return shapeFinder;
   }

}
