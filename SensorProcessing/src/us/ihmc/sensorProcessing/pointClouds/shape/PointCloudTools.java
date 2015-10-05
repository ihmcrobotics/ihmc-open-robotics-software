package us.ihmc.sensorProcessing.pointClouds.shape;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import javax.vecmath.Vector3d;

import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.UnconstrainedLeastSquares;
import org.ddogleg.optimization.UtilOptimize;
import org.ddogleg.optimization.functions.FunctionNtoM;
import org.ddogleg.struct.FastQueue;

import com.jme3.math.Vector3f;

import bubo.clouds.detect.CloudShapeTypes;
import bubo.clouds.detect.PointCloudShapeFinder.Shape;
import bubo.clouds.detect.alg.ApproximateSurfaceNormals;
import bubo.clouds.detect.alg.ConfigSchnabel2007;
import bubo.clouds.detect.alg.FoundShape;
import bubo.clouds.detect.alg.PointCloudShapeDetectionSchnabel2007;
import bubo.clouds.detect.alg.PointVectorNN;
import bubo.clouds.detect.wrapper.ConfigSurfaceNormals;
import bubo.io.serialization.DataDefinition;
import bubo.io.serialization.SerializationDefinitionManager;
import bubo.io.text.ReadCsvObjectSmart;
import georegression.geometry.UtilPlane3D_F64;
import georegression.geometry.UtilPoint3D_F64;
import georegression.metric.Distance3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.shapes.Box3D_F64;
import georegression.transform.se.SePointOps_F64;
import us.ihmc.sensorProcessing.pointClouds.shape.ExpectationMaximizationFitter.ScoringFunction;

public class PointCloudTools
{

   public static void transformCloud(List<Point3D_F64> cloud, Se3_F64 transform)
   {
      for (Point3D_F64 p : cloud)
         SePointOps_F64.transformReverse(transform, p, p);
   }

   public static List<Point3D_F64> readPointCloud(String fileName, int maxLines)
   {
      List<Point3D_F64> cloud = new ArrayList<Point3D_F64>();
      SerializationDefinitionManager manager = new SerializationDefinitionManager();
      manager.addDefinition(new DataDefinition("point3d", Point3D_F64.class, "x", "y", "z"));

      try
      {
         InputStream in = PointCloudTools.class.getClassLoader().getResourceAsStream(fileName);
         ReadCsvObjectSmart<Point3D_F64> reader = new ReadCsvObjectSmart<Point3D_F64>(in, manager, "point3d");

         Point3D_F64 pt = new Point3D_F64();
         int count = 0;
         while ((reader.nextObject(pt) != null) && (maxLines <= 0 || count++ < maxLines))
         {
            cloud.add(pt.copy());
         }

      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      return cloud;
   }

   public static List<Point3D_F64> boundSphere(List<Point3D_F64> fullCloud, Point3D_F64 center, double radius)
   {
      List<Point3D_F64> cloud = new ArrayList<Point3D_F64>();

      for (Point3D_F64 pt : fullCloud)
      {
         if (pt.distance(center) < radius)
            cloud.add(pt.copy());
      }
      return cloud;
   }

   public static FastQueue<PointVectorNN> boundSphereNormals(FastQueue<PointVectorNN> fullCloud, Point3D_F64 center, double radius)
   {
      FastQueue<PointVectorNN> result = new FastQueue<PointVectorNN>(PointVectorNN.class, false);
      for (int i = 0; i < fullCloud.size(); i++)
      {
         PointVectorNN pt = fullCloud.get(i);
         if (pt.p.distance(center) < radius)
            result.add(pt);
      }

      return result;
   }

   public static Point3D_F64 getCoM(List<Point3D_F64> points)
   {
      Point3D_F64 com = new Point3D_F64();
      for (Point3D_F64 p : points)
      {
         com = com.plus(p);
      }

      com.scale(1.0 / points.size());
      return com;
   }

   public static List<Point3D_F64> boundCube(List<Point3D_F64> fullCloud, Point3D_F64 center, double size)
   {
      Vector3d max = new Vector3d(center.x + size, center.y + size, center.z + size);
      Vector3d min = new Vector3d(center.x - size, center.y - size, center.z - size);

      List<Point3D_F64> cloud = new ArrayList<Point3D_F64>();
      SerializationDefinitionManager manager = new SerializationDefinitionManager();
      manager.addDefinition(new DataDefinition("point3d", Point3D_F64.class, "x", "y", "z"));

      for (Point3D_F64 pt : fullCloud)
      {
         if ((pt.x >= min.x) && (pt.y >= min.y) && (pt.z >= min.z) && (pt.x <= max.x) && (pt.y <= max.y) && (pt.z <= max.z))
         {
            cloud.add(pt);
         }
      }
      return cloud;
   }

   public static Point3D_F64 vecToPoint(Vector3f v)
   {
      return new Point3D_F64(v.x, v.y, v.z);
   }

   public static Vector3f pointToVec(Point3D_F64 p)
   {
      return new Vector3f((float) p.x, (float) p.y, (float) p.z);
   }

   public static List<Point3D_F64> thinCloud(List<Point3D_F64> cloud, double size)
   {
      ApproximateSurfaceNormals surface = new ApproximateSurfaceNormals(1000, size);
      FastQueue<PointVectorNN> pointNormList = new FastQueue<PointVectorNN>(PointVectorNN.class, false);
      surface.process(cloud, pointNormList);

      List<Point3D_F64> result = new ArrayList<Point3D_F64>();
      HashSet<Point3D_F64> seen = new HashSet<Point3D_F64>();
      Point3D_F64 avg = new Point3D_F64();
      int count;
      for (PointVectorNN p : pointNormList.toList())
      {
         if (!seen.add(p.p))
            continue;
         avg = p.p.copy();
         count = 1;
         for (PointVectorNN n : p.neighbors.toList())
         {
            if (!seen.add(n.p))
               continue;
            avg = avg.plus(n.p);
            count++;
         }

         avg.scale(1.0 / count);
         result.add(avg.copy());
      }

      return result;
   }

   public static List<Point3D_F64> filterByResidual(List<Point3D_F64> cloud, ConfigSurfaceNormals configNormal, double thresh)
   {
      ApproximateSurfaceNormals surface = new ApproximateSurfaceNormals( configNormal.numNeighbors,
            configNormal.maxDistanceNeighbor);
      FastQueue<PointVectorNN> pointNormList = new FastQueue<PointVectorNN>(PointVectorNN.class, false);
      surface.process(cloud, pointNormList);

      List<Point3D_F64> result = new ArrayList<Point3D_F64>();
      for (PointVectorNN p : pointNormList.toList())
      {
         if (getAverageError(p) < thresh)
         {
            result.add(p.p);
         }
      }

      return result;
   }

   public static List<Point3D_F64> filterByNormalOrientation(List<Point3D_F64> cloud, ConfigSurfaceNormals configNormal, Vector3D_F64 desiredNormal,
         double thresh)
   {
      ApproximateSurfaceNormals surface = new ApproximateSurfaceNormals(configNormal.numNeighbors,
            configNormal.maxDistanceNeighbor);
      FastQueue<PointVectorNN> pointNormList = new FastQueue<PointVectorNN>(PointVectorNN.class, false);
      surface.process(cloud, pointNormList);

      desiredNormal.normalize();
      List<Point3D_F64> result = new ArrayList<Point3D_F64>();
      for (PointVectorNN p : pointNormList.toList())
      {
         double angle = Math.acos(Math.abs(p.normal.dot(desiredNormal) / (p.normal.norm())));
         if (Math.abs(angle) < thresh)
         {
            result.add(p.p);
         }
      }

      return result;
   }

   public static void growPlane(Shape s, List<Point3D_F64> cloud, ConfigSurfaceNormals configNormal, double distTresh, double angleThresh)
   {
      PlaneGeneral3D_F64 plane = (PlaneGeneral3D_F64) s.parameters;
      //PlaneNormal3D_F64 planeNorm = UtilPlane3D_F64.convert();

      ApproximateSurfaceNormals surface = new ApproximateSurfaceNormals( configNormal.numNeighbors,
            configNormal.maxDistanceNeighbor);
      FastQueue<PointVectorNN> pointNormList = new FastQueue<PointVectorNN>(PointVectorNN.class, false);
      surface.process(cloud, pointNormList);

      List<Point3D_F64> newPoints = new ArrayList<Point3D_F64>();
      for (PointVectorNN p : pointNormList.toList())
      {
         //if ()
      }
   }

   public static List<Shape> process(List<Point3D_F64> cloud, ConfigSurfaceNormals configNormal, ConfigSchnabel2007 configRansac, Point3D_F64 center,
         double radius)
   {
      ApproximateSurfaceNormals surface = new ApproximateSurfaceNormals(configNormal.numNeighbors,
            configNormal.maxDistanceNeighbor);
      FastQueue<PointVectorNN> pointNormList = new FastQueue<PointVectorNN>(PointVectorNN.class, false);
      surface.process(cloud, pointNormList);

      //System.out.println("Point cloud size: " + pointNormList.size());
      pointNormList = boundSphereNormals(pointNormList, center, radius);
      //System.out.println("After bounding: " + pointNormList.size());

      //System.out.println("about to process...");
      PointCloudShapeDetectionSchnabel2007 alg = new PointCloudShapeDetectionSchnabel2007(configRansac);

      Box3D_F64 boundingBox = new Box3D_F64();
      UtilPoint3D_F64.boundingBox(cloud, boundingBox);

      alg.process(pointNormList, boundingBox);
      //System.out.println("done...");

      return convertIntoOuput(alg.getFoundObjects().toList());
   }

   private static ArrayList<Shape> convertIntoOuput(List<FoundShape> schnabelShapes)
   {
      List<CloudShapeTypes> shapeList = new ArrayList<CloudShapeTypes>();
      shapeList.add(CloudShapeTypes.PLANE);
      FastQueue<Shape> output = new FastQueue<Shape>(Shape.class, true);
      output.reset();

      for (int i = 0; i < schnabelShapes.size(); i++)
      {
         FoundShape fs = schnabelShapes.get(i);
         Shape os = output.grow();
         os.parameters = fs.modelParam;
         os.type = shapeList.get(fs.whichShape);
         os.points.clear();
         os.indexes.reset();

         // add the points to it
         for (int j = 0; j < fs.points.size(); j++)
         {
            PointVectorNN pv = fs.points.get(j);
            os.points.add(pv.p);
            os.indexes.add(pv.index);
         }
      }

      return new ArrayList<Shape>(output.toList());
   }

   public static void nonLinearFitNormal(final PointVectorNN point)
   {
      // Define the function being optimized and create the optimizer
      final int size = point.neighbors.size();
      final PointVectorNN[] neighbors = point.neighbors.data;
      final Vector3D_F64[] diffs = new Vector3D_F64[neighbors.length];
      for (int i = 0; i < size; i++)
         diffs[i] = new Vector3D_F64(point.p, neighbors[i].p);

      FunctionNtoM func = new FunctionNtoM()
      {
         double TAU_MAX = .0000002; // maximum projection distance
         double LAMBDA = 00.0; // discount rate -> computed as ln(.5)/.05
         double[] n = new double[3];
         double dot;
         Vector3D_F64 a;

         public void process(double[] in, double[] out)
         {
            for (int i = 0; i < size; i++)
            {
               phiThetaToNormal(in[0], in[1], n);

               a = diffs[i];
               dot = (a.x * n[0]) + (a.y * n[1]) + (a.z * n[2]);// - in[2];

               out[i] = Math.pow(Math.E, -LAMBDA * (dot * dot)) * dot;

            }
            //out[point.neighbors.size] = Math.log((TAU_MAX - in[2]) / TAU_MAX);
            //out[point.neighbors.size + 1] = Math.log((TAU_MAX + in[2]) / TAU_MAX);
         }

         public int getNumOfInputsN()
         {
            // phi, theta, tau
            return 2;
         }

         public int getNumOfOutputsM()
         {
            // two log boundary functions
            return neighbors.length + 2;
         }
      };

      UnconstrainedLeastSquares optimizer = FactoryOptimization.leastSquaresLM(1e-3, true);

      // if no jacobian is specified it will be computed numerically
      optimizer.setFunction(func, null);

      // provide it an extremely crude initial estimate of the line equation
      double phi = Math.asin(point.normal.z);
      double theta = Math.asin(point.normal.y / Math.cos(phi));
      optimizer.initialize(new double[] { phi, theta, 0 }, 1e-12, 1e-12);
      //optimizer.initialize(new double[] { 0.2, 0.2}, 1e-12, 1e-12);

      // iterate 500 times or until it converges.
      // Manually iteration is possible too if more control over is required
      UtilOptimize.process(optimizer, 500);

      double found[] = optimizer.getParameters();

      double[] n = new double[3];
      //phiThetaToNormal(phi, theta, n);
      phiThetaToNormal(found[0], found[1], n);
      point.normal = new Vector3D_F64(n[0], n[1], n[2]);
   }

   public static void weightedLinearFit(final PointVectorNN point, ScoringFunction<PlaneNormal3D_F64, Point3D_F64> scorer, int iterations)
   {
      // Define the function being optimized and create the optimizer
      final int size = point.neighbors.size();
      final PointVectorNN[] neighbors = point.neighbors.data;

      List<Point3D_F64> diffs = new ArrayList<Point3D_F64>();
      for (int i = 0; i < size; i++)
      {
         Vector3D_F64 v = new Vector3D_F64(point.p, neighbors[i].p);
         diffs.add(i, new Point3D_F64(v.x, v.y, v.z));
      }

      FitPlaneWeighted3D_F64 fitter = new FitPlaneWeighted3D_F64();
      Point3D_F64 fitOrigin = new Point3D_F64();
      Vector3D_F64 fitNormal = new Vector3D_F64();

      PlaneNormal3D_F64 plane = new PlaneNormal3D_F64(point.p, point.normal);

      double[] weights = new double[size];

      for (; iterations > 0; iterations--)
      {
         getWeights(weights, plane, diffs, scorer);
         fitter.svd(diffs, weights, fitOrigin, fitNormal);
         plane = new PlaneNormal3D_F64(fitOrigin, fitNormal);
      }

      point.normal = fitNormal;
   }

   public static double[] getWeights(double[] weights, PlaneNormal3D_F64 plane, List<Point3D_F64> allPoints,
         ScoringFunction<PlaneNormal3D_F64, Point3D_F64> scorer)
   {
      if (weights == null)
         weights = new double[allPoints.size()];

      for (int i = 0; i < allPoints.size(); i++)
      {
         weights[i] = scorer.score(plane, allPoints.get(i));
      }

      return weights;
   }

   public static void phiThetaToNormal(double phi, double theta, double[] n)
   {
      n[0] = Math.cos(phi) * Math.cos(theta);
      n[1] = Math.cos(phi) * Math.sin(theta);
      n[2] = Math.sin(phi);
   }

   public static double getAverageError(PointVectorNN p)
   {
      PlaneNormal3D_F64 normPlane = new PlaneNormal3D_F64(p.p, p.normal);
      PlaneGeneral3D_F64 genPlane = UtilPlane3D_F64.convert(normPlane, null);

      double averageError = 0;
      for (PointVectorNN n : p.neighbors.toList())
      {
         averageError += Math.abs(Distance3D_F64.distance(genPlane, n.p));
      }

      averageError /= p.neighbors.size();
      return averageError;
   }

   public static double score(PointVectorNN p)
   {
      PlaneNormal3D_F64 normPlane = new PlaneNormal3D_F64(p.p, p.normal);
      PlaneGeneral3D_F64 genPlane = UtilPlane3D_F64.convert(normPlane, null);

      double averageError = getAverageError(p);

      double above = 0;
      double below = 0;
      for (PointVectorNN n : p.neighbors.toList())
      {
         double dist = Distance3D_F64.distance(genPlane, n.p);
         if (Math.abs(dist) < averageError / 4)
            continue;

         if (dist > 0)
            above += dist;
         else
            below -= dist;
      }

      return (above * below) / Math.pow(p.neighbors.size(), 2);
   }

   public static double filter(PointVectorNN p, double norm, double alpha)
   {

      if (score(p) / norm > alpha)
         return 0;

      //boolean[] neighbors = new boolean[p.neighbors.size];
      double badNeighbors = 0;
      for (int i = 0; i < p.neighbors.size(); i++)
      {
         //neighbors[i] = score(p.neighbors.get(i)) > alpha;
         if (score(p.neighbors.get(i)) / norm > alpha)
            badNeighbors++;
      }

      return badNeighbors / p.neighbors.size();
   }

}
