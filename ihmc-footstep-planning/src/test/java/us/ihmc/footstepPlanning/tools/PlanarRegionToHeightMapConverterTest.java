package us.ihmc.footstepPlanning.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.euclid.tools.EuclidCoreTestTools.assertEquals;

public class PlanarRegionToHeightMapConverterTest
{
   private static final int iters = 5000;

   @Test
   public void testFittingNormals()
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(2.0, 2.0);
      polygon.addVertex(2.0, -2.0);
      polygon.addVertex(-2.0, -2.0);
      polygon.addVertex(-2.0, 2.0);
      polygon.update();
      ArrayList<ConvexPolygon2D> polygons = new ArrayList<>();
      polygons.add(polygon);

      PlanarRegion planarRegion = new PlanarRegion();
      planarRegion.set(transformToWorld, polygons);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegion);

      // test random orientations
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         QuaternionReadOnly orientation3DReadOnly = EuclidCoreRandomTools.nextQuaternion(random);

         transformToWorld.setIdentity();
         transformToWorld.getRotation().set(orientation3DReadOnly);
         planarRegion.set(transformToWorld, polygons);

         // get the planar region as a height map
         HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList,
                                                                                                                                                PlanarRegionToHeightMapConverter.defaultResolution,
                                                                                                                                                Double.NaN));



         // get the desired normal from the input values
         Vector3D vertical = new Vector3D(0.0, 0.0, 1.0);
         Vector3D normal = new Vector3D(vertical);
         orientation3DReadOnly.transform(normal);

         // the normal was flipped, don't do that.
         if (normal.getZ() < 0.0)
            normal.negate();

         // fixme may be best to just fit Z?
         Vector3DReadOnly heightMapNormal = getNormalOfHeightMap(heightMapData);
         if (heightMapNormal == null)
            continue;
         assertEquals(normal, heightMapNormal, 1e-3);
      }
   }

   public static Vector3DReadOnly getNormalOfHeightMap(HeightMapData heightMapData)
   {
      // convert this height map to a point cloud, so that we can do a plane fit.
      List<Point3D> pointCloud = collectHeightMapAsPointCloud(heightMapData);
      if (pointCloud.size() < 3)
         return null;

      // fit a plane to the point cloud using least squares.
      LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();
      Plane3D bestPlane = new Plane3D();
      planeFitter.fitPlaneToPoints(pointCloud, bestPlane);

      if (bestPlane.containsNaN())
         return null;

      return bestPlane.getNormal();
   }

   private static List<Point3D> collectHeightMapAsPointCloud(HeightMapData heightMapData)
   {
      List<Point3D> pointCloud = new ArrayList<>();
      for (int xIndex = 0; xIndex < heightMapData.getCellsPerAxis(); xIndex++)
      {
         for (int yIndex = 0; yIndex < heightMapData.getCellsPerAxis(); yIndex++)
         {
            double x = HeightMapTools.indexToCoordinate(xIndex, heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
            double y = HeightMapTools.indexToCoordinate(yIndex, heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
            double height = heightMapData.getHeightAt(xIndex, yIndex);

            if (Double.isFinite(height))
               pointCloud.add(new Point3D(x,  y, height));
         }
      }

      return pointCloud;
   }
}
