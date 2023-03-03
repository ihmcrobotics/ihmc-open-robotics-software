package us.ihmc.sensorProcessing.heightMap;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.IntFunction;

public class HeightMapPlanarRegionCalculator
{
   private static final double distanceEpsilon0 = 0.055;
   private static final double angularEpsilon0 = Math.toRadians(70.0);

   private static final double distanceEpsilon1 = 0.065;
   private static final double angularEpsilon1 = Math.toRadians(75.0);

   private static final int minRegionSize = 10;
   private static final int noRegionId = -1;

   /* Maps from cell key to region id */
   private int[] regionIds;
   /* Maps from region id to list of cell keys */
   private final List<TIntArrayList> regions = new ArrayList<>();
   /* Maps from region id to plane */
   private final List<Plane3D> planes = new ArrayList<>();

   private static final int[] xSearchOffsets = new int[]{-1, -1, -1,  0,  1,  1,  1,  0}; // ,  2, -2,  0,  0};
   private static final int[] ySearchOffsets = new int[]{-1,  0,  1,  1,  1,  0, -1, -1}; // ,  0,  0, -2,  2};

   private final Plane3D planeEstimate = new Plane3D();
   private final LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

   public void computeRegions(HeightMapData heightMapData, IntFunction<UnitVector3DReadOnly> surfaceNormals)
   {
      regionIds = new int[heightMapData.getCellsPerAxis() * heightMapData.getCellsPerAxis()];
      Arrays.fill(regionIds, noRegionId);
      regions.clear();
      planes.clear();
      int numMatches = 0;

      /* First pass with tighter constraints */
      growRegions(heightMapData, surfaceNormals, distanceEpsilon0, angularEpsilon0);
      /* Second pass with looser constraints */
      growRegions(heightMapData, surfaceNormals, distanceEpsilon1, angularEpsilon1);
      /* Prune regions with insufficient cells */
      removeRegionsWithInsufficientSize();

      int numRegions = 0;
      for (int i = 0; i < regions.size(); i++)
      {
         if (!regions.get(i).isEmpty())
            numRegions++;
      }

      System.out.println("Num regions: " + numRegions);
      System.out.println("Num matches: " + numMatches);
   }

   private void growRegions(HeightMapData heightMapData, IntFunction<UnitVector3DReadOnly> surfaceNormals, double distanceEpsilon, double angularEpsilon)
   {
      for (int xi = 0; xi < heightMapData.getCellsPerAxis(); xi++)
      {
         for (int yi = 0; yi < heightMapData.getCellsPerAxis(); yi++)
         {
            int centerIndex = heightMapData.getCenterIndex();
            int key = HeightMapTools.indicesToKey(xi, yi, centerIndex);

            /* Don't calculate if ground plane */
            if (heightMapData.isCellAtGroundPlane(xi, yi))
            {
               continue;
            }
            /* Check if already assigned */
            if (regionIds[key] >= 0)
            {
               continue;
            }

            planeEstimate.getPoint()
                         .set(HeightMapTools.indexToCoordinate(xi, heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), centerIndex),
                              HeightMapTools.indexToCoordinate(yi, heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), centerIndex),
                              heightMapData.getHeightAt(xi, yi));
            planeEstimate.getNormal().set(surfaceNormals.apply(key));

            double minUnassignedDistance = Double.MAX_VALUE;
            double minAssignedDistance = Double.MAX_VALUE;
            int minUnassignedDistanceIndex = noRegionId;
            int minAssignedDistanceIndex = noRegionId;

            for (int i = 0; i < xSearchOffsets.length; i++)
            {
               int xNeighbor = xi + xSearchOffsets[i];
               int yNeighbor = yi + ySearchOffsets[i];
               int neighborKey = HeightMapTools.indicesToKey(xNeighbor, yNeighbor, centerIndex);

               if (xNeighbor < 0 || xNeighbor >= heightMapData.getCellsPerAxis() || yNeighbor < 0 || yNeighbor >= heightMapData.getCellsPerAxis())
               {
                  continue;
               }

               getPlaneEstimate(neighborKey, surfaceNormals, heightMapData);
               double euclideanDistance = planeEstimate.distance(HeightMapTools.indexToCoordinate(xi, heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), centerIndex),
                                                                 HeightMapTools.indexToCoordinate(yi, heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), centerIndex),
                                                                 heightMapData.getHeightAt(xi, yi));
               double angularDistance = Math.abs(planeEstimate.getNormal().angle(surfaceNormals.apply(key)));
               boolean neighborIsAssigned = regionIds[neighborKey] >= 0;
               int numNeighbors = neighborIsAssigned ? regions.get(regionIds[neighborKey]).size() : 0;

               double distance = euclideanDistance / distanceEpsilon + angularDistance / angularEpsilon + 0.5 * Math.max(0.0, 1.0 - numNeighbors / 7.0);
               boolean acceptableDistance = euclideanDistance < distanceEpsilon && angularDistance < angularEpsilon;

               if (acceptableDistance && neighborIsAssigned && distance < minAssignedDistance)
               {
                  minAssignedDistance = distance;
                  minAssignedDistanceIndex = i;
               }
               else if (acceptableDistance && !neighborIsAssigned && distance < minUnassignedDistance)
               {
                  minUnassignedDistance = distance;
                  minUnassignedDistanceIndex = i;
               }
            }

            /* Neighbor is assigned, use the neighbor's id */
            if (minAssignedDistanceIndex >= 0)
            {
               int xNeighbor = xi + xSearchOffsets[minAssignedDistanceIndex];
               int yNeighbor = yi + ySearchOffsets[minAssignedDistanceIndex];
               int neighborKey = HeightMapTools.indicesToKey(xNeighbor, yNeighbor, centerIndex);

               int id = regionIds[neighborKey];
               regionIds[key] = id;
               regions.get(id).add(key);
            }
            /* Neighbor is unassigned, give them both a new id */
            else if (minUnassignedDistanceIndex >= 0)
            {
               int xNeighbor = xi + xSearchOffsets[minUnassignedDistanceIndex];
               int yNeighbor = yi + ySearchOffsets[minUnassignedDistanceIndex];
               int neighborKey = HeightMapTools.indicesToKey(xNeighbor, yNeighbor, centerIndex);

               int id = regions.size();
               regionIds[key] = id;
               regionIds[neighborKey] = id;
               regions.add(new TIntArrayList());
               planes.add(new Plane3D());

               regions.get(id).add(key);
               regions.get(id).add(neighborKey);
            }
            /* No matches, give a unique region id */
            else
            {
               int id = regions.size();

               regionIds[key] = id;
               regions.add(new TIntArrayList());
               planes.add(new Plane3D());
               regions.get(id).add(key);
            }

            updatePlaneEstimate(regionIds[key], surfaceNormals, heightMapData);
         }
      }
   }

   private void getPlaneEstimate(int key, IntFunction<UnitVector3DReadOnly> surfaceNormals, HeightMapData heightMapData)
   {
      if (regionIds[key] < 0)
      {
         int xIndex = HeightMapTools.keyToXIndex(key, heightMapData.getCenterIndex());
         int yIndex = HeightMapTools.keyToYIndex(key, heightMapData.getCenterIndex());

         planeEstimate.getPoint()
                      .set(HeightMapTools.indexToCoordinate(xIndex, heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex()),
                           HeightMapTools.indexToCoordinate(yIndex, heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex()),
                           heightMapData.getHeightAt(xIndex, yIndex));
         planeEstimate.getNormal().set(surfaceNormals.apply(key));
      }
      else
      {
         planeEstimate.set(planes.get(regionIds[key]));
      }
   }

   private final RecyclingArrayList<Point3D> tempPoints = new RecyclingArrayList<>(Point3D::new);

   public void updatePlaneEstimate(int regionId, IntFunction<UnitVector3DReadOnly> surfaceNormals, HeightMapData heightMapData)
   {
      TIntArrayList region = regions.get(regionId);
      int numberOfPoints = region.size();

      Plane3D plane = planes.get(regionId);
      tempPoints.clear();

      for (int i = 0; i < numberOfPoints; i++)
      {
         int xIndex = HeightMapTools.keyToXIndex(region.get(i), heightMapData.getCenterIndex());
         int yIndex = HeightMapTools.keyToYIndex(region.get(i), heightMapData.getCenterIndex());
         tempPoints.add().set(HeightMapTools.indexToCoordinate(xIndex, heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex()),
                              HeightMapTools.indexToCoordinate(yIndex, heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex()),
                              heightMapData.getHeightAt(xIndex, yIndex));
      }

      if (numberOfPoints == 1)
      {
         plane.getPoint().set(tempPoints.get(0));
         plane.getNormal().set(surfaceNormals.apply(region.get(0)));
      }
      else if (numberOfPoints == 2)
      {
         plane.getPoint().interpolate(tempPoints.get(0), tempPoints.get(1), 0.5);
         plane.getNormal().interpolate(surfaceNormals.apply(region.get(0)), surfaceNormals.apply(region.get(1)), 0.5);
      }
      else if (numberOfPoints == 3)
      {
         plane.set(tempPoints.get(0), tempPoints.get(1), tempPoints.get(2));
         if (plane.getNormal().getZ() < 0.0)
         {
            plane.getNormal().setX(-plane.getNormalX());
            plane.getNormal().setY(-plane.getNormalY());
            plane.getNormal().setZ(-plane.getNormalZ());
         }
      }
      else
      {
         planeFitter.fitPlaneToPoints(tempPoints, plane);
      }
   }

   private void removeRegionsWithInsufficientSize()
   {
      for (int i = regions.size() - 1; i >= 0; i--)
      {
         TIntArrayList region = regions.get(i);
         if (region.size() < minRegionSize)
         {
            for (int j = 0; j < region.size(); j++)
            {
               regionIds[region.get(j)] = noRegionId;
            }

            region.clear();
         }
      }
   }

   public int getRegionId(int cellKey)
   {
      return regionIds[cellKey];
   }

   public int getNumberOfRegions()
   {
      return regions.size();
   }

   public static void main(String[] args)
   {
      UnitVector3DBasics v1 = new UnitVector3D(Axis3D.X);
      UnitVector3DBasics v2 = new UnitVector3D(Axis3D.Z);
      UnitVector3DBasics v3 = new UnitVector3D();

      v3.interpolate(v1, v2, 0.5);
      System.out.println(v3);
   }
}
