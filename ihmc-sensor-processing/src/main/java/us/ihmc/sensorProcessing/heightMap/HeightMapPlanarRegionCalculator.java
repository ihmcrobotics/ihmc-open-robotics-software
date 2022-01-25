package us.ihmc.sensorProcessing.heightMap;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.IntFunction;

public class HeightMapPlanarRegionCalculator
{
   private static final double distanceEpsilon = 0.055;
   private static final double angularEpsilon = Math.toRadians(70.0);

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

   // TODO switch from plane expansion being based just on point being expanded to

   public void computeRegions(HeightMapData heightMapData, IntFunction<UnitVector3DBasics> surfaceNormals)
   {
      regionIds = new int[heightMapData.getCellsPerAxis() * heightMapData.getCellsPerAxis()];
      Arrays.fill(regionIds, -1);
      regions.clear();
      planes.clear();
      int numMatches = 0;

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
            int minUnassignedDistanceIndex = -1;
            int minAssignedDistanceIndex = -1;

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

      /* Make a pass to merge neighbor regions */
//      for (int xi = 0; xi < heightMapData.getCellsPerAxis(); xi++)
//      {
//         for (int yi = 0; yi < heightMapData.getCellsPerAxis(); yi++)
//         {
//            int key = HeightMapTools.indicesToKey(xi, yi, heightMapData.getCenterIndex());
//            int regionId = regionIds[key];
//            Plane3D plane = planes.get(regionId);
//
//            for (int i = 0; i < xSearchOffsets.length; i++)
//            {
//               int xNeighbor = xi + xSearchOffsets[i];
//               int yNeighbor = yi + ySearchOffsets[i];
//               int neighborKey = HeightMapTools.indicesToKey(xNeighbor, yNeighbor, heightMapData.getCenterIndex());
//
//               if (xNeighbor < 0 || xNeighbor >= heightMapData.getCellsPerAxis() || yNeighbor < 0 || yNeighbor >= heightMapData.getCellsPerAxis())
//               {
//                  continue;
//               }
//
//               int neighborRegionId = regionIds[neighborKey];
//               Plane3D neighborPlane = planes.get(neighborRegionId);
//
//               if (regionId == neighborRegionId)
//               {
//                  continue;
//               }
//
//               boolean normalsAreSimilar = Math.abs(plane.getNormal().angle(neighborPlane.getNormal())) > Math.toRadians(25.0);
//               boolean distanceIsClose = plane.distance(HeightMapTools.indexToCoordinate(yNeighbor, heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex()),
//                                                        HeightMapTools.indexToCoordinate(yNeighbor, heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex()),
//                                                        heightMapData.getHeightAt(xNeighbor, yNeighbor)) < 0.04;
//
//               if (normalsAreSimilar && distanceIsClose)
//               {
//                  // merge regions
//                  mergeRegions(regionId, neighborRegionId, surfaceNormals, heightMapData);
//                  break;
//               }
//            }
//         }
//      }

      System.out.println("Num regions: " + regions.size());
      System.out.println("Num matches: " + numMatches);
   }

   private void getPlaneEstimate(int key, IntFunction<UnitVector3DBasics> surfaceNormals, HeightMapData heightMapData)
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

   public void updatePlaneEstimate(int regionId, IntFunction<UnitVector3DBasics> surfaceNormals, HeightMapData heightMapData)
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

   private void mergeRegions(int regionId0, int regionId1, IntFunction<UnitVector3DBasics> surfaceNormals, HeightMapData heightMapData)
   {
      TIntArrayList region0 = regions.get(regionId0);
      TIntArrayList region1 = regions.get(regionId1);

      region0.addAll(region1);

      for (int i = 0; i < region1.size(); i++)
      {
         regionIds[region1.get(i)] = regionId0;
      }

      planes.get(regionId1).setToNaN();
      updatePlaneEstimate(regionId0, surfaceNormals, heightMapData);
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
