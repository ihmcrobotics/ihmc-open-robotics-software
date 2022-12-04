package us.ihmc.perception.mapping;

import us.ihmc.bytedeco.slamWrapper.SlamWrapper;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

public class PlanarRegionFilteredMap
{
   private PlanarRegionFilteredMapParameters parameters;
   private SlamWrapper.FactorGraphExternal factorGraph;

   private static final double updateAlphaTowardsMatch = 0.05;

   private static final double angleThresholdBetweenNormalsForMatch = Math.toRadians(15);
   private static final float outOfPlaneDistanceFromOneRegionToAnother = 0.05f;
   private static final float maxDistanceBetweenRegionsForMatch = 0.0f;

   private boolean initialized = false;
   private boolean modified = false;
   private int uniqueRegionsFound = 0;
   private int uniqueIDtracker = 0;

   private final HashSet<Integer> mapRegionIDSet = new HashSet<>();

   private PlanarRegionsList finalMap;

   private int currentTimeIndex = 0;

   public PlanarRegionFilteredMap()
   {
      factorGraph = new SlamWrapper.FactorGraphExternal();
      parameters = new PlanarRegionFilteredMapParameters();
      finalMap = new PlanarRegionsList();

      factorGraph.addPriorPoseFactor(1, new float[]{0,0,0,0,0,0});
   }

   public void submitRegionsUsingIterativeReduction(PlanarRegionsList regions)
   {
      LogTools.info("-------------------------------------------------------- New Iteration --------------------------------------------------------------");
      modified = true;
      if (!initialized)
      {
         regions.getPlanarRegionsAsList().forEach(region ->
                                                  {
                                                     region.setRegionId(uniqueIDtracker++);
                                                     mapRegionIDSet.add(region.getRegionId());
                                                  });
         finalMap.addPlanarRegionsList(regions);

         initialized = true;
      }
      else
      {
         //// merge map regions within themselves
         //LogTools.info("Before Self First: {}", finalMap.getNumberOfPlanarRegions());
         //finalMap = selfReduceRegionsIteratively(finalMap,
         //                                        (float) updateAlphaTowardsMatch,
         //                                        (float) Math.cos(angleThresholdBetweenNormalsForMatch),
         //                                        outOfPlaneDistanceFromOneRegionToAnother,
         //                                        maxDistanceBetweenRegionsForMatch);

         LogTools.info("Incoming: {}", regions.getPlanarRegionsAsList().size());
         LogTools.info("Before Cross: {}", finalMap.getNumberOfPlanarRegions());
         // merge all the new regions in
         finalMap = crossReduceRegionsIteratively(finalMap,
                                                  regions,
                                                  (float) updateAlphaTowardsMatch,
                                                  (float) Math.cos(angleThresholdBetweenNormalsForMatch),
                                                  outOfPlaneDistanceFromOneRegionToAnother,
                                                  maxDistanceBetweenRegionsForMatch);

         LogTools.info("After Cross: {}", finalMap.getNumberOfPlanarRegions());
         // merge map regions within themselves again
         finalMap = selfReduceRegionsIteratively(finalMap,
                                                 (float) updateAlphaTowardsMatch,
                                                 (float) Math.cos(angleThresholdBetweenNormalsForMatch),
                                                 outOfPlaneDistanceFromOneRegionToAnother,
                                                 maxDistanceBetweenRegionsForMatch);

         LogTools.info("After Final Self: {}", finalMap.getPlanarRegionsAsList().size());

         //LogTools.info("Unique Set: [{}]", mapRegionIDSet);
      }

      currentTimeIndex++;
      LogTools.info("-------------------------------------------------------- Done --------------------------------------------------------------\n");
   }

   public void submitRegionsUsingGraphicalReduction(PlanarRegionsList regions)
   {
      modified = true;
      if (!initialized)
      {
         regions.getPlanarRegionsAsList().forEach(region ->
                                                  {
                                                     region.setRegionId(uniqueIDtracker++);
                                                     mapRegionIDSet.add(region.getRegionId());
                                                  });
         finalMap.addPlanarRegionsList(regions);

         initialized = true;
      }
      else
      {
         // initialize the planar region graph
         PlanarRegionGraph planarRegionGraph = new PlanarRegionGraph();
         finalMap.getPlanarRegionsAsList().forEach(planarRegionGraph::addRootOfBranch);

         // assign unique ids to the incoming regions
         regions.getPlanarRegionsAsList().forEach(region -> region.setRegionId(uniqueIDtracker++));

         addIncomingRegionsToGraph(finalMap,
                                   regions,
                                   planarRegionGraph,
                                   (float) Math.cos(angleThresholdBetweenNormalsForMatch),
                                   outOfPlaneDistanceFromOneRegionToAnother,
                                   maxDistanceBetweenRegionsForMatch);

         //LogTools.info("Regions Before: {}", regions.getPlanarRegionsAsList().size());

         // merge all the new regions in
         planarRegionGraph.collapseGraphByMerging(updateAlphaTowardsMatch);

         // go back through the existing regions and add them to the graph to check for overlap
         checkMapRegionsForOverlap(planarRegionGraph,
                                   (float) Math.cos(angleThresholdBetweenNormalsForMatch),
                                   outOfPlaneDistanceFromOneRegionToAnother,
                                   maxDistanceBetweenRegionsForMatch);

         planarRegionGraph.collapseGraphByMerging(updateAlphaTowardsMatch);

         finalMap = planarRegionGraph.getAsPlanarRegionsList();

         LogTools.info("Regions: {}, Unique: {}, Map: {}",
                       regions.getPlanarRegionsAsList().size(),
                       uniqueRegionsFound,
                       finalMap.getPlanarRegionsAsList().size());

         LogTools.info("Unique Set: [{}]", mapRegionIDSet);
      }
   }

   public static void addIncomingRegionsToGraph(PlanarRegionsList map,
                                                PlanarRegionsList incoming,
                                                PlanarRegionGraph graphToUpdate,
                                                float normalThreshold,
                                                float normalDistanceThreshold,
                                                float distanceThreshold)
   {
      PlanarRegionTools planarRegionTools = new PlanarRegionTools();

      List<PlanarRegion> incomingRegions = incoming.getPlanarRegionsAsList();
      List<PlanarRegion> mapRegions = map.getPlanarRegionsAsList();

      for (int incomingRegionId = 0; incomingRegionId < incomingRegions.size(); incomingRegionId++)
      {
         PlanarRegion newRegion = incomingRegions.get(incomingRegionId);
         boolean foundMatch = false;

         for (int mapRegionIndex = 0; mapRegionIndex < mapRegions.size(); mapRegionIndex++)
         {
            PlanarRegion mapRegion = mapRegions.get(mapRegionIndex);

            //if (boxesIn3DIntersect(mapRegion, newRegion, mapRegion.getBoundingBox3dInWorld().getMaxZ() - mapRegion.getBoundingBox3dInWorld().getMinZ()))
            {
               Point3D newOrigin = new Point3D();
               newRegion.getOrigin(newOrigin);

               Point3D mapOrigin = new Point3D();
               mapRegion.getOrigin(mapOrigin);

               Vector3D originVec = new Vector3D();
               originVec.sub(newOrigin, mapOrigin);

               double normalDistance = Math.abs(originVec.dot(mapRegion.getNormal()));
               double normalSimilarity = newRegion.getNormal().dot(mapRegion.getNormal());

               double originDistance = originVec.norm();

               // check to make sure the angles are similar enough
               boolean wasMatched = normalSimilarity > normalThreshold;
               // check that the regions aren't too far out of plane with one another. TODO should check this normal distance measure. That's likely a problem
               wasMatched &= normalDistance < normalDistanceThreshold;
               // check that the regions aren't too far from one another
               if (wasMatched)
                  wasMatched = planarRegionTools.getDistanceBetweenPlanarRegions(mapRegion, newRegion) <= distanceThreshold;

               //LogTools.info(String.format("(%d): (%d -> %d) Metrics: (%.3f > %.3f), (%.3f < %.3f), (%.3f < %.3f)",
               //                            mapRegionIndex,
               //                            mapRegion.getRegionId(),
               //                            newRegion.getRegionId(),
               //                            normalSimilarity,
               //                            normalThreshold,
               //                            normalDistance,
               //                            normalDistanceThreshold,
               //                            originDistance,
               //                            distanceThreshold) + ": [{}]", wasMatched);

               if (wasMatched)
               {
                  graphToUpdate.addEdge(mapRegion, newRegion);
                  foundMatch = true;
               }
            }
         }

         if (!foundMatch)
            graphToUpdate.addRootOfBranch(newRegion);
      }
   }

   public static void checkMapRegionsForOverlap(PlanarRegionGraph graphToUpdate, float normalThreshold, float normalDistanceThreshold, float distanceThreshold)
   {
      List<PlanarRegion> mapRegions = graphToUpdate.getAsPlanarRegionsList().getPlanarRegionsAsList();

      for (int idA = 0; idA < mapRegions.size(); idA++)
      {
         PlanarRegion regionA = mapRegions.get(idA);

         for (int idB = idA + 1; idB < mapRegions.size(); idB++)
         {
            PlanarRegion regionB = mapRegions.get(idB);

            boolean wasMatched = PlanarRegionSLAMTools.checkRegionsForOverlap(regionA, regionB, normalThreshold, normalDistanceThreshold, distanceThreshold);

            if (wasMatched)
            {
               graphToUpdate.addEdge(regionB, regionA);
            }
         }
      }
   }

   public PlanarRegionsList selfReduceRegionsIteratively(PlanarRegionsList map,
                                                         float updateTowardsChildAlpha,
                                                         float normalThreshold,
                                                         float normalDistanceThreshold,
                                                         float distanceThreshold)
   {
      boolean changed = false;
      do
      {

         changed = false;

         int parentIndex = 0;
         int childIndex = 0;

         LogTools.info("Change Iteration: MapTotal({}) - Parent({}) - Child({})", map.getNumberOfPlanarRegions(), parentIndex, childIndex);

         while (parentIndex < map.getNumberOfPlanarRegions())
         {
            LogTools.info("Parent Iteration: MapTotal({}) - Parent({}) - Child({})", map.getNumberOfPlanarRegions(), parentIndex, childIndex);
            childIndex = 0;
            PlanarRegion parentRegion = map.getPlanarRegion(parentIndex);
            while (childIndex < map.getNumberOfPlanarRegions())
            {
               if (parentIndex == childIndex)
               {
                  childIndex++;
               }
               else
               {
                  PlanarRegion childRegion = map.getPlanarRegion(childIndex);

                  LogTools.info("Child Iteration: MapTotal({}) - Parent({}) - Child({})", map.getNumberOfPlanarRegions(), parentIndex, childIndex);

                  if (PlanarRegionSLAMTools.checkRegionsForOverlap(parentRegion, childRegion, normalThreshold, normalDistanceThreshold, distanceThreshold))
                  {
                     LogTools.info("Matched({},{}) -> Merging", parentIndex, childIndex);
                     if (PlanarRegionSLAMTools.mergeRegionIntoParent(parentRegion, childRegion, updateTowardsChildAlpha))
                     {
                        LogTools.info("Merged({},{}) -> Removing({})", parentIndex, childIndex, childIndex);

                        changed = true;
                        map.getPlanarRegionsAsList().remove(childIndex);
                     }
                     else
                     {
                        childIndex++;
                     }
                  }
                  else
                  {
                     childIndex++;
                  }

                  if (changed)
                     break;
               }
               if (changed)
                  break;
            }
            parentIndex++;
         }
      }
      while (changed);

      return map;
   }

   public PlanarRegionsList crossReduceRegionsIteratively(PlanarRegionsList map,
                                                          PlanarRegionsList regions,
                                                          float updateTowardsChildAlpha,
                                                          float normalThreshold,
                                                          float normalDistanceThreshold,
                                                          float distanceThreshold)
   {
      map.addPlanarRegionsList(regions);
      map = selfReduceRegionsIteratively(map,
                                         (float) updateAlphaTowardsMatch,
                                         (float) Math.cos(angleThresholdBetweenNormalsForMatch),
                                         outOfPlaneDistanceFromOneRegionToAnother,
                                         maxDistanceBetweenRegionsForMatch);
      return map;
   }

   public void applyFactorGraphBasedSmoothing(PlanarRegionsList map, PlanarRegionsList regions, HashMap<Integer, ArrayList<Integer>> matches)
   {
      Vector3D translation = new Vector3D();
      Point3D eulerAngles = new Point3D();

      factorGraph.addOdometryFactor(new float[]{translation.getX32(), translation.getY32(), translation.getZ32(), eulerAngles.getX32(), eulerAngles.getY32(), eulerAngles.getZ32()},2);

      // TODO: Convert to world frame and insert the initial value

      factorGraph.optimize();
   }

   public PlanarRegionsList getMapRegions()
   {
      return finalMap;
   }

   public boolean isModified()
   {
      return modified;
   }

   public void setModified(boolean modified)
   {
      this.modified = modified;
   }

   public PlanarRegionFilteredMapParameters getParameters()
   {
      return parameters;
   }
}

