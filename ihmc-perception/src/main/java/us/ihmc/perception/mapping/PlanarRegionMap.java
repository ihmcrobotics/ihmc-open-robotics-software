package us.ihmc.perception.mapping;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.bytedeco.slamWrapper.SlamWrapper;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;

import java.util.*;

public class PlanarRegionMap
{
   public enum MergingMode
   {
      FILTERING, SMOOTHING
   }

   public enum MatchingMode
   {
      ITERATIVE, GRAPHICAL
   }

   int sensorPoseIndex = 0;

   private MergingMode merger;
   private MatchingMode matcher;

   private PlanarRegionMappingParameters parameters;
   private SlamWrapper.FactorGraphExternal factorGraph;

   private final RigidBodyTransform previousSensorToWorldFrameTransform = new RigidBodyTransform();
   private final RigidBodyTransform worldToSensorFrameTransform = new RigidBodyTransform();
   private final RigidBodyTransform worldToPreviousSensorFrameTransform = new RigidBodyTransform();
   private final RigidBodyTransform odoometry = new RigidBodyTransform();

   private static final double updateAlphaTowardsMatch = 0.05;

   private static final double angleThresholdBetweenNormalsForMatch = Math.toRadians(15);
   private static final float outOfPlaneDistanceFromOneRegionToAnother = 0.05f;
   private static final float maxDistanceBetweenRegionsForMatch = 0.0f;

   private boolean initialized = false;
   private boolean modified = false;
   private int uniqueRegionsFound = 0;
   private int uniqueIDtracker = 1;

   private final TIntArrayList mapIDs = new TIntArrayList();
   private final TIntArrayList incomingIDs = new TIntArrayList();

   private final HashMap<Integer, TIntArrayList> planarRegionMatches = new HashMap<>();
   private final HashSet<Integer> mapRegionIDSet = new HashSet<>();

   private PlanarRegionsList finalMap;

   private int currentTimeIndex = 0;

   public PlanarRegionMap(boolean useSmoothingMerger)
   {
      if (useSmoothingMerger)
      {
         this.merger = MergingMode.SMOOTHING;

         BytedecoTools.loadGTSAMNatives();
         factorGraph = new SlamWrapper.FactorGraphExternal();
      }
      else
      {
         this.merger = MergingMode.FILTERING;
      }

      parameters = new PlanarRegionMappingParameters();
      finalMap = new PlanarRegionsList();
   }

   public void reset()
   {
      initialized = false;
      finalMap.clear();
   }

   public void submitRegionsUsingIterativeReduction(PlanarRegionsListWithPose regionsWithPose)
   {
      PlanarRegionsList regions = regionsWithPose.getPlanarRegionsList();
      LogTools.info("-------------------------------------------------------- New Iteration --------------------------------------------------------------");
      LogTools.info("Incoming: {}", regions.getPlanarRegionsAsList().size());

      modified = true;

      // Assign unique IDs to all incoming regions
      for (PlanarRegion region : regions.getPlanarRegionsAsList())
      {
         LogTools.info("New Region ID: ({})", uniqueIDtracker);

         if (!initialized)
            region.setRegionId(uniqueIDtracker++);
         else
            region.setRegionId(-uniqueIDtracker++);
      }

      if (!initialized)
      {
         finalMap.addPlanarRegionsList(regions);
         if(merger == MergingMode.SMOOTHING)
         {
            initializeFactorGraphForSmoothing(finalMap);
         }
         initialized = true;
      }
      else
      {
         LogTools.info("Before Cross: {}", finalMap.getNumberOfPlanarRegions());

         PlanarRegionSLAMTools.printRegionIDs("Map", finalMap);
         PlanarRegionSLAMTools.printRegionIDs("Incoming", regions);

         // merge all the new regions in
         finalMap = crossReduceRegionsIteratively(finalMap,
                                                  regions,
                                                  (float) updateAlphaTowardsMatch,
                                                  (float) Math.cos(angleThresholdBetweenNormalsForMatch),
                                                  outOfPlaneDistanceFromOneRegionToAnother,
                                                  maxDistanceBetweenRegionsForMatch);

         processUniqueRegions(finalMap);
         PlanarRegionSLAMTools.printMatches("Cross", finalMap, regions, planarRegionMatches);

         if(merger == MergingMode.SMOOTHING)
         {
            applyFactorGraphBasedSmoothing(finalMap, regions, regionsWithPose.getSensorToWorldFrameTransform(), planarRegionMatches, sensorPoseIndex);
         }

         LogTools.info("After Cross: {}", finalMap.getNumberOfPlanarRegions());
      }

      sensorPoseIndex++;

      finalMap.getPlanarRegionsAsList().forEach(region -> mapIDs.add(region.getRegionId()));
      regions.getPlanarRegionsAsList().forEach(region -> incomingIDs.add(region.getRegionId()));

      LogTools.info("Map IDs: {}", Arrays.toString(mapIDs.toArray()));
      LogTools.info("Incoming IDs: {}", Arrays.toString(incomingIDs.toArray()));

      mapIDs.clear();
      incomingIDs.clear();

      currentTimeIndex++;
      LogTools.debug("-------------------------------------------------------- Done --------------------------------------------------------------\n");
   }

   public void submitRegionsUsingGraphicalReduction(PlanarRegionsListWithPose regionsWithPose)
   {
      PlanarRegionsList regions = regionsWithPose.getPlanarRegionsList();
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

         // merge all the new regions in
         planarRegionGraph.collapseGraphByMerging(updateAlphaTowardsMatch);

         // go back through the existing regions and add them to the graph to check for overlap
         checkMapRegionsForOverlap(planarRegionGraph,
                                   (float) Math.cos(angleThresholdBetweenNormalsForMatch),
                                   outOfPlaneDistanceFromOneRegionToAnother,
                                   maxDistanceBetweenRegionsForMatch);

         planarRegionGraph.collapseGraphByMerging(updateAlphaTowardsMatch);

         finalMap = planarRegionGraph.getAsPlanarRegionsList();

         LogTools.debug("Regions: {}, Unique: {}, Map: {}",
                        regions.getPlanarRegionsAsList().size(),
                        uniqueRegionsFound,
                        finalMap.getPlanarRegionsAsList().size());

         LogTools.debug("Unique Set: [{}]", mapRegionIDSet);
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

               //LogTools.debug(String.format("(%d): (%d -> %d) Metrics: (%.3f > %.3f), (%.3f < %.3f), (%.3f < %.3f)",
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
                                                         float distanceThreshold,
                                                         HashMap<Integer, TIntArrayList> matches)
   {
      matches.clear();
      boolean changed = false;
      do
      {
         changed = false;

         int parentIndex = 0;
         int childIndex = 0;

         //LogTools.info("Change Iteration: MapTotal({}) - Parent({}) - Child({})", map.getNumberOfPlanarRegions(), parentIndex, childIndex);

         while (parentIndex < map.getNumberOfPlanarRegions())
         {
            //LogTools.info("Parent Iteration: MapTotal({}) - Parent({}) - Child({})", map.getNumberOfPlanarRegions(), parentIndex, childIndex);
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
                  int parentId = map.getPlanarRegion(parentIndex).getRegionId();
                  int childId = map.getPlanarRegion(childIndex).getRegionId();

                  PlanarRegion childRegion = map.getPlanarRegion(childIndex);

                  //LogTools.info("Checking Match: Parent({}) - Child({})", parentId, childId);

                  if (PlanarRegionSLAMTools.checkRegionsForOverlap(parentRegion, childRegion, normalThreshold, normalDistanceThreshold, distanceThreshold))
                  {
                     //LogTools.info("Matched({},{}) -> Merging", parentIndex, childIndex);

                     if (!matches.containsKey(parentId))
                     {
                        matches.put(parentId, new TIntArrayList());
                     }

                     matches.get(parentId).add(childId);

                     if (PlanarRegionSLAMTools.mergeRegionIntoParent(parentRegion, childRegion, updateTowardsChildAlpha))
                     {
                        //LogTools.info("Merged({},{}) -> Removing({})", parentIndex, childIndex, childIndex);

                        int finalId = generatePostMergeId(parentRegion.getRegionId(), childRegion.getRegionId());
                        parentRegion.setRegionId(finalId);

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
                                         maxDistanceBetweenRegionsForMatch,
                                         planarRegionMatches);
      return map;
   }

   public void applyFactorGraphBasedSmoothing(PlanarRegionsList map,
                                              PlanarRegionsList regions,
                                              RigidBodyTransform transform,
                                              HashMap<Integer, TIntArrayList> matches,
                                              int poseIndex)
   {
      worldToSensorFrameTransform.setAndInvert(transform);

      worldToPreviousSensorFrameTransform.setAndInvert(previousSensorToWorldFrameTransform);

      odoometry.set(worldToPreviousSensorFrameTransform);
      odoometry.multiply(transform);

      // Convert odometry to float array. Add into factor graph using 16-double method for addOdometryFactor().
      float[] odometryValue = new float[16];
      odoometry.get(odometryValue);

      LogTools.info("Adding odometry factor: x{} -> x{}", poseIndex - 1, poseIndex);
      factorGraph.addOdometryFactorExtended(odometryValue, poseIndex);

      //Convert sensor pose to float array. Add 16-float method for setPoseInitialValue().
      float[] poseValue = new float[16];
      transform.get(poseValue);

      LogTools.info("Adding Pose Initial Value: {}", poseIndex);
      factorGraph.setPoseInitialValueExtended(poseIndex, poseValue);

      for (Integer parentId : matches.keySet())
      {
         for (Integer childId : matches.get(parentId).toArray())
         {
            int landmarkId = Math.abs(generatePostMergeId(parentId, childId));

            if(childId >= 0)
               continue;

            LogTools.info("ParentID: {}, Finding Child Region with ID: {}", parentId, childId);
            PlanarRegion childRegion = regions.getRegionWithId(childId);

            // Get origin and normal in sensor frame.
            Point3D childOrigin = new Point3D();
            Vector3D childNormal = new Vector3D();
            childRegion.getOrigin(childOrigin);
            childRegion.getNormal(childNormal);
            childOrigin.applyTransform(worldToSensorFrameTransform);
            childNormal.applyTransform(worldToSensorFrameTransform);

            // Get origin and normal in world frame
            Point3D childOriginInWorld = new Point3D();
            Vector3D childNormalInWorld = new Vector3D();
            childRegion.getOrigin(childOriginInWorld);
            childRegion.getNormal(childNormalInWorld);

            // Insert plane factor based on planar region normal and origin in sensor frame
            double localDot = childOrigin.dot(childNormal);
            float[] planeInSensorFrame = new float[] {childNormalInWorld.getX32(), childNormalInWorld.getY32(), childNormalInWorld.getZ32(), (float) -localDot};

            LogTools.info("Adding plane factor: x{} -> l{}", poseIndex, landmarkId);
            factorGraph.addOrientedPlaneFactor(planeInSensorFrame, landmarkId, poseIndex);

            // Set initial value for plane based on planar region normal and origin in world frame
            double worldDot = childOriginInWorld.dot(childNormalInWorld);
            float[] planeInWorldFrame = new float[] {childNormalInWorld.getX32(), childNormalInWorld.getY32(), childNormalInWorld.getZ32(), (float) -worldDot};

            LogTools.info("Setting plane value: {}", landmarkId);
            factorGraph.setOrientedPlaneInitialValue(landmarkId, planeInWorldFrame);
         }
      }

      LogTools.info("Solving factor graph");
      factorGraph.optimize();

      factorGraph.printResults();
   }

   public void initializeFactorGraphForSmoothing(PlanarRegionsList map)
   {
      factorGraph.addPriorPoseFactor(0, new float[] {0,0,0,0,0,0});
      factorGraph.setPoseInitialValue(0, new float[] {0,0,0,0,0,0});

      for (PlanarRegion region : map.getPlanarRegionsAsList())
      {
         // Get origin and normal in sensor frame.
         Point3D childOrigin = new Point3D();
         Vector3D childNormal = new Vector3D();
         region.getOrigin(childOrigin);
         region.getNormal(childNormal);

         // Insert plane factor based on planar region normal and origin in sensor frame
         double localDot = childOrigin.dot(childNormal);
         float[] plane = new float[] {childNormal.getX32(), childNormal.getY32(), childNormal.getZ32(), (float) -localDot};
         factorGraph.addOrientedPlaneFactor(plane, region.getRegionId(), 0);
         factorGraph.setOrientedPlaneInitialValue(region.getRegionId(), plane);
      }
   }

   private int generatePostMergeId(int parentIndex, int childIndex)
   {
      if (parentIndex > 0 && childIndex > 0)
         return Math.min(parentIndex, childIndex);
      else if (parentIndex > 0 && childIndex < 0)
         return parentIndex;
      else if (parentIndex < 0 && childIndex > 0)
         return childIndex;
      else if (parentIndex < 0 && childIndex < 0)
         return Math.max(parentIndex, childIndex);
      else
         return 0;
   }

   private void processUniqueRegions(PlanarRegionsList map)
   {
      for (PlanarRegion region : map.getPlanarRegionsAsList())
      {
         region.setRegionId(Math.abs(region.getRegionId()));
      }
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

   public PlanarRegionMappingParameters getParameters()
   {
      return parameters;
   }
}

