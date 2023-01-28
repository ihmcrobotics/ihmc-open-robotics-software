package us.ihmc.perception.mapping;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.slamWrapper.FactorGraph;
import us.ihmc.perception.slamWrapper.SlamWrapper;
import us.ihmc.perception.tools.PerceptionPrintTools;
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
   private final RigidBodyTransform odometry = new RigidBodyTransform();

   private boolean initialized = false;
   private boolean modified = false;
   private int uniqueRegionsFound = 0;
   private int uniqueIDtracker = 1;

   private final TIntArrayList mapIDs = new TIntArrayList();
   private final TIntArrayList incomingIDs = new TIntArrayList();

   private final HashMap<Integer, TIntArrayList> allPlanarRegionMatches = new HashMap<>();
   private final HashMap<Integer, Integer> bestPlanarRegionMatches = new HashMap<>();

   private final HashSet<Integer> mapRegionIDSet = new HashSet<>();

   private PlanarRegionsList finalMap;

   private int currentTimeIndex = 0;

   public PlanarRegionMap(boolean useSmoothingMerger)
   {
      this(useSmoothingMerger, "");
   }

   public PlanarRegionMap(boolean useSmoothingMerger, String version)
   {
      if (useSmoothingMerger)
      {
         this.merger = MergingMode.SMOOTHING;

         BytedecoTools.loadSlamWrapper();
         factorGraph = new SlamWrapper.FactorGraphExternal();
      }
      else
      {
         this.merger = MergingMode.FILTERING;
      }

      parameters = new PlanarRegionMappingParameters(version);
      finalMap = new PlanarRegionsList();
   }

   public void reset()
   {
      initialized = false;
      finalMap.clear();
   }

   public void submitRegionsUsingIterativeReduction(PlanarRegionsListWithPose regionsWithPose)
   {
      PlanarRegionsList regionsRaw = regionsWithPose.getPlanarRegionsList();
      PlanarRegionsList regions = new PlanarRegionsList();

      if(regionsRaw.getNumberOfPlanarRegions() == 0)
      {
         LogTools.warn("Number Of Regions is 0!");
         return;
      }


      // Filters out regions that have area less than threshold
      for(PlanarRegion region : regionsRaw.getPlanarRegionsAsList())
      {
         if(region.getArea() > parameters.getMinimumPlanarRegionArea())
         {
            regions.addPlanarRegion(region);
         }
      }

      // TODO: Remove this: This is a hack to get around the fact that the sensor pose is not being updated correctly
      regions.applyTransform(regionsWithPose.getSensorToWorldFrameTransform());

      LogTools.info("-------------------------------------------------------- New Iteration --------------------------------------------------------------");
      LogTools.info("Incoming: {}", regions.getPlanarRegionsAsList().size());

      modified = true;

      // Assign unique IDs to all incoming regions
      for (PlanarRegion region : regions.getPlanarRegionsAsList())
      {
         if (!initialized)
            region.setRegionId(uniqueIDtracker++);
         else
            region.setRegionId(-uniqueIDtracker++);
      }

      if (!initialized)
      {
         finalMap.addPlanarRegionsList(regions);
         if (merger == MergingMode.SMOOTHING)
         {
            initializeFactorGraphForSmoothing(finalMap, regionsWithPose.getSensorToWorldFrameTransform());
         }
         initialized = true;
      }
      else
      {
         PerceptionPrintTools.printRegionIDs("Map", finalMap);
         PerceptionPrintTools.printRegionIDs("Incoming", regions);

         PlanarRegionSLAMTools.findPlanarRegionMatches(finalMap, regions, allPlanarRegionMatches, (float) parameters.getMinimumOverlapThreshold(),
                                                       (float) parameters.getSimilarityThresholdBetweenNormals(),
                                                       (float) parameters.getOrthogonalDistanceThreshold());

         if (merger == MergingMode.SMOOTHING)
         {
            // Find the optimal transform for incoming regions before merge
            applyFactorGraphBasedSmoothing(finalMap, regions, regionsWithPose.getSensorToWorldFrameTransform(), allPlanarRegionMatches, sensorPoseIndex);

            // Extract the optimal transform from the factor graph
            double[] transformArray = new double[16];
            factorGraph.getPoseById(sensorPoseIndex, transformArray);
            RigidBodyTransform optimizedSensorTransform = new RigidBodyTransform(transformArray);

            // Transform the incoming regions to the map frame with the optimal transform
            regions.applyTransform(optimizedSensorTransform);
         }

         // merge all the new regions in
         finalMap = crossReduceRegionsIteratively(finalMap, regions, allPlanarRegionMatches);

         processUniqueRegions(finalMap);
         PerceptionPrintTools.printMatches("Cross Matches", finalMap, regions, allPlanarRegionMatches);

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
                                   (float) parameters.getSimilarityThresholdBetweenNormals(),
                                   (float) parameters.getOrthogonalDistanceThreshold(),
                                   (float) parameters.getMaxInterRegionDistance());

         // merge all the new regions in
         planarRegionGraph.collapseGraphByMerging(parameters.getUpdateAlphaTowardsMatch());

         // go back through the existing regions and add them to the graph to check for overlap
         checkMapRegionsForOverlap(planarRegionGraph,
                                   (float) parameters.getSimilarityThresholdBetweenNormals(),
                                   (float) parameters.getOrthogonalDistanceThreshold(),
                                   (float) parameters.getMaxInterRegionDistance());

         planarRegionGraph.collapseGraphByMerging(parameters.getUpdateAlphaTowardsMatch());

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

   public PlanarRegionsList selfReduceRegionsIteratively(PlanarRegionsList map, HashMap<Integer, TIntArrayList> matches)
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

                  if (PlanarRegionSLAMTools.checkRegionsForOverlap(parentRegion, childRegion, (float) parameters.getSimilarityThresholdBetweenNormals(),
                                                                   (float) parameters.getOrthogonalDistanceThreshold(),
                                                                   (float) parameters.getMinimumOverlapThreshold()))
                  {
                     // Request a merge for parent and child regions. If they don't merge, pick the parent if it belongs to the map (based on sign of ID)
                     if (PlanarRegionSLAMTools.mergeRegionIntoParent(parentRegion, childRegion, parameters.getUpdateAlphaTowardsMatch()))
                     {
                        //LogTools.info("Merged({},{}) -> Removing({})", parentIndex, childIndex, childIndex);

                        // Generate ID for the merged region
                        int finalId = generatePostMergeId(parentRegion.getRegionId(), childRegion.getRegionId());
                        parentRegion.setRegionId(finalId);
                        changed = true;

                        map.getPlanarRegionsAsList().remove(childRegion);
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

   public PlanarRegionsList crossReduceRegionsIteratively(PlanarRegionsList map, PlanarRegionsList regions, HashMap<Integer, TIntArrayList> incomingMatches)
   {
      LogTools.info("Performing Cross Reduction");
      map.addPlanarRegionsList(regions);
      map = selfReduceRegionsIteratively(map, incomingMatches);
      return map;
   }

   public void applyFactorGraphBasedSmoothing(PlanarRegionsList mapRegionList,
                                              PlanarRegionsList incomingRegionList,
                                              RigidBodyTransform estimatedSensorToWorldTransform,
                                              HashMap<Integer, TIntArrayList> matches,
                                              int poseIndex)
   {
      LogTools.info("------------------------------- Performing Factor Graph Based Smoothing ---------------------------------");

      worldToSensorFrameTransform.setAndInvert(estimatedSensorToWorldTransform);

      worldToPreviousSensorFrameTransform.setAndInvert(previousSensorToWorldFrameTransform);

      odometry.set(worldToPreviousSensorFrameTransform);
      odometry.multiply(estimatedSensorToWorldTransform);

      // Convert odometry to float array. Add into factor graph using 16-double method for addOdometryFactor().
      float[] odometryValue = new float[16];
      odometry.get(odometryValue);

      LogTools.info("Adding odometry factor: x{} -> x{}", poseIndex - 1, poseIndex);
      factorGraph.addOdometryFactorExtended(odometryValue, poseIndex);

      //Convert sensor pose to float array. Add 16-float method for setPoseInitialValue().
      float[] poseValue = new float[16];
      estimatedSensorToWorldTransform.get(poseValue);

      LogTools.info("Adding Pose Initial Value: {}", poseIndex);
      factorGraph.setPoseInitialValueExtended(poseIndex, poseValue);

      for (Integer incomingIndex : matches.keySet())
      {
         int incomingId = incomingRegionList.getPlanarRegion(incomingIndex).getRegionId();
         PlanarRegion incomingRegion = incomingRegionList.getPlanarRegion(incomingIndex);

         int landmarkId = findBestMatchIndex(incomingId, mapRegionList, matches.get(incomingIndex));
         insertLandmarkFactorAndValue(incomingRegion, previousSensorToWorldFrameTransform, odometry, poseIndex, landmarkId);
      }

      LogTools.info("Solving factor graph");
      factorGraph.optimizeISAM2((byte) 4);

      factorGraph.printResults();

      LogTools.info("+++++++++ --------- +++++++++ Done (Smoothing) +++++++++ --------- +++++++++");
   }

   public void insertLandmarkFactorAndValue(PlanarRegion childRegion, RigidBodyTransform previousToWorldFrameTransform,
                                                RigidBodyTransform previousToCurrentFrameTransform, int poseId, int landmarkId)
   {
      RigidBodyTransform currentToWorldFrameTransform = new RigidBodyTransform();
      currentToWorldFrameTransform.setAndInvert(previousToCurrentFrameTransform);
      currentToWorldFrameTransform.multiply(previousToWorldFrameTransform);

      // Get origin and normal in sensor frame.
      Point3D childOrigin = new Point3D();
      Vector3D childNormal = new Vector3D();
      childRegion.getOrigin(childOrigin);
      childRegion.getNormal(childNormal);

      // Get origin and normal in world frame
      Point3D childOriginInWorld = new Point3D(childOrigin);
      Vector3D childNormalInWorld = new Vector3D(childNormal);
      childOriginInWorld.applyTransform(currentToWorldFrameTransform);
      childNormalInWorld.applyTransform(currentToWorldFrameTransform);

      // Compute plane parameters in sensor frame
      double localDot = childOrigin.dot(childNormal);
      float[] planeInSensorFrame = new float[] {childNormal.getX32(), childNormal.getY32(), childNormal.getZ32(), (float) -localDot};

      // Insert plane factor based on planar region normal and origin in sensor frame
      LogTools.info("Adding plane factor: x{} -> l{} ({})", poseId, landmarkId, Arrays.toString(planeInSensorFrame));
      factorGraph.addOrientedPlaneFactor(planeInSensorFrame, landmarkId, poseId);

      // Compute plane parameters in world frame
      double worldDot = childOriginInWorld.dot(childNormalInWorld);
      float[] planeInWorldFrame = new float[] {childNormalInWorld.getX32(), childNormalInWorld.getY32(), childNormalInWorld.getZ32(), (float) -worldDot};

      // Set initial value for plane based on planar region normal and origin in world frame
      LogTools.info("Setting plane value: {} {}", landmarkId, Arrays.toString(planeInWorldFrame));
      factorGraph.setOrientedPlaneInitialValue(landmarkId, planeInWorldFrame);
   }

   public void initializeFactorGraphForSmoothing(PlanarRegionsList map, RigidBodyTransform transform)
   {
      LogTools.info("------------------------------- Initializing Factor Graph ---------------------------------");

      Vector3DBasics translation = transform.getTranslation();
      Vector3D eulerAngles = new Vector3D();
      transform.getRotation().getEuler(eulerAngles);
      float[] initialPose = new float[] {eulerAngles.getZ32(), eulerAngles.getY32(), eulerAngles.getX32(),
                                         translation.getX32(), translation.getY32(), translation.getZ32()};

      factorGraph.addPriorPoseFactor(0, initialPose);
      factorGraph.setPoseInitialValue(0, initialPose);

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

      LogTools.info("+++++++++ --------- +++++++++ Done (Smoothing Initialization) +++++++++ --------- +++++++++");
   }

   private int findBestMatchIndex(int incomingId, PlanarRegionsList mapRegions, TIntArrayList matchedMapIndices)
   {
      int landmarkId = mapRegions.getPlanarRegion(0).getRegionId();

      for (Integer mapIndex : matchedMapIndices.toArray())
      {
         int mapId = mapRegions.getPlanarRegion(mapIndex).getRegionId();
         int bestLandmarkId = Math.abs(generatePostMergeId(incomingId, mapId));

         if (bestLandmarkId > 0)
            landmarkId = Math.min(bestLandmarkId, landmarkId);
      }

      return landmarkId;
   }

   private int generatePostMergeId(int parentIndex, int childIndex)
   {
      int idToReturn = 0;
      if (parentIndex > 0 && childIndex > 0) // Both belong to the map
      {
         idToReturn = Math.min(parentIndex, childIndex);
         LogTools.info("Action: Both belong to the map: ({}) ({}) -> ({})", parentIndex, childIndex, idToReturn);
      }
      else if (parentIndex > 0 && childIndex < 0) // Parent belongs to the map, child belongs to the incoming regions
      {
         idToReturn = parentIndex;
         LogTools.info("Action: Parent in map, child incoming: ({}) ({}) -> ({})", parentIndex, childIndex, idToReturn);
      }
      else if (parentIndex < 0 && childIndex > 0) // Parent belongs to the incoming regions, child belongs to the map
      {
         idToReturn = childIndex;
         LogTools.info("Action: Parent incoming, child in map: ({}) ({}) -> ({})", parentIndex, childIndex, idToReturn);
      }
      else if (parentIndex < 0 && childIndex < 0) // Both belong to the incoming regions
      {
         idToReturn = Math.max(parentIndex, childIndex);
         LogTools.info("Action: Both incoming: ({}) ({}) -> ({})", parentIndex, childIndex, idToReturn);
      }
      return idToReturn;
   }

   private void processUniqueRegions(PlanarRegionsList map)
   {
      LogTools.info("------------------------------- Processing Unique Region IDs ---------------------------------");
      for (PlanarRegion region : map.getPlanarRegionsAsList())
      {
         region.setRegionId(Math.abs(region.getRegionId()));
      }
      LogTools.info("+++++++++ --------- +++++++++ Done (Processing Unique Region IDs) +++++++++ --------- +++++++++");
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

   public void destroy()
   {
      factorGraph.clearISAM2();
   }
}

