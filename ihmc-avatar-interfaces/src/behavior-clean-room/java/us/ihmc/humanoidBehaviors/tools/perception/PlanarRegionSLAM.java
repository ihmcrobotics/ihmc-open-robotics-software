package us.ihmc.humanoidBehaviors.tools.perception;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.lists.PairList;

public class PlanarRegionSLAM
{
   private static boolean verbose = false;

   public static PlanarRegionSLAMResult slam(PlanarRegionsList map, PlanarRegionsList newDataIn)
   {
      return slam(map, newDataIn, new PlanarRegionSLAMParameters());
   }

   /**
    * Updates the map with new data and returns the detected drift.
    *
    * @param map
    * @param newData
    * @return detected drift
    */
   public static PlanarRegionSLAMResult slam(PlanarRegionsList map, PlanarRegionsList newDataIn, PlanarRegionSLAMParameters parameters)
   {
      PlanarRegionsList transformedNewData = newDataIn;
      RigidBodyTransform totalDriftCorrectionTransform = new RigidBodyTransform();

      //TODO: Put in a parameter file somewhere.

      for (int i = 0; i < parameters.getIterations(); i++)
      {
         Map<PlanarRegion, PairList<PlanarRegion, Point2D>> matchesWithReferencePoints = findHighConfidenceRegionMatchesAndReferencePoints(map,
                                                                                                                                           transformedNewData,
                                                                                                                                           parameters);

         if (verbose)
         {
            LogTools.info("matchesWithReferencePoints.keys: {}", matchesWithReferencePoints.size());
            LogTools.info("matchesWithReferencePoints.values: {}", matchesWithReferencePoints.values().size());
         }

         RigidBodyTransform driftCorrectionTransform = PlanarRegionSLAMTools.findDriftCorrectionTransform(matchesWithReferencePoints);
         //TODO: Multiply or post multiply???
         totalDriftCorrectionTransform.preMultiply(driftCorrectionTransform);

         transformedNewData = transformedNewData.copy();
         transformedNewData.transformByPreMultiply(driftCorrectionTransform);

         if (verbose)
         {
            LogTools.info(driftCorrectionTransform.toString());
         }
      }

      PlanarRegionsList mergedMap = new PlanarRegionsList();
      map.getPlanarRegionsAsList().forEach(region -> mergedMap.addPlanarRegion(region));
      transformedNewData.getPlanarRegionsAsList().forEach(region -> mergedMap.addPlanarRegion(region));

      PlanarRegionSLAMResult result = new PlanarRegionSLAMResult(totalDriftCorrectionTransform, mergedMap);
      return result;
   }

   /**
    * Looks through two PlanarRegionsLists and finds PlanarRegion pairs that are good potential matches.
    * @param map The map that you are building.
    * @param newData The newData that you are adding to the map.
    * @return A Map from PlanarRegions in the map to matching regions in the new data, and matching reference points in the new Data.
    */
   public static Map<PlanarRegion, PairList<PlanarRegion, Point2D>> findHighConfidenceRegionMatchesAndReferencePoints(PlanarRegionsList map,
                                                                                                                      PlanarRegionsList newData,
                                                                                                                      PlanarRegionSLAMParameters parameters)
   {
      Map<PlanarRegion, List<PlanarRegion>> boundingBox3DCollisions = PlanarRegionSLAMTools.detectLocalBoundingBox3DCollisions(map, newData,
                                                                                                                               parameters.getBoundingBoxHeight());

      if (verbose)
      {
         LogTools.info("boundboxCollisions.keys: {}", boundingBox3DCollisions.size());
         LogTools.info("boundboxCollisions.values: {}", boundingBox3DCollisions.values().size());
      }

      Map<PlanarRegion, List<PlanarRegion>> normalSimilarityFiltered = PlanarRegionSLAMTools.filterMatchesBasedOnNormalSimilarity(boundingBox3DCollisions,
                                                                                                                                  parameters.getMinimumNormalDotProduct());

      if (verbose)
      {
         LogTools.info("normalSimilarityFiltered.keys: {}", normalSimilarityFiltered.size());
         LogTools.info("normalSimilarityFiltered.values: {}", normalSimilarityFiltered.values().size());
      }

      Map<PlanarRegion, PairList<PlanarRegion, Point2D>> matchesWithReferencePoints = PlanarRegionSLAMTools.filterMatchesBasedOn2DBoundingBoxShadow(normalSimilarityFiltered);
      return matchesWithReferencePoints;
   }

   public static PlanarRegionSLAMResult intentionallyDrift(PlanarRegionsList newData)
   {
      AxisAngle smallRotation = new AxisAngle(0.1, 0.1, 0.1);
      Vector3D smallTranslation = new Vector3D(0.05, -0.05, 0.05);
      RigidBodyTransform smallTransform = new RigidBodyTransform(smallRotation, smallTranslation);
      PlanarRegionsList transformedNewData = newData.copy();
      transformedNewData.transform(smallTransform);

      PlanarRegionSLAMResult result = new PlanarRegionSLAMResult(smallTransform, transformedNewData);
      return result;
   }

   private static void mergeNewDataIntoMap(PlanarRegionsList map, PlanarRegionsList newData)
   {
      List<PlanarRegion> allRegionsInWorld = new ArrayList<>(); // worry about merging ids?
      allRegionsInWorld.addAll(map.getPlanarRegionsAsList());
      allRegionsInWorld.addAll(newData.getPlanarRegionsAsList());

      // match regions by normality and coplanarity (this time including partially visible surfaces)
      List<List<PlanarRegion>> regionsToMerge = matchRegionsToMerge(allRegionsInWorld);

      List<PlanarRegion> mergedRegions = mergeRegions(regionsToMerge);

      map.clear();
      for (PlanarRegion mergedRegion : mergedRegions)
      {
         map.addPlanarRegion(mergedRegion); // set ids?
      }
   }

   public static RigidBodyTransform snapToObviousGroundPlane(PlanarRegionsList map, PlanarRegionsList newData)
   {
      // if there exist two very large ground planes that intersect very near to the robot, then snap drifted to map's
      // ground plane by rotating on ground planes' intersection axis

      RigidBodyTransform drift = new RigidBodyTransform();
      return drift;
   }

   public static RigidBodyTransform findDriftCorrectionTransform(PairList<PlanarRegion, PlanarRegion> highConfidencePairs)
   {
      RigidBodyTransform transform = new RigidBodyTransform();

      // find average transform of all pairs

      return transform;
   }

   public static List<List<PlanarRegion>> matchRegionsToMerge(List<PlanarRegion> regions)
   {
      ArrayList<List<PlanarRegion>> mappedRegions = new ArrayList<>();

      // this is probably also n squared for now?
      // need some data structure to sort them into a map

      return mappedRegions;
   }

   private static List<PlanarRegion> mergeRegions(List<List<PlanarRegion>> regionsToMerge)
   {
      ArrayList<PlanarRegion> mergedRegions = new ArrayList<>();

      for (List<PlanarRegion> touchingRegionsOnSamePlane : regionsToMerge)
      {
         // PlanarRegionTools.merge(region1, region2)
      }

      return mergedRegions;
   }
}
