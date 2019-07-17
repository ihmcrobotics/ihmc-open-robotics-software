package us.ihmc.humanoidBehaviors.tools.perception;

import javafx.util.Pair;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

public class PlanarRegionSLAM
{
   /**
    * Updates the map with new data and returns the detected drift.
    *
    * @param map
    * @param newData
    * @return detected drift
    */
   public static PlanarRegionSLAMResult slam(PlanarRegionsList map, PlanarRegionsList newData)
   {
      PlanarRegionsList mergedMap = new PlanarRegionsList();
      map.getPlanarRegionsAsList().forEach(region -> mergedMap.addPlanarRegion(region));
      newData.getPlanarRegionsAsList().forEach(region -> mergedMap.addPlanarRegion(region));

      RigidBodyTransform transformFromIncomingToMap = new RigidBodyTransform();

      PlanarRegionSLAMResult result = new PlanarRegionSLAMResult(transformFromIncomingToMap, mergedMap);
      return result;

      //      RigidBodyTransform detectedDrift = findDrift(map, newData);
      //      mergeNewDataIntoMap(map, newData);
      //      return detectedDrift;
   }

   private static RigidBodyTransform findDrift(PlanarRegionsList map, PlanarRegionsList newData)
   {
      RigidBodyTransform detectedDrift = new RigidBodyTransform();

      // could do an obvious ground correction at very beginning
      RigidBodyTransform groundPlaneCorrection = snapToObviousGroundPlane(map, newData);
      detectedDrift.transform(groundPlaneCorrection); // this is probably wrong

      // store regions in data structure by pose and size (shape similarity?)

      // find high confidence region matches (includes only: surfaces fully visible in both lists, i.e. excludes ground)
      List<Pair<PlanarRegion, PlanarRegion>> highConfidencePairs = findHighConfidencePairs(map, newData);

      // correct drift based on high confidence region pairs; hopefully there are many varied normals
      RigidBodyTransform snappedPairsCorrection = findDriftCorrectionTransform(highConfidencePairs);
      newData.transform(snappedPairsCorrection); // this is probably wrong
      detectedDrift.transform(snappedPairsCorrection); // this is probably wrong
      return detectedDrift;
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

   
   /**
    * Looks through two PlanarRegionsLists and finds PlanarRegion pairs that are good potential matches.
    * @param map The map that you are building.
    * @param newData The newData that you are adding to the map.
    * @return List of Pairs of PlanarRegions that are good potential matches.
    */
   public static List<Pair<PlanarRegion, PlanarRegion>> findHighConfidencePairs(PlanarRegionsList map, PlanarRegionsList newData)
   {
      ArrayList<Pair<PlanarRegion, PlanarRegion>> pairs = new ArrayList<>();

      // probably n squared search for now?

      return pairs;
   }

   public static RigidBodyTransform findDriftCorrectionTransform(List<Pair<PlanarRegion, PlanarRegion>> highConfidencePairs)
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
