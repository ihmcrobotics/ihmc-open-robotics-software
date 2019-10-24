package us.ihmc.robotEnvironmentAwareness.planarRegion.slam;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMerger;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMergerListener;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.lists.PairList;

public class PlanarRegionSLAM
{
   public static PlanarRegionSLAMResult slam(PlanarRegionsList map, PlanarRegionsList newDataIn, PlanarRegionSLAMParameters parameters)
   {
      return slam(map, newDataIn, parameters, null, null);
   }

   public static PlanarRegionSLAMResult slam(PlanarRegionsList map, PlanarRegionsList newDataIn, PlanarRegionSLAMParameters parameters,
                                             ConcaveHullMergerListener listener)
   {
      return slam(map, newDataIn, parameters, null, listener);
   }

   public static PlanarRegionSLAMResult slam(PlanarRegionsList map, PlanarRegionsList newDataIn, PlanarRegionSLAMParameters parameters, RigidBodyTransform referenceTransform)
   {
      return slam(map, newDataIn, parameters, referenceTransform, null);
   }

   /**
    * Updates the map with new data and returns the detected drift.
    *
    * @param map
    * @param newDataIn
    * @param parameters
    * @return Merged PlanarRegionsList and drift transform
    */
   public static PlanarRegionSLAMResult slam(PlanarRegionsList map, PlanarRegionsList newDataIn, PlanarRegionSLAMParameters parameters, RigidBodyTransform referenceTransform,
                                             ConcaveHullMergerListener listener)
   {
      PlanarRegionsList transformedNewData = newDataIn;
      RigidBodyTransform totalDriftCorrectionTransform = new RigidBodyTransform();

      for (int i = 0; i < parameters.getIterationsForMatching(); i++)
      {
         Map<PlanarRegion, PairList<PlanarRegion, Point2D>> matchesWithReferencePoints = findHighConfidenceRegionMatchesAndReferencePoints(map,
                                                                                                                                           transformedNewData,
                                                                                                                                           parameters);

         RigidBodyTransform driftCorrectionTransform = PlanarRegionSLAMTools.findDriftCorrectionTransform(matchesWithReferencePoints, parameters, referenceTransform);
         totalDriftCorrectionTransform.preMultiply(driftCorrectionTransform);

         transformedNewData = transformedNewData.copy();
         transformedNewData.transformByPreMultiply(driftCorrectionTransform);
      }

      PlanarRegionsList mergedMap = generateMergedMapByMergingAllPlanarRegionsMatches(map, transformedNewData, parameters, listener);
      PlanarRegionSLAMResult result = new PlanarRegionSLAMResult(totalDriftCorrectionTransform, mergedMap);
      return result;
   }

   private static PlanarRegionsList generateMergedMapByMergingAllPlanarRegionsMatches(PlanarRegionsList map, PlanarRegionsList transformedNewData,
                                                                                      PlanarRegionSLAMParameters parameters, ConcaveHullMergerListener listener)
   {
      Map<PlanarRegion, PairList<PlanarRegion, Point2D>> matchesWithReferencePoints = findHighConfidenceRegionMatchesAndReferencePoints(map,
                                                                                                                                        transformedNewData,
                                                                                                                                        parameters);

      PlanarRegionsList mergedMap = new PlanarRegionsList();

      HashSet<PlanarRegion> newRegionsConsidered = new HashSet<PlanarRegion>();

      List<PlanarRegion> mapPlanarRegions = map.getPlanarRegionsAsList();

      for (PlanarRegion mapPlanarRegion : mapPlanarRegions)
      {
         PairList<PlanarRegion, Point2D> matchingRegions = matchesWithReferencePoints.get(mapPlanarRegion);
         if (matchingRegions != null)
         {
            for (ImmutablePair<PlanarRegion, Point2D> matchingRegion : matchingRegions)
            {
               PlanarRegion newRegion = matchingRegion.getLeft();
               if (newRegionsConsidered.contains(newRegion))
               {
                  continue;
               }

               ArrayList<PlanarRegion> mergedMapPlanarRegions = ConcaveHullMerger.mergePlanarRegions(mapPlanarRegion,
                                                                                                     newRegion.copy(),
                                                                                                     parameters.getMaximumPointProjectionDistance(),
                                                                                                     listener);

               if (mergedMapPlanarRegions == null)
               {
                  // If something went wrong, just throw out both the map and the new region.
                  LogTools.error("Trouble with merging planar regions. Throwing both of them out.");
                  newRegionsConsidered.add(newRegion);
               }

               else if (mergedMapPlanarRegions.isEmpty())
               {
                  // If there was no intersection, keep both map and new region.
               }

               else
               {
                  mapPlanarRegion = mergedMapPlanarRegions.get(0);
                  newRegionsConsidered.add(newRegion);
               }
            }
         }

         mergedMap.addPlanarRegion(mapPlanarRegion.copy());
      }

      for (PlanarRegion newRegion : transformedNewData.getPlanarRegionsAsList())
      {
         if (newRegionsConsidered.contains(newRegion))
         {
            continue;
         }
         mergedMap.addPlanarRegion(newRegion);
      }

      return mergedMap;
   }

   /**
    * Looks through two PlanarRegionsLists and finds PlanarRegion pairs that are good potential
    * matches.
    * 
    * @param map     The map that you are building.
    * @param newData The newData that you are adding to the map.
    * @return A Map from PlanarRegions in the map to matching regions in the new data, and matching
    *         reference points in the new Data.
    */
   public static Map<PlanarRegion, PairList<PlanarRegion, Point2D>> findHighConfidenceRegionMatchesAndReferencePoints(PlanarRegionsList map,
                                                                                                                      PlanarRegionsList newData,
                                                                                                                      PlanarRegionSLAMParameters parameters)
   {
      Map<PlanarRegion, List<PlanarRegion>> boundingBox3DCollisions = PlanarRegionSLAMTools.detectLocalBoundingBox3DCollisions(map,
                                                                                                                               newData,
                                                                                                                               parameters.getBoundingBoxHeight());

      Map<PlanarRegion, List<PlanarRegion>> normalSimilarityFiltered = PlanarRegionSLAMTools.filterMatchesBasedOnNormalSimilarity(boundingBox3DCollisions,
                                                                                                                                  parameters.getMinimumNormalDotProduct());

      Map<PlanarRegion, PairList<PlanarRegion, Point2D>> matchesWithReferencePoints = PlanarRegionSLAMTools.filterMatchesBasedOn2DBoundingBoxShadow(parameters.getMinimumRegionOverlapDistance(),
                                                                                                                                                    parameters.getMaximumPointProjectionDistance(),
                                                                                                                                                    normalSimilarityFiltered);
      return matchesWithReferencePoints;
   }

   public static PlanarRegionSLAMResult intentionallyRandomlyDrift(PlanarRegionsList newData, Random random)
   {
      AxisAngle smallRotation = new AxisAngle(random.nextDouble() % 0.2, random.nextDouble() % 0.2, random.nextDouble() % 0.2);
      Vector3D smallTranslation = new Vector3D((random.nextDouble() - 0.5) % 0.1, (random.nextDouble() - 0.5) % 0.1, (random.nextDouble() - 0.5) % 0.1);
      RigidBodyTransform smallTransform = new RigidBodyTransform(smallRotation, smallTranslation);
      PlanarRegionsList transformedNewData = newData.copy();
      transformedNewData.transformByPreMultiply(smallTransform);

      PlanarRegionSLAMResult result = new PlanarRegionSLAMResult(smallTransform, transformedNewData);
      return result;
   }
}
