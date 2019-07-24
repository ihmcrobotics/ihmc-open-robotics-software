package us.ihmc.humanoidBehaviors.tools.perception;

import java.util.HashSet;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ConcaveHullMerger;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ConcaveHullMergerListener;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.lists.PairList;

public class PlanarRegionSLAM
{
   public static PlanarRegionSLAMResult slam(PlanarRegionsList map, PlanarRegionsList newDataIn, PlanarRegionSLAMParameters parameters)
   {
      return slam(map, newDataIn, parameters, null);
   }

   /**
    * Updates the map with new data and returns the detected drift.
    *
    * @param map
    * @param newDataIn
    * @param parameters
    * @return Merged PlanarRegionsList and drift transform
    */
   public static PlanarRegionSLAMResult slam(PlanarRegionsList map, PlanarRegionsList newDataIn, PlanarRegionSLAMParameters parameters,
                                             ConcaveHullMergerListener listener)
   {
      PlanarRegionsList transformedNewData = newDataIn;
      RigidBodyTransform totalDriftCorrectionTransform = new RigidBodyTransform();

      for (int i = 0; i < parameters.getIterations(); i++)
      {
         Map<PlanarRegion, PairList<PlanarRegion, Point2D>> matchesWithReferencePoints = findHighConfidenceRegionMatchesAndReferencePoints(map,
                                                                                                                                           transformedNewData,
                                                                                                                                           parameters);

         RigidBodyTransform driftCorrectionTransform = PlanarRegionSLAMTools.findDriftCorrectionTransform(matchesWithReferencePoints, parameters);
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
               newRegionsConsidered.add(newRegion);

               mapPlanarRegion = ConcaveHullMerger.mergePlanarRegions(mapPlanarRegion, newRegion, listener);
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
                                                                                                                                                    normalSimilarityFiltered);
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

}
