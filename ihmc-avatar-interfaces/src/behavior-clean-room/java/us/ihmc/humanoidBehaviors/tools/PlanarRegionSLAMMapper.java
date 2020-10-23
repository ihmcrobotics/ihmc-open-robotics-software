package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMergerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;

/**
 * Builds and provides a map by subscribing to REA.
 */
public class PlanarRegionSLAMMapper
{
   private volatile PlanarRegionsList slamMap = new PlanarRegionsList();
   private PlanarRegionSLAMParameters planarRegionSLAMParameters = new PlanarRegionSLAMParameters();

   public PlanarRegionsList update(PlanarRegionsList planarRegionsUpdate)
   {
      if (slamMap.isEmpty())
      {
         slamMap = planarRegionsUpdate;
      }
      else
      {
         try
         {
            slamMap = PlanarRegionSLAM.slam(slamMap, planarRegionsUpdate, planarRegionSLAMParameters, (ConcaveHullMergerListener) null).getMergedMap();
         }
         catch (Exception e)
         {
            // do nothing but need to fix these crashes
         }
      }

      return slamMap;
   }

   public void clear()
   {
      slamMap = new PlanarRegionsList();
   }

   public PlanarRegionsList getSlamMap()
   {
      return slamMap;
   }
}
