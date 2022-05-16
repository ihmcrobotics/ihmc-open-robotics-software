package us.ihmc.avatar.stepAdjustment;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.List;

public class PlanarRegionsFilterForStepping
{

   /**
    * Minimum planar region area for a foothold
    */
   private double minPlanarRegionArea = 0.05;

   /**
    * Maximum angle of planar region considered for a foothold.
    * Zero degrees corresponds to a vertical normal
    */
   private double maxPlanarRegionAngle = Math.toRadians(25.0);

   private final RecyclingArrayList<PlanarRegion> allPlanarRegionsList = new RecyclingArrayList<>(PlanarRegion::new);
   private final List<PlanarRegion> bigEnoughPlanarRegionsList = new ArrayList<>();
   private final List<PlanarRegion> filteredPlanarRegionsList = new ArrayList<>();


   public double getMinPlanarRegionArea()
   {
      return minPlanarRegionArea;
   }

   public double getMaxPlanarRegionAngle()
   {
      return maxPlanarRegionAngle;
   }


   public void setPlanarRegions(PlanarRegionsListCommand planarRegions)
   {
      allPlanarRegionsList.clear();
      bigEnoughPlanarRegionsList.clear();
      filteredPlanarRegionsList.clear();

      for (int i = 0; i < planarRegions.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegionCommand candidateRegion = planarRegions.getPlanarRegionCommand(i);

         double polygonArea = EuclidGeometryPolygonTools.computeConvexPolygon2DArea(candidateRegion.getConcaveHullsVertices(),
                                                                                    candidateRegion.getConcaveHullsVertices().size(),
                                                                                    true,
                                                                                    null);

         PlanarRegion planarRegion = allPlanarRegionsList.add();
         planarRegion.set(candidateRegion.getTransformToWorld(), candidateRegion.getConvexPolygons(), candidateRegion.getRegionId());

         if (polygonArea > getMinPlanarRegionArea())
            continue;

         bigEnoughPlanarRegionsList.add(planarRegion);

         if (candidateRegion.getTransformToWorld().getM22() >= Math.cos(getMaxPlanarRegionAngle()))
            continue;

         filteredPlanarRegionsList.add(planarRegion);
      }
   }

   public List<PlanarRegion> getPlanarRegions()
   {
      return allPlanarRegionsList;
   }

   public List<PlanarRegion> getBigEnoughPlanarRegions()
   {
      return bigEnoughPlanarRegionsList;
   }

   public List<PlanarRegion> getFilteredPlanarRegions()
   {
      return filteredPlanarRegionsList;
   }
}
