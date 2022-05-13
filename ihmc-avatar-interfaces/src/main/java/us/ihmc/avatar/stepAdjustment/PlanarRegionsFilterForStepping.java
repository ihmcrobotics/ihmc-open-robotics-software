package us.ihmc.avatar.stepAdjustment;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegion;

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

   public double getMinPlanarRegionArea()
   {
      return minPlanarRegionArea;
   }

   public double getMaxPlanarRegionAngle()
   {
      return maxPlanarRegionAngle;
   }

   private final RecyclingArrayList<PlanarRegion> planarRegionsList = new RecyclingArrayList<>(PlanarRegion::new);

   public void setPlanarRegions(PlanarRegionsListCommand planarRegions)
   {
      planarRegionsList.clear();
      for (int i = 0; i < planarRegions.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegionCommand candidateRegion = planarRegions.getPlanarRegionCommand(i);

         double polygonArea = EuclidGeometryPolygonTools.computeConvexPolygon2DArea(candidateRegion.getConcaveHullsVertices(),
                                                                                    candidateRegion.getConcaveHullsVertices().size(),
                                                                                    true,
                                                                                    null);
         if (polygonArea > getMinPlanarRegionArea())
            continue;

         if (candidateRegion.getTransformToWorld().getM22() >= Math.cos(getMaxPlanarRegionAngle()))
            continue;

         PlanarRegion planarRegion = planarRegionsList.add();
         planarRegion.set(candidateRegion.getTransformToWorld(), candidateRegion.getConvexPolygons(), candidateRegion.getRegionId());
      }
   }

   public List<PlanarRegion> getPlanarRegions()
   {
      return planarRegionsList;
   }
}
