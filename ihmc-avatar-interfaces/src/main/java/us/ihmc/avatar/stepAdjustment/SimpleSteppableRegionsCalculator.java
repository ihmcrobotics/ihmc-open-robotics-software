package us.ihmc.avatar.stepAdjustment;

import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.SteppableRegionsProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStepParameters;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.List;

public class SimpleSteppableRegionsCalculator implements SteppableRegionsProvider
{
   private final SnapAndWiggleSingleStepParameters parameters = new SnapAndWiggleSingleStepParameters();

   private final RecyclingArrayList<PlanarRegion> steppableRegionsList = new RecyclingArrayList<>(PlanarRegion::new);


   @Override
   public void consume(PlanarRegionsListCommand planarRegions)
   {
      steppableRegionsList.clear();
      for (int i = 0; i < planarRegions.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegionCommand candidateRegion = planarRegions.getPlanarRegionCommand(i);

         double polygonArea = EuclidGeometryPolygonTools.computeConvexPolygon2DArea(candidateRegion.getConcaveHullsVertices(),
                                                                                    candidateRegion.getConcaveHullsVertices().size(),
                                                                                    true,
                                                                                    null);
         if (polygonArea < parameters.getMinPlanarRegionArea())
            continue;

         if (Math.abs(candidateRegion.getTransformToWorld().getM22()) < Math.cos(parameters.getMaxPlanarRegionAngle()))
            continue;

         PlanarRegion planarRegion = steppableRegionsList.add();
         planarRegion.set(candidateRegion.getTransformToWorld(), candidateRegion.getConvexPolygons(), candidateRegion.getConcaveHullsVertices(), candidateRegion.getRegionId());
      }
   }

   @Override
   public List<PlanarRegion> getSteppableRegions()
   {
      return steppableRegionsList;
   }
}
