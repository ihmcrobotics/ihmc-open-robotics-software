package us.ihmc.avatar.stepAdjustment;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStepParameters;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.List;

public class SimpleSteppableRegionsCalculator implements SteppableRegionsProvider
{
   private final DoubleProvider minRegionArea;
   private final DoubleProvider maxNormalAngle;

   private final RecyclingArrayList<PlanarRegion> steppableRegionsList = new RecyclingArrayList<>(PlanarRegion::new);

   public SimpleSteppableRegionsCalculator()
   {
      this(new SnapAndWiggleSingleStepParameters());
   }

   public SimpleSteppableRegionsCalculator(SnapAndWiggleSingleStepParameters parameters)
   {
      this(parameters::getMinPlanarRegionArea, parameters::getMaxPlanarRegionAngle);
   }

   public SimpleSteppableRegionsCalculator(DoubleProvider minRegionArea, DoubleProvider maxNormalAngle)
   {
      this.minRegionArea = minRegionArea;
      this.maxNormalAngle = maxNormalAngle;
   }

   @Override
   public void consume(PlanarRegionsListCommand planarRegions)
   {
      steppableRegionsList.clear();
      for (int i = 0; i < planarRegions.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegionCommand candidateRegion = planarRegions.getPlanarRegionCommand(i);

         double polygonArea = 0.0;
         for (int regionIdx = 0; regionIdx < candidateRegion.getConvexPolygons().size(); regionIdx++)
            polygonArea += candidateRegion.getConvexPolygons().get(i).getArea();

         if (polygonArea < minRegionArea.getValue())
            continue;

         if (Math.abs(candidateRegion.getTransformToWorld().getM22()) < Math.cos(maxNormalAngle.getValue()))
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
