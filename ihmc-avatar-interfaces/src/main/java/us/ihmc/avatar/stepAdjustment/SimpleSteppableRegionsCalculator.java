package us.ihmc.avatar.stepAdjustment;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
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
   private final ConvexPolygon2D convexHullOfAllThePlanarRegions = new ConvexPolygon2D();

   private final FramePoint2D tempPoint = new FramePoint2D();

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
            polygonArea += candidateRegion.getConvexPolygons().get(regionIdx).getArea();

         if (polygonArea < minRegionArea.getValue())
            continue;

         if (Math.abs(candidateRegion.getTransformToWorld().getM22()) < Math.cos(maxNormalAngle.getValue()))
            continue;

         PlanarRegion planarRegion = steppableRegionsList.add();
         planarRegion.set(candidateRegion.getTransformToWorld(), candidateRegion.getConvexPolygons(), candidateRegion.getConcaveHullsVertices(), candidateRegion.getRegionId());
      }

      computeBoundaryOfPlanarRegions(steppableRegionsList);
   }

   private void computeBoundaryOfPlanarRegions(List<PlanarRegion> planarRegionsList)
   {
      // get the convex hull of all the planar regions in the environment by adding all the convex hulls
      convexHullOfAllThePlanarRegions.clear();
      for (int i = 0; i < planarRegionsList.size(); i++)
      {
         PlanarRegion region = planarRegionsList.get(i);

         for (int j = 0; j < region.getConcaveHullSize(); j++)
         {
            region.getTransformToWorld().transform(region.getConcaveHullVertex(j), tempPoint, false);
            convexHullOfAllThePlanarRegions.addVertex(tempPoint);
         }
      }
      convexHullOfAllThePlanarRegions.update();

   }

   @Override
   public List<PlanarRegion> getSteppableRegions()
   {
      return steppableRegionsList;
   }

   @Override
   public ConvexPolygon2DReadOnly getConvexHullOfAllRegions()
   {
      return convexHullOfAllThePlanarRegions;
   }
}
