package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;
import java.util.stream.Collectors;

public class StepConstraintListConverter
{
   public static List<StepConstraintRegion> convertPlanarRegionListToStepConstraintRegion(PlanarRegionsList planarRegion)
   {
      return convertPlanarRegionListToStepConstraintRegion(planarRegion.getPlanarRegionsAsList());
   }

   public static List<StepConstraintRegion> convertPlanarRegionListToStepConstraintRegion(List<PlanarRegion> planarRegions)
   {
      return planarRegions.stream().map(StepConstraintListConverter::convertPlanarRegionToStepConstraintRegion).collect(Collectors.toList());
   }

   public static void convertPlanarRegionListToStepConstraintRegion(List<PlanarRegion> planarRegions,
                                                                    RecyclingArrayList<StepConstraintRegion> stepConstraintRegionsToPack)
   {
      stepConstraintRegionsToPack.clear();
      for (int i = 0; i < planarRegions.size(); i++)
         convertPlanarRegionToStepConstraintRegion(planarRegions.get(i), stepConstraintRegionsToPack.add());
   }

   public static StepConstraintRegion convertPlanarRegionToStepConstraintRegion(PlanarRegion planarRegion)
   {
      StepConstraintRegion constraintRegion = new StepConstraintRegion();
      convertPlanarRegionToStepConstraintRegion(planarRegion, constraintRegion);

      return constraintRegion;
   }

   public static void convertPlanarRegionToStepConstraintRegion(PlanarRegion planarRegion, StepConstraintRegion stepConstraintRegionToPack)
   {
      stepConstraintRegionToPack.set(planarRegion.getTransformToWorld(), planarRegion.getConcaveHull(), null);
      stepConstraintRegionToPack.setRegionId(planarRegion.getRegionId());
   }
}
