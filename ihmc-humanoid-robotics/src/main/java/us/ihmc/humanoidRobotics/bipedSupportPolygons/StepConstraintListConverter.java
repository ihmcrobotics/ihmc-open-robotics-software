package us.ihmc.humanoidRobotics.bipedSupportPolygons;

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

   public static StepConstraintRegion convertPlanarRegionToStepConstraintRegion(PlanarRegion planarRegion)
   {
      StepConstraintRegion constraintRegion = new StepConstraintRegion(planarRegion.getTransformToWorld(), planarRegion.getConcaveHull());
      constraintRegion.setRegionId(planarRegion.getRegionId());

      return constraintRegion;
   }
}
