package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.euclid.Axis;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;

public class ObstacleCoursePlanarRegionEnvironment extends PlanarRegionEnvironmentInterface
{
   public ObstacleCoursePlanarRegionEnvironment()
   {
      // ground plane
      generator.translate(0.0, 0.0, -0.01);
      generator.addRectangle(30.0, 30.0);

      // staircase
      generator.identity();
      generator.translate(3.0, 0.0, 0.0);
      double stepHeight = 0.13;
      double stepLength = 0.30;
      int numberOfSteps = 6;
      double startingBlockLength = 1.0;

      for (int i = 0; i < numberOfSteps; i++)
      {
         generator.translate(stepLength, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(stepLength, 1.0, (i + 1) * stepHeight);
      }

      generator.translate(0.5 * (startingBlockLength + stepLength), 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(startingBlockLength, startingBlockLength, numberOfSteps * stepHeight);
      generator.translate(0.5 * (startingBlockLength - stepLength), 0.0, 0.0);

      for (int i = 0; i < numberOfSteps; i++)
      {
         generator.translate(stepLength, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(stepLength, 1.0, (numberOfSteps - i) * stepHeight);
      }

      // ramps
      // TODO

      // cinder blocks
      generator.identity();
      generator.translate(-3.0, 0.0, -0.05);
      generator.rotate(Math.PI, Axis.Z);
      PlanarRegionsListExamples.generateCinderBlockField(generator, 0.4, 0.1, 9, 9, 0.1);
   }
}
