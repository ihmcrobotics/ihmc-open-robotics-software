package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.euclid.Axis;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;

public class ObstacleCoursePlanarRegionEnvironment extends PlanarRegionEnvironmentInterface
{
   private static final double RAMP_ANGLE = Math.toRadians(20.0);

   public ObstacleCoursePlanarRegionEnvironment()
   {
      // ground plane
      generator.translate(0.0, 0.0, -0.01);
      generator.addRectangle(15.0, 15.0);

      // staircase and ramps
      generator.identity();
      generator.translate(2.0, 0.0, 0.0);
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

      for (int i = 1; i < numberOfSteps; i++)
      {
         generator.translate(stepLength, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(stepLength, 1.0, (numberOfSteps - i) * stepHeight);
      }

      // ramps
      generator.identity();
      generator.translate(2.0, 1.0, 0.0);
      generator.addRampReferencedAtBottomMiddle((numberOfSteps - 0.5) * stepLength, 1.0, stepHeight * numberOfSteps);
      generator.translate(numberOfSteps * stepLength + 0.5 * startingBlockLength, 0.0, stepHeight * numberOfSteps);
      generator.addRectangle(startingBlockLength + stepLength, startingBlockLength);
      generator.translate(numberOfSteps * stepLength + 0.5 * startingBlockLength, 0.0, -stepHeight * numberOfSteps);
      generator.rotate(Math.PI, Axis.Z);
      generator.addRampReferencedAtBottomMiddle((numberOfSteps - 0.5) * stepLength, 1.0, stepHeight * numberOfSteps);

      // cinder blocks
      generator.identity();
      generator.translate(-2.0, 0.0, -0.05);
      generator.rotate(Math.PI, Axis.Z);
      PlanarRegionsListExamples.generateCinderBlockField(generator, 0.4, 0.1, 9, 9, 0.1);

      // stepping stones
      generator.identity();
      double steppingStoneHeight = 0.3;
      generator.translate(0.0, 2.0, 0.0);
      generator.rotate(0.5 * Math.PI, Axis.Z);
      generator.addRampReferencedAtBottomMiddle(1.0, 1.0, steppingStoneHeight);
      generator.translate(1.4, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.8, 1.0, steppingStoneHeight);

      generator.translate(-0.3, 0.1, 0.0); // shift the stepping stones a little to align with starting block

      generator.translate(0.925, 0.125, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.25, 0.25, steppingStoneHeight);
      generator.translate(-0.925, -0.125, 0.0);

      generator.translate(1.1, -0.375, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.6, 0.25, steppingStoneHeight);
      generator.translate(-1.1, 0.375, 0.0);

      generator.translate(1.35, 0.05, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.2, 0.2, steppingStoneHeight);
      generator.translate(-1.35, -0.05, 0.0);

      generator.translate(1.65, 0.2, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.3, 0.3, steppingStoneHeight);
      generator.translate(-1.65, -0.2, 0.0);

      generator.translate(1.65, -0.175, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.2, 0.2, steppingStoneHeight);
      generator.translate(-1.65, 0.175, 0.0);

      generator.translate(1.975, 0.175, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.25, 0.25, steppingStoneHeight);
      generator.translate(-1.975, -0.175, 0.0);

      generator.translate(2.0, -0.2, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.3, 0.25, steppingStoneHeight);
      generator.translate(-2.0, 0.2, 0.0);

      generator.translate(2.0 + 0.5 * 0.3 + 0.1 + 0.4, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.8, 1.0, steppingStoneHeight);
      generator.translate(0.0, -(0.5 + 1.0), 0.0);
      generator.rotate(0.5 * Math.PI, Axis.Z);
      generator.addRampReferencedAtBottomMiddle(1.0, 1.0, steppingStoneHeight);
   }
}
