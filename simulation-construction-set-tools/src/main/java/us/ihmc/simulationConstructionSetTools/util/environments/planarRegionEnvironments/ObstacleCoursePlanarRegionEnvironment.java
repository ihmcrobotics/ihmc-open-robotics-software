package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.euclid.Axis3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;

public class ObstacleCoursePlanarRegionEnvironment extends PlanarRegionEnvironmentInterface
{
   public ObstacleCoursePlanarRegionEnvironment()
   {
      // ground plane
      generator.translate(3.25, -2.4, -0.01);
      generator.addRectangle(9.5, 11.25);
      generator.translate(-3.25, 2.4, 0.0);

      generator.translate(3.25, 6.45, 0.0);
      generator.addRectangle(9.5, 3.1);
      generator.translate(-3.25, -6.45, 0.0);

      generator.translate(-1.0, 4.05, 0.0);
      generator.addRectangle(1.0, 1.7);
      generator.translate(1.0, -4.05, 0.0);

      generator.translate(4.25, 4.05, 0.0);
      generator.addRectangle(7.5, 1.7);
      generator.translate(-4.25, -4.05, 0.0);

      generator.translate(-4.75, -4.6, 0.0);
      generator.addRectangle(6.5, 6.8);
      generator.translate(4.75, 4.6, 0.0);

      generator.translate(-4.75, 4.6, 0.0);
      generator.addRectangle(6.5, 6.8);
      generator.translate(4.75, -4.6, 0.0);

      generator.translate(-6.7, 0.0, 0.0);
      generator.addRectangle(2.6, 2.4);
      generator.translate(6.7, 0.0, 0.0);
      addPlanarRegionsToTerrain(YoAppearance.RGBColor(110 / 256.0, 121 / 256.0, 121 / 256.0));

      // staircase and ramps
      generator.identity();
      generator.translate(1.5, 0.0, 0.0);
      double stepHeight = 0.1;
      double stepLength = 0.30;
      int numberOfSteps = 6;
      double startingBlockLength = 1.0;

      for (int i = 0; i < numberOfSteps; i++)
      {
         generator.translate(stepLength, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(stepLength, 1.0, (i + 1) * stepHeight);
      }

      generator.translate(0.5 * (startingBlockLength + stepLength) - 0.5 * stepLength, 0.5 * startingBlockLength, 0.0);
      generator.addCubeReferencedAtBottomMiddle(startingBlockLength + stepLength, 2.0 * startingBlockLength, numberOfSteps * stepHeight);
      generator.translate(0.5 * (startingBlockLength - stepLength), - 0.5 * startingBlockLength, 0.0);

      for (int i = 1; i < numberOfSteps; i++)
      {
         generator.translate(stepLength, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(stepLength, 1.0, (numberOfSteps - i) * stepHeight);
      }

      // ramps
      generator.identity();
      generator.translate(1.5, 1.0, 0.0);
      generator.addRampReferencedAtBottomMiddle((numberOfSteps - 0.5) * stepLength, 1.0, stepHeight * numberOfSteps);
      generator.translate(numberOfSteps * stepLength + 0.5 * startingBlockLength, 0.0, stepHeight * numberOfSteps);
      generator.addRectangle(startingBlockLength + stepLength, startingBlockLength);
      generator.translate(numberOfSteps * stepLength + 0.5 * startingBlockLength, 0.0, -stepHeight * numberOfSteps);
      generator.rotate(Math.PI, Axis3D.Z);
      generator.addRampReferencedAtBottomMiddle((numberOfSteps - 0.5) * stepLength, 1.0, stepHeight * numberOfSteps);

      // cinder blocks
      generator.identity();
      generator.translate(-1.5, 0.0, -0.05);
      generator.rotate(Math.PI, Axis3D.Z);
      PlanarRegionsListExamples.generateCinderBlockField(generator, 0.4, 0.1, 9, 6, 0.05);

      // stepping stones
      generator.identity();
      double steppingStoneHeight = 0.3;
      generator.translate(0.0, 1.5, 0.0);
      generator.rotate(0.5 * Math.PI, Axis3D.Z);
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

      generator.translate(1.25, 0.05, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.3, 0.2, steppingStoneHeight);
      generator.translate(-1.25, -0.05, 0.0);

      generator.translate(1.65, 0.2, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.3, 0.3, steppingStoneHeight);
      generator.translate(-1.65, -0.2, 0.0);

      generator.translate(1.65, -0.275, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.2, 0.2, steppingStoneHeight);
      generator.translate(-1.65, 0.275, 0.0);

      generator.translate(1.975, 0.075, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.25, 0.35, steppingStoneHeight);
      generator.translate(-1.975, -0.075, 0.0);

      generator.translate(2.0, -0.35, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.35, 0.35, steppingStoneHeight);
      generator.translate(-2.0, 0.35, 0.0);

      generator.translate(2.0 + 0.5 * 0.3 + 0.1 + 0.6, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(1.2, 1.0, steppingStoneHeight);
      generator.translate(0.0, -(0.5 + 1.0), 0.0);
      generator.rotate(0.5 * Math.PI, Axis3D.Z);
      generator.addRampReferencedAtBottomMiddle(1.0, 1.2, steppingStoneHeight);

      addPlanarRegionsToTerrain(YoAppearance.Gray());
   }
}
