package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class StaircaseEnvironment extends PlanarRegionEnvironmentInterface
{
   public StaircaseEnvironment(int numberOfSteps, double stepHeight, double stepLength)
   {
      this(numberOfSteps, stepHeight, stepLength, false);
   }

   public StaircaseEnvironment(int numberOfSteps, double stepHeight, double stepLength, boolean includeDown)
   {
      this(numberOfSteps, stepHeight, stepLength, 1.75, includeDown);
   }

   public StaircaseEnvironment(int numberOfSteps, double stepHeight, double stepLength, double stairsWidth, boolean includeDown)
   {
      double startingBlockLength = 1.2;
      generator.identity();
      generator.addRectangle(startingBlockLength, stairsWidth);
      generator.translate(0.5 * (startingBlockLength - stepLength), 0.0, 0.0);

      for (int i = 0; i < numberOfSteps; i++)
      {
         generator.translate(stepLength, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(stepLength, stairsWidth, (i + 1) * stepHeight);
      }

      generator.translate(0.5 * (startingBlockLength + stepLength), 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(startingBlockLength, stairsWidth, numberOfSteps * stepHeight);

      if (includeDown)
      {
         generator.translate(0.5 * (startingBlockLength - stepLength), 0.0, 0.0);

         for (int i = 0; i < numberOfSteps; i++)
         {
            generator.translate(stepLength, 0.0, 0.0);
            generator.addCubeReferencedAtBottomMiddle(stepLength, stairsWidth, (numberOfSteps - i) * stepHeight);
         }
      }

      generator.translate(0.5 * (startingBlockLength + stepLength), 0.0, 0.0);
      generator.addRectangle(startingBlockLength, stairsWidth);

      addPlanarRegionsToTerrain(YoAppearance.Grey());
   }
}
