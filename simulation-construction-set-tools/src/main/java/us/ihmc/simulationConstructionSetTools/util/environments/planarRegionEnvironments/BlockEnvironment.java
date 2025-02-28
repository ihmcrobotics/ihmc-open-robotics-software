package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class BlockEnvironment extends PlanarRegionEnvironmentInterface
{
   public BlockEnvironment()
   {
      // first ground plane
      generator.identity();
      generator.addRectangle(10.0, 10.0);

      generator.translate(0.15, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.3, 0.3, 0.09);

      addPlanarRegionsToTerrain(YoAppearance.Grey());
   }

   public BlockEnvironment(double blockWidth, double blockLength, double blockHeight)
   {
      // first ground plane
      generator.identity();
      generator.addRectangle(10.0, 10.0);

      generator.addCubeReferencedAtBottomMiddle(blockWidth, blockLength, blockHeight);

      addPlanarRegionsToTerrain(YoAppearance.Grey());
   }
}
