package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class LittleWallsWithIncreasingHeightPlanarRegionEnvironment extends PlanarRegionEnvironmentInterface
{
   public LittleWallsWithIncreasingHeightPlanarRegionEnvironment()
   {
      this(true);
   }

   public LittleWallsWithIncreasingHeightPlanarRegionEnvironment(boolean drawGround)
   {
      generator.translate(2.0, 0.0, -0.01);
      if (drawGround)
         generator.addCubeReferencedAtBottomMiddle(6.0, 1.0, 0.01);
      generator.translate(-2.0, 0.0, 0.0);
      generator.translate(0.35, 0.2, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.1);
      generator.translate(0.63, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.14);
      generator.translate(0.3, -0.3, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.15);
      generator.translate(0.4, 0.1, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 1.0, 0.11);
      addPlanarRegionsToTerrain(YoAppearance.Grey());
   }
}
