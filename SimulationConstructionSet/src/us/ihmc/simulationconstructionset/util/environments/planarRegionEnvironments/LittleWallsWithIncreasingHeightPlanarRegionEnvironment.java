package us.ihmc.simulationconstructionset.util.environments.planarRegionEnvironments;

public class LittleWallsWithIncreasingHeightPlanarRegionEnvironment extends PlanarRegionEnvironmentInterface
{
   public LittleWallsWithIncreasingHeightPlanarRegionEnvironment()
   {
      generator.translate(2.0, 0.0, -0.01);
      generator.addCubeReferencedAtCenter(6.0, 1.0, 0.00005);
      generator.translate(-2.0, 0.0, 0.0);
      generator.translate(0.35, 0.2, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.1);
      generator.translate(0.62, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.14);
      generator.translate(0.3, -0.3, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.15);
      generator.translate(0.4, 0.1, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.1, 1.0, 0.11);
   }
}
