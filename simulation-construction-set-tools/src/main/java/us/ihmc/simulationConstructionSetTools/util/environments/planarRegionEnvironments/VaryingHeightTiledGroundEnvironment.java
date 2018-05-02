package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;

import java.util.Random;

public class VaryingHeightTiledGroundEnvironment extends PlanarRegionEnvironmentInterface
{
   private final Random random = new Random(2390L);

   public VaryingHeightTiledGroundEnvironment(double tileWidth, int courseLengthInNumberOfTiles, int courseWidthInNumberOfTiles, double nominalGroundHeight, double maxHeightVariation)
   {
      generator.identity();
      double startCoordinateX = - 0.5 * tileWidth * (courseLengthInNumberOfTiles - 1);
      double startCoordinateY = - 0.5 * tileWidth * (courseWidthInNumberOfTiles- 1);

      for (int i = 0; i < courseLengthInNumberOfTiles; i++)
      {
         for (int j = 0; j < courseWidthInNumberOfTiles; j++)
         {
            double tilePositionX = startCoordinateX + tileWidth * i;
            double tilePositionY = startCoordinateY + tileWidth * j;
            double tilePositionZ = nominalGroundHeight + EuclidCoreRandomTools.nextDouble(random, 0.5 * maxHeightVariation);

            generator.translate(tilePositionX, tilePositionY, tilePositionZ);
            generator.addRectangle(tileWidth, tileWidth);
            generator.identity();
         }
      }
   }
}
