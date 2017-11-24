package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;

public class CinderBlockFieldPlanarRegionEnvironment extends PlanarRegionEnvironmentInterface
{
   private enum BlockOrientation {NORTH, SOUTH, EAST, WEST}

   private static final double cinderBlockTiltDegrees = 15;
   private static final double cinderBlockTiltRadians = Math.toRadians(cinderBlockTiltDegrees);

   private static final double cinderBlockLength = 0.40; // 40 cm (approx 16 in, just less than 16in)
   private static final double flatCinderBlockHeight = 0.08; // 15 cm (approx 6 in, less than 6 in, but consistent with other cm measurements)
   private static final double cinderBlockHeight = 0.15; // 15 cm (approx 6 in, less than 6 in, but consistent with other cm measurements)

   private static final double distanceToFirstCinderBlock = 0.5;
   private static final double gapBetweenCinderBlockSets = 1.0;

   public CinderBlockFieldPlanarRegionEnvironment()
   {
      generator.translate(0.0, 0.0, -0.01);
      generator.addCubeReferencedAtBottomMiddle(2.0 * distanceToFirstCinderBlock, 4 * cinderBlockLength, 0.01);
      generator.identity();

      double xPosition = distanceToFirstCinderBlock + 0.5 * cinderBlockLength;

      setUpFlatBlock(generator, xPosition, 0.2, 0.0, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, 0.6, 0.0, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, -0.2, 0.0, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, -0.6, 0.0, flatCinderBlockHeight);

      xPosition += cinderBlockLength;

      setUpFlatBlock(generator, xPosition, -0.6, flatCinderBlockHeight, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, -0.2, flatCinderBlockHeight, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, 0.2, -flatCinderBlockHeight, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, 0.6, flatCinderBlockHeight, flatCinderBlockHeight);

      xPosition += cinderBlockLength;

      setUpFlatBlock(generator, xPosition, -0.6, 2.0 * flatCinderBlockHeight, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, -0.2, flatCinderBlockHeight, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, 0.2, 0.0, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, 0.6, flatCinderBlockHeight, flatCinderBlockHeight);

      xPosition += cinderBlockLength;

      setUpFlatBlock(generator, xPosition, -0.6, flatCinderBlockHeight, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, -0.2, 2.0 * flatCinderBlockHeight, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, 0.2, flatCinderBlockHeight, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, 0.6, 0.0, flatCinderBlockHeight);

      xPosition += cinderBlockLength;

      setUpFlatBlock(generator, xPosition, -0.6, 0.0, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, -0.2, flatCinderBlockHeight, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, 0.2, 0.0, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, 0.6, -flatCinderBlockHeight, flatCinderBlockHeight);

      xPosition += cinderBlockLength;

      setUpFlatBlock(generator, xPosition, -0.6, -flatCinderBlockHeight, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, -0.2, 0.0, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, 0.2, -flatCinderBlockHeight, flatCinderBlockHeight);
      setUpFlatBlock(generator, xPosition, 0.6, -flatCinderBlockHeight, flatCinderBlockHeight);



      xPosition += 0.5 * (gapBetweenCinderBlockSets + cinderBlockLength);

      generator.translate(xPosition, 0.0, -0.01);
      generator.addCubeReferencedAtBottomMiddle(gapBetweenCinderBlockSets, 4.0 * cinderBlockLength, 0.01);
      generator.identity();

      xPosition += 0.5 * (gapBetweenCinderBlockSets + cinderBlockLength);




      setUpRampBlock(generator, xPosition, -0.6, 0.0, BlockOrientation.NORTH);
      setUpRampBlock(generator, xPosition, -0.2, 0.0, BlockOrientation.NORTH);
      setUpRampBlock(generator, xPosition, 0.2, 0.0, BlockOrientation.NORTH);
      setUpRampBlock(generator, xPosition, 0.6, 0.0, BlockOrientation.NORTH);

      xPosition += cinderBlockLength;

      setUpRampBlock(generator, xPosition, -0.6, 0.0, BlockOrientation.SOUTH);
      setUpRampBlock(generator, xPosition, -0.2, 0.0, BlockOrientation.SOUTH);
      setUpRampBlock(generator, xPosition, 0.2, 0.0, BlockOrientation.SOUTH);
      setUpRampBlock(generator, xPosition, 0.6, 0.0, BlockOrientation.SOUTH);

      xPosition += cinderBlockLength;

      setUpRampBlock(generator, xPosition, -0.6, 0.0, BlockOrientation.EAST);
      setUpRampBlock(generator, xPosition, -0.2, 0.0, BlockOrientation.WEST);
      setUpRampBlock(generator, xPosition, 0.2, 0.0, BlockOrientation.EAST);
      setUpRampBlock(generator, xPosition, 0.6, 0.0, BlockOrientation.WEST);

      xPosition += cinderBlockLength;

      setUpRampBlock(generator, xPosition, -0.6, 0.0, BlockOrientation.WEST);
      setUpRampBlock(generator, xPosition, -0.2, 0.0, BlockOrientation.EAST);
      setUpRampBlock(generator, xPosition, 0.2, 0.0, BlockOrientation.WEST);
      setUpRampBlock(generator, xPosition, 0.6, 0.0, BlockOrientation.EAST);



      xPosition += cinderBlockLength;

      setUpFlatBlock(generator, xPosition, -0.6, cinderBlockHeight);
      setUpFlatBlock(generator, xPosition, -0.2, cinderBlockHeight);
      setUpFlatBlock(generator, xPosition, 0.2, cinderBlockHeight);
      setUpFlatBlock(generator, xPosition, 0.6, cinderBlockHeight);

      xPosition += cinderBlockLength;

      setUpRampBlock(generator, xPosition, -0.6, 0.0, BlockOrientation.NORTH);
      setUpRampBlock(generator, xPosition, -0.2, 0.0, BlockOrientation.EAST);
      setUpRampBlock(generator, xPosition, 0.2, 0.0, BlockOrientation.SOUTH);
      setUpRampBlock(generator, xPosition, 0.6, 0.0, BlockOrientation.WEST);

      xPosition += cinderBlockLength;

      setUpRampBlock(generator, xPosition, -0.6, cinderBlockHeight, BlockOrientation.EAST);
      setUpRampBlock(generator, xPosition, -0.2, cinderBlockHeight, BlockOrientation.SOUTH);
      setUpRampBlock(generator, xPosition, 0.2, cinderBlockHeight, BlockOrientation.WEST);
      setUpRampBlock(generator, xPosition, 0.6, cinderBlockHeight, BlockOrientation.NORTH);

      xPosition += cinderBlockLength;

      setUpRampBlock(generator, xPosition, -0.6, 0.0, BlockOrientation.SOUTH);
      setUpRampBlock(generator, xPosition, -0.2, 0.0, BlockOrientation.WEST);
      setUpRampBlock(generator, xPosition, 0.2, 0.0, BlockOrientation.NORTH);
      setUpRampBlock(generator, xPosition, 0.6, 0.0, BlockOrientation.EAST);

      xPosition += 0.75 + 0.5 * cinderBlockLength;
      generator.translate(xPosition, 0.0, -0.01);
      generator.addCubeReferencedAtBottomMiddle(1.5, 4.0 * cinderBlockLength, 0.01);
      generator.identity();
   }

   private static void setUpFlatBlock(PlanarRegionsListGenerator generator, double xInWorld, double yInWorld, double bottomSurfaceHeightLowestPoint)
   {
      setUpFlatBlock(generator, xInWorld, yInWorld, bottomSurfaceHeightLowestPoint, cinderBlockHeight);
   }

   private static void setUpFlatBlock(PlanarRegionsListGenerator generator, double xInWorld, double yInWorld, double bottomSurfaceHeightLowestPoint, double blockHeight)
   {
      generator.translate(xInWorld, yInWorld, bottomSurfaceHeightLowestPoint);

      generator.addCubeReferencedAtBottomMiddle(cinderBlockLength, cinderBlockLength, blockHeight);

      generator.identity();
   }

   private static void setUpRampBlock(PlanarRegionsListGenerator generator, double xInWorld, double yInWorld, double surfaceHeightLowestPoint,
                                      BlockOrientation orientation)
   {
      double zHeightToCenter = 0.5 * cinderBlockLength * Math.sin(cinderBlockTiltRadians);

      generator.translate(xInWorld, yInWorld, surfaceHeightLowestPoint + zHeightToCenter);

      Quaternion quaternion = new Quaternion();
      switch (orientation)
      {
      case NORTH:
         quaternion.appendPitchRotation(-cinderBlockTiltRadians);
         break;
      case EAST:
         quaternion.appendRollRotation(-cinderBlockTiltRadians);
         break;
      case WEST:
         quaternion.appendRollRotation(cinderBlockTiltRadians);
         break;
      case SOUTH:
         quaternion.appendPitchRotation(cinderBlockTiltRadians);
         break;
      }
      generator.rotate(quaternion);

      generator.addCubeReferencedAtBottomMiddle(cinderBlockLength, cinderBlockLength, cinderBlockHeight);

      switch (orientation)
      {
      case NORTH:
         quaternion.appendPitchRotation(cinderBlockTiltRadians);
         break;
      case EAST:
         quaternion.appendRollRotation(cinderBlockTiltRadians);
         break;
      case WEST:
         quaternion.appendRollRotation(-cinderBlockTiltRadians);
         break;
      case SOUTH:
         quaternion.appendPitchRotation(-cinderBlockTiltRadians);
         break;
      }
      generator.identity();
   }

}
