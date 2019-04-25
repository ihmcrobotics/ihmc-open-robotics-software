package us.ihmc.simulationConstructionSetTools.util.planarRegions;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.util.Random;

public class PlanarRegionsListExamples
{
   public static PlanarRegionsList generateFlatGround(double lengthX, double widthY)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.addCubeReferencedAtBottomMiddle(lengthX, widthY, 0.001);
      PlanarRegionsList flatGround = generator.getPlanarRegionsList();
      return flatGround;
   }

   public static PlanarRegionsList generateStairCase()
   {
      return generateStairCase(new Vector3D(), new Vector3D());
   }

   public static PlanarRegionsList generateStairCase(Vector3D rotationVector)
   {
      return generateStairCase(new Vector3D(), rotationVector);
   }

   public static PlanarRegionsList generateStairCase(Vector3D translationVector, Vector3D rotationVector)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(translationVector);

      int numberOfSteps = 5;

      double length = 0.4;
      double width = 0.8;
      double height = 0.1;

      generator.translate(length * numberOfSteps / 2.0, 0.0, 0.001);
      generator.addRectangle(1.2 * length * numberOfSteps, 1.2 * width);

      generator.identity();
      generator.translate(translationVector);
      generator.translate(length, 0.0, 0.0);
      generator.rotateEuler(rotationVector);
      for (int i = 0; i < numberOfSteps; i++)
      {
         generator.addCubeReferencedAtBottomMiddle(length, width, height);
         generator.translate(length, 0.0, 0.0);
         height = height + 0.1;
      }

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      return planarRegionsList;
   }

   public static void generateCinderBlockField(PlanarRegionsListGenerator generator, double cinderBlockSize, double cinderBlockHeight,
                                               int courseWidthXInNumberOfBlocks, int courseLengthYInNumberOfBlocks, double heightVariation,
                                               double extrusionLength, double startingBlockLength)
   {
      double defaultTiltAngle = Math.toRadians(15.0);
      double randomHeightVariation = 0.0;
      boolean onlyGenerateTopOfBlock = false;

      generateCinderBlockField(generator,
                               cinderBlockSize,
                               cinderBlockHeight,
                               courseWidthXInNumberOfBlocks,
                               courseLengthYInNumberOfBlocks,
                               heightVariation,
                               extrusionLength,
                               startingBlockLength,
                               0.0,
                               defaultTiltAngle,
                               defaultTiltAngle,
                               randomHeightVariation,
                               onlyGenerateTopOfBlock);
   }

   public static void generateCinderBlockField(PlanarRegionsListGenerator generator, double cinderBlockSize, double cinderBlockHeight,
                                               int courseWidthXInNumberOfBlocks, int courseLengthYInNumberOfBlocks, double heightVariation,
                                               double extrusionLength, double startingBlockLength, double absentBlockPercentage, double minTiltAngle,
                                               double maxTiltAngle, double randomHeightVariation, boolean onlyGenerateTopOfBlock)
   {
      double courseWidth = courseLengthYInNumberOfBlocks * cinderBlockSize;

      generator.addRectangle(startingBlockLength + extrusionLength, courseWidth + extrusionLength); // standing platform
      generator.translate(0.5 * startingBlockLength + 0.5 * cinderBlockSize, 0.0, 0.0); // forward to first row
      generator.translate(0.0, -0.5 * (courseLengthYInNumberOfBlocks - 1) * cinderBlockSize, 0.0); // over to grid origin

      Random random = new Random(1231239L);
      for (int x = 0; x < courseWidthXInNumberOfBlocks; x++)
      {
         for (int y = 0; y < courseLengthYInNumberOfBlocks; y++)
         {
            int angleType = Math.abs(random.nextInt() % 3);
            int axisType = Math.abs(random.nextInt() % 2);

            double extraHeightVariation = EuclidCoreRandomTools.nextDouble(random, randomHeightVariation);
            generator.translate(0.0, 0.0, extraHeightVariation);

            double tiltAngle = EuclidCoreRandomTools.nextDouble(random, minTiltAngle, maxTiltAngle);
            if (random.nextDouble() > absentBlockPercentage)
               generateSingleCinderBlock(generator,
                                         cinderBlockSize + extrusionLength,
                                         cinderBlockHeight + extrusionLength,
                                         angleType,
                                         axisType,
                                         tiltAngle,
                                         onlyGenerateTopOfBlock);

            generator.translate(0.0, 0.0, -extraHeightVariation);
            generator.translate(0.0, cinderBlockSize, 0.0);
         }

         if ((x / 2) % 2 == 0)
         {
            generator.translate(0.0, 0.0, heightVariation);
         }
         else
         {
            generator.translate(0.0, 0.0, -heightVariation);
         }

         generator.translate(cinderBlockSize, -cinderBlockSize * courseLengthYInNumberOfBlocks, 0.0);
      }

      generator.translate(0.5 * startingBlockLength - 0.5 * cinderBlockSize, 0.0, 0.0); // forward to platform middle
      generator.translate(0.0, 0.5 * (courseLengthYInNumberOfBlocks - 1) * cinderBlockSize, 0.0); // over to grid middle
      generator.addRectangle(startingBlockLength + extrusionLength, courseWidth + extrusionLength);
   }

   public static void generateCinderBlockField(PlanarRegionsListGenerator generator, double cinderBlockSize, double cinderBlockHeight,
                                               int courseWidthXInNumberOfBlocks, int courseLengthYInNumberOfBlocks, double heightVariation)
   {
      generateCinderBlockField(generator,
                               cinderBlockSize,
                               cinderBlockHeight,
                               courseWidthXInNumberOfBlocks,
                               courseLengthYInNumberOfBlocks,
                               heightVariation,
                               0.0,
                               0.6);
   }

   public static void generateCinderBlockSlope(PlanarRegionsListGenerator generator,
                                               Random random,
                                               double cinderBlockSurfaceSquareSize,
                                               double cinderBlockThickness,
                                               int courseLengthXNumberOfBlocks,
                                               int courseWidthYNumberOfBlocks,
                                               double zStepUpPerRow,
                                               double tiltedBlockPercentage,
                                               double absentBlockPercentage,
                                               double tiltAngle,
                                               double randomHeightVariation
   )
   {
      for (int x = 0; x < courseLengthXNumberOfBlocks; x++)
      {
         for (int y = 0; y < courseWidthYNumberOfBlocks; y++)
         {
            int angleType = Math.abs(random.nextInt() % 2);
            if (angleType == 1) angleType = 2;
            int axisType = Math.abs(random.nextInt() % 2);

            double randomHeightOccurrence = EuclidCoreRandomTools.nextDouble(random, randomHeightVariation);
            generator.translate(0.0, 0.0, randomHeightOccurrence);

            boolean tilt = random.nextDouble() > tiltedBlockPercentage;
            double tiltAngleOccurrence = tilt ? tiltAngle : 0.0;
            if (random.nextDouble() > absentBlockPercentage)
               generateSingleCinderBlockOrigin(generator,
                                               cinderBlockSurfaceSquareSize,
                                               cinderBlockThickness,
                                               angleType,
                                               axisType,
                                               tiltAngleOccurrence);

            generator.translate(0.0, 0.0, -randomHeightOccurrence);

            generator.translate(0.0, cinderBlockSurfaceSquareSize, 0.0);
         }

         generator.translate(0.0, 0.0, zStepUpPerRow);

         generator.translate(cinderBlockSurfaceSquareSize, -cinderBlockSurfaceSquareSize * courseWidthYNumberOfBlocks, 0.0);
      }
      generator.translate(-cinderBlockSurfaceSquareSize * courseLengthXNumberOfBlocks, 0.0, -zStepUpPerRow * courseLengthXNumberOfBlocks);
   }

   public static void generateCinderBlockCornerSlope(PlanarRegionsListGenerator generator,
                                                     Random random,
                                                     double cinderBlockSurfaceSquareSize,
                                                     double cinderBlockThickness,
                                                     int courseLengthXNumberOfBlocks,
                                                     int courseWidthYNumberOfBlocks,
                                                     double zStepUpPerRow,
                                                     double tiltedBlockPercentage,
                                                     double absentBlockPercentage,
                                                     double tiltAngle,
                                                     double randomHeightVariation
   )
   {
      for (int x = 0; x < courseLengthXNumberOfBlocks; x++)
      {
         for (int y = 0; y < courseWidthYNumberOfBlocks; y++)
         {
            int angleType = Math.abs(random.nextInt() % 2);
            if (angleType == 1) angleType = 2;
            int axisType = Math.abs(random.nextInt() % 2);

            double randomHeightOccurrence = EuclidCoreRandomTools.nextDouble(random, randomHeightVariation);
            generator.translate(0.0, 0.0, randomHeightOccurrence);

            boolean tilt = random.nextDouble() > tiltedBlockPercentage;
            double tiltAngleOccurrence = tilt ? tiltAngle : 0.0;
            if (random.nextDouble() > absentBlockPercentage)
               generateSingleCinderBlockOrigin(generator,
                                               cinderBlockSurfaceSquareSize,
                                               cinderBlockThickness,
                                               angleType,
                                               axisType,
                                               tiltAngleOccurrence);

            generator.translate(0.0, 0.0, -randomHeightOccurrence);

            generator.translate(0.0, cinderBlockSurfaceSquareSize, 0.0);

            if ((courseWidthYNumberOfBlocks - y) < x) // <-- complicated function to get the corner pile
            {
               generator.translate(0.0, 0.0, zStepUpPerRow);
            }
         }

         generator.translate(0.0, 0.0, -x * zStepUpPerRow);

         generator.translate(cinderBlockSurfaceSquareSize, -cinderBlockSurfaceSquareSize * courseWidthYNumberOfBlocks, 0.0);
      }
      generator.translate(-cinderBlockSurfaceSquareSize * courseLengthXNumberOfBlocks, 0.0, 0.0);
   }

   public static PlanarRegionsList generateSteppingStoneField(double steppingStoneWidth, double steppingStoneLength, double stepWidth, double stepLength,
                                                              int numberOfSteps)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double platformLength = 0.6;
      double platformWidth = 1.0;

      generator.addRectangle(platformLength, platformWidth);
      generator.translate(0.5 * platformLength, 0.0, 0.0);

      for (int i = 0; i < numberOfSteps; i++)
      {
         RobotSide side = (i % 2 == 0) ? RobotSide.LEFT : RobotSide.RIGHT;
         double xOffset = stepLength;
         double yOffset = side.negateIfRightSide(0.5 * stepWidth);
         generator.translate(xOffset, yOffset, 0.0);
         generator.addRectangle(steppingStoneLength, steppingStoneWidth);
      }

      generator.translate(stepLength + 0.5 * platformLength, 0.0, 0.0);
      generator.addRectangle(platformLength, platformWidth);
      return generator.getPlanarRegionsList();
   }

   public static void generateSingleCinderBlock(PlanarRegionsListGenerator generator, double cinderBlockSize, double cinderBlockHeight, int angleType,
                                                int axisType)
   {
      double defaultTiltAngle = Math.toRadians(15.0);
      boolean onlyGenerateTopOfBlock = false;
      generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, angleType, axisType, defaultTiltAngle, onlyGenerateTopOfBlock);
   }

   public static void generateSingleCinderBlock(PlanarRegionsListGenerator generator, double cinderBlockSize, double cinderBlockHeight, int angleType,
                                                int axisType, double tiltAngle, boolean onlyGenerateTopOfBlock)
   {
      double angle = 0;
      switch (angleType)
      {
      case 0:
         angle = 0.0;
         break;
      case 1:
         angle = tiltAngle;
         break;
      case 2:
         angle = -tiltAngle;
         break;
      }

      Axis axis = null;
      switch (axisType)
      {
      case 0:
         axis = Axis.X;
         break;
      case 1:
         axis = Axis.Y;
         break;
      }

      generator.rotate(angle, axis);
      if (onlyGenerateTopOfBlock)
         generator.addRectangle(cinderBlockSize, cinderBlockSize);
      else
         generator.addCubeReferencedAtBottomMiddle(cinderBlockSize, cinderBlockSize, cinderBlockHeight);
      generator.rotate(-angle, axis);
   }

   public static void generateSingleCinderBlockOrigin(PlanarRegionsListGenerator generator,
                                                      double cinderBlockSize,
                                                      double cinderBlockHeight,
                                                      int angleType,
                                                      int axisType,
                                                      double tiltAngle)
   {

      double angle = 0;
      switch (angleType)
      {
      case 0:
         angle = 0.0;
         break;
      case 1:
         angle = tiltAngle;
         break;
      case 2:
         angle = -tiltAngle;
         break;
      }

      Axis axis = null;
      switch (axisType)
      {
      case 0:
         axis = Axis.X;
         break;
      case 1:
         axis = Axis.Y;
         break;
      }

      double halfCinderBlockSize = cinderBlockSize / 2;
      double additionalHeightForAngledCinderBlock = halfCinderBlockSize * Math.sin(Math.abs(angle));
      LogTools.info("tile additional hieght: {} ", additionalHeightForAngledCinderBlock);
      generator.translate(halfCinderBlockSize, halfCinderBlockSize, additionalHeightForAngledCinderBlock);

      generator.rotate(angle, axis);
      generator.addCubeReferencedAtBottomMiddle(cinderBlockSize, cinderBlockSize, cinderBlockHeight);
      generator.rotate(-angle, axis);

      generator.translate(-halfCinderBlockSize, -halfCinderBlockSize, -additionalHeightForAngledCinderBlock);
   }

   public static PlanarRegionsList generateRandomObjects(Random random, int numberOfRandomObjects, double maxX, double maxY, double maxZ)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double length = RandomNumbers.nextDouble(random, 0.2, 1.0);
      double width = RandomNumbers.nextDouble(random, 0.2, 1.0);
      double height = RandomNumbers.nextDouble(random, 0.2, 1.0);

      for (int i = 0; i < numberOfRandomObjects; i++)
      {
         generator.identity();

         Vector3D translationVector = RandomGeometry.nextVector3D(random, -maxX, -maxY, 0.0, maxX, maxY, maxZ);
         generator.translate(translationVector);

         Quaternion rotation = RandomGeometry.nextQuaternion(random);
         generator.rotate(rotation);

         generator.addCubeReferencedAtBottomMiddle(length, width, height);
      }

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      return planarRegionsList;
   }

   public static PlanarRegionsList generateBumpyGround(Random random, double maxX, double maxY, double maxZ)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double length = 0.5;
      double width = 0.5;

      generator.translate(maxX / 2.0 + length / 2.0, maxY / 2.0 - width / 2.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(1.5 * maxX, 1.25 * maxY, 0.01);
      generator.identity();

      int sizeX = (int) (maxX / length);
      int sizeY = (int) (maxY / width);

      for (int i = 0; i < sizeY; i++)
      {
         generator.identity();
         generator.translate(0.0, i * width, 0.0);
         for (int j = 0; j < sizeX; j++)
         {
            generator.translate(length, 0.0, 0.0);
            double height = RandomNumbers.nextDouble(random, 0.01, maxZ);
            generator.addCubeReferencedAtBottomMiddle(length, width, height + random.nextDouble() * 0.1);
         }
      }

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      return planarRegionsList;
   }

   public static ConvexPolygon2D createRectanglePolygon(double lengthX, double widthY)
   {
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(lengthX / 2.0, widthY / 2.0);
      convexPolygon.addVertex(-lengthX / 2.0, widthY / 2.0);
      convexPolygon.addVertex(-lengthX / 2.0, -widthY / 2.0);
      convexPolygon.addVertex(lengthX / 2.0, -widthY / 2.0);
      convexPolygon.update();
      return convexPolygon;
   }

   public static PlanarRegionsList generateSteppingStonesEnvironment(double pathRadius)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      double cinderBlockWidth = 0.25;
      double cinderBlockLength = 0.35;
      double cinderBlockSeparationWidth = 0.25;
      double cinderBlockSeparationLength = 0.35;

      double quarterCircleLength = 0.5 * Math.PI * pathRadius;
      int numberOfSteps = (int) Math.round(quarterCircleLength / cinderBlockSeparationLength);
      if (numberOfSteps % 2 != 1)
         numberOfSteps++;

      // starting block
      generator.translate(0.0, -0.5, 0.001);
      generator.addRectangle(2.0, 1.0);
      generator.identity();

      // ending block
      generator.translate(pathRadius + 0.5, pathRadius, 0.001);
      generator.addRectangle(1.0, 2.0);
      generator.identity();

      // cinder blocks
      for (int i = 1; i < numberOfSteps; i++)
      {
         double percentageAlongCurve = ((double) i / (double) numberOfSteps);

         double angle = percentageAlongCurve * 0.5 * Math.PI;
         double xPositionAlongCurve = pathRadius * (1.0 - Math.cos(angle));
         double yPositionAlongCurve = pathRadius * Math.sin(angle);

         generator.translate(xPositionAlongCurve, yPositionAlongCurve, -0.001);
         generator.rotate(-angle, Axis.Z);

         double xTranslation = cinderBlockSeparationWidth * 0.5;
         if (i % 2 == 0)
            xTranslation *= -1.0;
         generator.translate(xTranslation, 0.0, 0.0);

         generator.addRectangle(cinderBlockWidth, cinderBlockLength);
         generator.identity();
      }

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList createMazeEnvironment()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      double extrusionDistance = -0.05;

      // main ground plane
      generator.translate(5.0, 5.0, 0.0);
      generator.addRectangle(10.0, 10.0);
      generator.identity();

      // maze walls along the yz-plane
      generator.translate(2.0, 5.0, 1.0);
      generator.rotate(0.5 * Math.PI, Axis.Y);
      generator.addRectangle(2.0 + extrusionDistance, 6.0 + extrusionDistance);
      generator.identity();

      generator.translate(4.0, 4.0, 1.0);
      generator.rotate(0.5 * Math.PI, Axis.Y);
      generator.addRectangle(2.0 + extrusionDistance, 4.0 + extrusionDistance);
      generator.identity();

      generator.translate(6.0, 6.0, 1.0);
      generator.rotate(0.5 * Math.PI, Axis.Y);
      generator.addRectangle(2.0 + extrusionDistance, 4.0 + extrusionDistance);
      generator.identity();

      generator.translate(8.0, 4.0, 1.0);
      generator.rotate(0.5 * Math.PI, Axis.Y);
      generator.addRectangle(2.0 + extrusionDistance, 4.0 + extrusionDistance);
      generator.identity();

      generator.translate(8.0, 9.0, 1.0);
      generator.rotate(0.5 * Math.PI, Axis.Y);
      generator.addRectangle(2.0 + extrusionDistance, 2.0 + extrusionDistance);
      generator.identity();

      // maze walls along the xz-plane
      generator.translate(6.0, 2.0, 1.0);
      generator.rotate(0.5 * Math.PI, Axis.X);
      generator.addRectangle(4.0 + extrusionDistance, 2.0 + extrusionDistance);
      generator.identity();

      generator.translate(9.0, 6.0, 1.0);
      generator.rotate(0.5 * Math.PI, Axis.X);
      generator.addRectangle(2.0 + extrusionDistance, 2.0 + extrusionDistance);
      generator.identity();

      generator.translate(5.0, 8.0, 1.0);
      generator.rotate(0.5 * Math.PI, Axis.X);
      generator.addRectangle(6.0 + extrusionDistance, 2.0 + extrusionDistance);
      generator.identity();

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList createBodyPathPlannerTestEnvironment()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      double extrusionDistance = -0.05;

      // starting plane
      generator.translate(1.0, 0.5, 0.0);
      generator.addRectangle(2.0 + extrusionDistance, 1.0 + extrusionDistance);
      generator.identity();

      // long plane on the start side
      generator.translate(2.5, -3.5, 0.0);
      generator.addRectangle(1.0 + extrusionDistance, 13.0 + extrusionDistance);
      generator.identity();

      // narrow passage
      double wallSeparation = 0.4;
      double wallWidth = 0.5;
      double wallHeight = 1.0;

      generator.translate(4.5, 2.5, 0.0);
      generator.addRectangle(3.0 + extrusionDistance, 1.0 + extrusionDistance);
      generator.rotate(0.5 * Math.PI, Axis.Y);
      generator.translate(-0.5 * wallHeight, 0.5 * (wallSeparation + wallWidth), 0.0);
      generator.addRectangle(wallHeight, wallWidth);
      generator.translate(0.0, -2.0 * 0.5 * (wallSeparation + wallWidth), 0.0);
      generator.addRectangle(wallHeight, wallWidth);
      generator.identity();

      // high-sloped ramp
      generator.translate(3.5, 0.5, 0.5);
      generator.rotate(-0.25 * Math.PI, Axis.Y);
      generator.addRectangle(Math.sqrt(2.0), 1.0);
      generator.identity();
      generator.translate(4.5, 0.5, 1.0);
      generator.addRectangle(1.0 + extrusionDistance, 1.0 + extrusionDistance);
      generator.identity();
      generator.translate(5.5, 0.5, 0.5);
      generator.rotate(0.25 * Math.PI, Axis.Y);
      generator.addRectangle(Math.sqrt(2.0), 1.0);
      generator.identity();

      // large step down
      double stepDownHeight = 0.4;
      generator.translate(3.5, -1.5, 0.0);
      generator.addRectangle(1.0 + extrusionDistance, 1.0 + extrusionDistance);
      generator.translate(1.0, 0.0, -stepDownHeight);
      generator.addRectangle(1.0 + extrusionDistance, 1.0 + extrusionDistance);
      generator.translate(1.0, 0.0, stepDownHeight);
      generator.addRectangle(1.0 + extrusionDistance, 1.0 + extrusionDistance);
      generator.identity();

      // large step up
      double stepUpHeight = 0.4;
      generator.translate(3.5, -3.5, 0.0);
      generator.addRectangle(1.0 + extrusionDistance, 1.0 + extrusionDistance);
      generator.translate(1.0, 0.0, stepUpHeight);
      generator.addRectangle(1.0 + extrusionDistance, 1.0 + extrusionDistance);
      generator.translate(1.0, 0.0, -stepUpHeight);
      generator.addRectangle(1.0 + extrusionDistance, 1.0 + extrusionDistance);
      generator.identity();

      // barrier
      double barrierHeight = 1.5;
      double barrierWidth = 0.8;

      generator.translate(4.5, -5.5, 0.0);
      generator.addRectangle(3.0 + extrusionDistance, 1.0 + extrusionDistance);
      generator.translate(0.0, 0.0, 0.5 * barrierHeight);
      generator.rotate(0.5 * Math.PI, Axis.Y);
      generator.addRectangle(barrierHeight, barrierWidth);
      generator.identity();

      // long gap
      generator.translate(3.5, -7.5, 0.0);
      generator.addRectangle(1.0 + extrusionDistance, 1.0 + extrusionDistance);
      generator.translate(2.0, 0.0, 0.0);
      generator.addRectangle(1.0 + extrusionDistance, 1.0 + extrusionDistance);
      generator.identity();

      // long plane on the goal side
      generator.translate(6.5, -3.5, 0.01);
      generator.addRectangle(1.0 + extrusionDistance, 13.0 + extrusionDistance);
      generator.identity();

      // goal plane
      generator.translate(8.0, -3.5, 0.01);
      generator.addRectangle(2.0 + extrusionDistance, 1.0 + extrusionDistance);
      generator.identity();

      PlanarRegionsList obstacleCourse = generator.getPlanarRegionsList();

      // overhang, wide barrier, and stepping stones
      generator.translate(4.5, -9.5, 2.5);
      generator.addRectangle(1.5, 0.8);
      generator.identity();

      wallSeparation = 0.9;
      wallWidth = 0.2;
      wallHeight = 1.0;

      generator.translate(3.0, -9.5, 0.0);
      generator.rotate(0.5 * Math.PI, Axis.Y);
      generator.translate(-0.5 * wallHeight, 0.5 * (wallSeparation + wallWidth), 0.0);
      generator.addRectangle(wallHeight, wallWidth);
      generator.translate(0.0, -2.0 * 0.5 * (wallSeparation + wallWidth), 0.0);
      generator.addRectangle(wallHeight, wallWidth);
      generator.identity();

      generator.translate(3.0, -9.5, 0.0);
      generateCinderBlockField(generator, 0.25, 0.2, 11, 4, 0.0);
      return obstacleCourse;
   }

   public static void main(String[] args)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("exampleRobot"));
      //      PlanarRegionsList planarRegionsList = createMazeEnvironment();
      //      PlanarRegionsList planarRegionsList = generateSteppingStoneField(0.1, 0.1, 0.25, 0.3, 6);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double cinderBlockSize = 0.08;
      double cinderBlockHeight = 0.015;
      double courseLength = 3.25;
      double courseWidth = 0.9;
      double heightVariation = 0.0;
      double extrusionLength = -0.01;
      double percentageAbsent = 0.22;
      double minTilt = Math.toRadians(10.0);
      double maxTilt = Math.toRadians(75.0);
      double randomHeightVariation = 0.0;
      boolean onlyGenerateTopOfBlock = false;

      PlanarRegionsListExamples.generateCinderBlockField(generator,
                                                         cinderBlockSize,
                                                         cinderBlockHeight,
                                                         (int) (courseLength / cinderBlockSize),
                                                         (int) (courseWidth / cinderBlockSize),
                                                         heightVariation,
                                                         extrusionLength,
                                                         0.5,
                                                         percentageAbsent,
                                                         minTilt,
                                                         maxTilt,
                                                         randomHeightVariation,
                                                         onlyGenerateTopOfBlock);

      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment("ExamplePlanarRegionsListEnvironment",
                                                                                                new PlanarRegionsList[] {generator.getPlanarRegionsList()},
                                                                                                null,
                                                                                                1e-5,
                                                                                                false);
      TerrainObject3D terrainObject3D = environment.getTerrainObject3D();
      scs.addStaticLinkGraphics(terrainObject3D.getLinkGraphics());
      scs.setGroundVisible(false);

      //      Graphics3DObject startAndEndGraphics = new Graphics3DObject();
      //      startAndEndGraphics.translate(0.0, 0.0, 0.5);
      //      startAndEndGraphics.addSphere(0.2, YoAppearance.Green());
      //      startAndEndGraphics.identity();
      //      startAndEndGraphics.translate(3.0, 2.5, 0.5);
      //      startAndEndGraphics.addSphere(0.2, YoAppearance.Red());
      //      scs.addStaticLinkGraphics(startAndEndGraphics);

      scs.startOnAThread();
   }
}
