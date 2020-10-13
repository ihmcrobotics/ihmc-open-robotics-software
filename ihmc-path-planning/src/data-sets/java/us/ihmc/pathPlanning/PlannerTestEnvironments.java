package us.ihmc.pathPlanning;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class PlannerTestEnvironments
{
   public static PlanarRegionsList getWall()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(5.0, 5.0);
      generator.translate(-0.5, 0.5, 0.5);
      generator.rotate(Math.PI / 2.0, Axis3D.Y);
      generator.addRectangle(1.0, 1.5);
      generator.identity();
      generator.translate(0.5, -0.5, 0.5);
      generator.rotate(Math.PI / 2.0, Axis3D.Y);
      generator.addRectangle(1.0, 1.5);

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getMultiRegionFlatGround()
   {
      List<ConvexPolygon2D> polygons = new ArrayList<>();
      ConvexPolygon2D polygon0 = new ConvexPolygon2D();
      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      ConvexPolygon2D polygon2 = new ConvexPolygon2D();
      ConvexPolygon2D polygon3 = new ConvexPolygon2D();
      polygons.add(polygon0);
      polygons.add(polygon1);
      polygons.add(polygon2);
      polygons.add(polygon3);

      polygon0.addVertex(-10.0, 4.0);
      polygon0.addVertex(-10.0, -4.0);
      polygon0.addVertex(-5.0, 4.0);
      polygon0.addVertex(-5.0, -4.0);
      polygon0.update();

      polygon1.addVertex(-5.0, 4.0);
      polygon1.addVertex(-5.0, -4.0);
      polygon1.addVertex(-0.0, 3.99);
      polygon1.addVertex(-0.0, -3.99);
      polygon1.update();

      polygon2.addVertex(0.0, 3.99);
      polygon2.addVertex(0.0, -3.99);
      polygon2.addVertex(5.0, 3.99);
      polygon2.addVertex(5.0, -3.99);
      polygon2.update();

      polygon3.addVertex(10.0, 4.0);
      polygon3.addVertex(10.0, -4.0);
      polygon3.addVertex(5.0, 3.99);
      polygon3.addVertex(5.0, -3.99);
      polygon3.update();

      List<Point2D> concaveHullVertices = new ArrayList<>();
      concaveHullVertices.add(new Point2D(10.0, 4.0));
      concaveHullVertices.add(new Point2D(10.0, -4.0));
      concaveHullVertices.add(new Point2D(5.0, -3.99));
      concaveHullVertices.add(new Point2D(-5.0, -3.99));
      concaveHullVertices.add(new Point2D(-10.0, -4.0));
      concaveHullVertices.add(new Point2D(-10.0, 4.0));
      concaveHullVertices.add(new Point2D(-5.0, 3.99));
      concaveHullVertices.add(new Point2D(5.0, 3.99));

      PlanarRegion groundRegion = new PlanarRegion(new RigidBodyTransform(), concaveHullVertices, polygons);
      PlanarRegionsList regions = new PlanarRegionsList();
      regions.addPlanarRegion(groundRegion);

      return regions;
   }


   public static PlanarRegionsList getStepUpsAndDowns()
   {
      double cinderBlockSize = 0.4;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double cinderBlockHeight = 0.15;

      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(0.6, 5.0);
      generator.translate(0.4, 0.0, 0.0);

      generator.translate(0.0, -0.5 * cinderBlockSize, 0.0);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(0.0, cinderBlockSize, 0.0);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 2, 1);
      generator.translate(cinderBlockSize, -1.5 * cinderBlockSize, 0.05);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(0.0, cinderBlockSize, -0.1);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);

      generator.identity();
      generator.translate(3 * cinderBlockSize, 0.0, 0.001);
      generator.addRectangle(0.6, 5.0);
      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getStepAfterPitchUp()
   {
      return getStepAfterPitch(true);
   }

   public static PlanarRegionsList getStepAfterPitchDown()
   {
      return getStepAfterPitch(false);
   }

   private static PlanarRegionsList getStepAfterPitch(boolean pitchCinderBack)
   {
      double cinderBlockSize = 0.4;
      double fieldHeight = 0.4;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double cinderBlockHeight = 0.15;

      generator.translate(0.0, 0.0, fieldHeight); // avoid graphical issue
      generator.addRectangle(0.6, 5.0); // standing platform
      generator.translate(0.2 + cinderBlockSize, 0.0, 0.0); // forward to first row

      generator.translate(0.0, -0.5 * cinderBlockSize, -cinderBlockHeight);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(0.0, cinderBlockSize, 0.0);
      if (pitchCinderBack)
         PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 2, 1);
      else
         PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 1, 1);

      generator.translate(cinderBlockSize, -1.5 * cinderBlockSize, 0.03);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(0.0, cinderBlockSize, -0.06);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(cinderBlockSize, -cinderBlockSize, 0.06);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(0.0, cinderBlockSize, -0.06);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(cinderBlockSize, -cinderBlockSize, 0.06);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(0.0, cinderBlockSize, -0.06);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(cinderBlockSize, -0.5 * cinderBlockSize, 0.03);
      if (pitchCinderBack)
         PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 2, 1);
      else
         PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 1, 1);
      generator.translate(0.0, cinderBlockSize, 0.0);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);

      generator.translate(cinderBlockSize, -0.5 * cinderBlockSize, 0.03);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(0.0, cinderBlockSize, -0.06);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(cinderBlockSize, -cinderBlockSize, 0.06);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(0.0, cinderBlockSize, -0.06);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);

      generator.identity();
      generator.translate(8 * cinderBlockSize, 0.0, fieldHeight);
      generator.addRectangle(0.6, 5.0);
      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getPlatformsWithGaps()
   {
      double cinderBlockSize = 1.0;
      double fieldHeight = 0.4;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double cinderBlockHeight = 0.15;

      generator.translate(0.0, 0.0, fieldHeight); // avoid graphical issue
      generator.addRectangle(0.6, 5.0); // standing platform
      generator.translate(0.2 + cinderBlockSize, 0.0, 0.0); // forward to first row

      generator.translate(-0.2, -0.5 * cinderBlockSize, -cinderBlockHeight);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(0.0, cinderBlockSize, 0.0);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(0.2 + cinderBlockSize, -cinderBlockSize, 0.0);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
      generator.translate(0.0, cinderBlockSize, 0.0);
      PlanarRegionsListExamples.generateSingleCinderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);

      generator.identity();
      generator.translate(3 * cinderBlockSize, 0.0, fieldHeight);
      generator.addRectangle(0.6, 5.0);
      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getStepOnBox()
   {
      double stepHeight = 0.2;
      double boxSize = 1.0;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(1.0 + boxSize / 2.0, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(5.0, 5.0);

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getRotatedStepOnBox()
   {
      double stepHeight = 0.2;
      double boxSize = 1.0;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.rotate(Math.PI / 4.0, Axis3D.Z);
      generator.translate(-6.0 + 1.0 + boxSize / 2.0, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(5.0, 5.0);
      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getRandomTerrain(Random random)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(14.0, 3.0); // floor plane

      double length = RandomNumbers.nextDouble(random, 0.3, 1.0);
      double width = RandomNumbers.nextDouble(random, 0.3, 1.0);
      double height = RandomNumbers.nextDouble(random, 0.07, 0.3);

      for (int i = 0; i < 100; i++)
      {
         generator.identity();

         Vector3D translationVector = RandomGeometry.nextVector3D(random, -5.0, -1.0, -0.05, 5.0, 1.0, 0.0);
         generator.translate(translationVector);

         Quaternion rotation = RandomGeometry.nextQuaternion(random, Math.toRadians(15.0));
         generator.rotate(rotation);

         generator.addCubeReferencedAtBottomMiddle(length, width, height);
      }

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      return planarRegionsList;
   }

   public static PlanarRegionsList getSimpleGaps()
   {
      double boxHeight = 0.2;
      double boxSize = 0.87;
      double gapSize = 0.2;
      int numberOfGaps = 6;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, boxHeight);

      for (int i = 0; i < numberOfGaps; i++)
      {
         generator.translate(boxSize + gapSize, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, boxHeight);
      }

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getPartialGaps()
   {
      double absAngle = Math.toRadians(15.0);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.3, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.6, 1.0, 0.5);
      generator.translate(0.55, 0.0, 0.0);
      for (int i = 0; i < 10; i++)
      {
         double rotationAngle = i % 2 == 0 ? -absAngle : absAngle;
         generator.rotate(rotationAngle, Axis3D.Y);
         generator.addCubeReferencedAtBottomMiddle(0.1, 1.0, 0.5);
         generator.rotate(-rotationAngle, Axis3D.Y);
         double translation = i % 2 == 0 ? -0.05 : 0.45;
         generator.translate(translation, 0.0, 0.0);
      }
      generator.translate(0.1, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.6, 1.0, 0.5);

      // define start and goal conditions
      FramePose3D initialStanceFootPose = new FramePose3D();
      initialStanceFootPose.getPosition().set(0.3, 0.0, 0.5);
      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose3D goalPose = new FramePose3D();
      goalPose.getPosition().set(3.0, 0.0, 0.5);

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getBox()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(2.0, 0.0, 0.0001);
      generator.addRectangle(5.0, 5.0);
      generator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.0);
      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getSpiralStaircase()
   {
      ConvexPolygon2D circlePolygon = new ConvexPolygon2D();
      ArrayList<ConvexPolygon2D> steps = new ArrayList<>();

      int circleVertices = 10;
      double circleRadius = 1.0;
      double stepWidth = 0.5;
      double stepHeight = 0.175;

      for (int i = 0; i < circleVertices; i++)
      {
         ConvexPolygon2D stepPolygon = new ConvexPolygon2D();

         double x = circleRadius * Math.sin(2.0 * Math.PI * i / circleVertices);
         double y = circleRadius * Math.cos(2.0 * Math.PI * i / circleVertices);
         Point2D vertex = new Point2D(x, y);
         circlePolygon.addVertex(vertex);
         stepPolygon.addVertex(vertex);

         double xNext = circleRadius * Math.sin(2.0 * Math.PI * (i + 1) / circleVertices);
         double yNext = circleRadius * Math.cos(2.0 * Math.PI * (i + 1) / circleVertices);
         Point2D nextVertex = new Point2D(xNext, yNext);
         stepPolygon.addVertex(nextVertex);

         double xOutside1 = (circleRadius + stepWidth) * Math.sin(2.0 * Math.PI * i / circleVertices);
         double yOutside1 = (circleRadius + stepWidth) * Math.cos(2.0 * Math.PI * i / circleVertices);
         Point2D outsideVertex1 = new Point2D(xOutside1, yOutside1);
         stepPolygon.addVertex(outsideVertex1);

         double xOutside2 = (circleRadius + stepWidth) * Math.sin(2.0 * Math.PI * (i + 1) / circleVertices);
         double yOutside2 = (circleRadius + stepWidth) * Math.cos(2.0 * Math.PI * (i + 1) / circleVertices);
         Point2D outside2Vertex = new Point2D(xOutside2, yOutside2);
         stepPolygon.addVertex(outside2Vertex);

         stepPolygon.update();
         steps.add(stepPolygon);
      }
      circlePolygon.update();

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.0, 0.0, 0.0001);
      generator.addPolygon(circlePolygon);
      for (ConvexPolygon2D step : steps)
      {
         generator.addPolygon(step);
         generator.translate(0.0, 0.0, stepHeight);
      }

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getHole()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.0, 0.0, -0.5);
      generator.addRectangle(10.0, 10.0);
      generator.addCubeReferencedAtBottomMiddle(0.5, 0.5, 0.5);

      generator.translate(0.7, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.3, 0.5, 0.5);

      generator.translate(0.0, 0.5, 0.0);
      generator.addCubeReferencedAtBottomMiddle(1.9, 0.5, 0.5);
      generator.translate(0.0, -0.5, 0.0);

      generator.translate(0.7, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.5, 0.5, 0.5);

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getCorridor()
   {
      double corridorStartDistance = 0.5;
      double corridorWidth = 0.4;
      double corridorHeight = 2.0;
      double corridorLength = 0.25;
      double blockWidth = 2.0;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(5.0, 0.0, 0.0);
      generator.addRectangle(20.0, 2.0 * blockWidth + corridorWidth);
      generator.identity();
      generator.translate(corridorStartDistance + corridorLength / 2.0, (blockWidth + corridorWidth) / 2.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(corridorLength, blockWidth, corridorHeight);
      generator.translate(0.0, -blockWidth - corridorWidth, 0.0);
      generator.addCubeReferencedAtBottomMiddle(corridorLength, blockWidth, corridorHeight);
      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getQuadrupedEnvironment0()
   {
      // all flat with some space in between
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double startingBlockLength = 1.0;
      double cinderBlockSize = 0.25;
      double cinderBlockHeight = 0.02;
      double courseLength = 3.0;
      double courseWidth = 1.0;
      double heightVariation = 0.0;
      double extrusionLength = -0.05;
      double percentageAbsent = 0.0;
      double minTilt = Math.toRadians(0.0);
      double maxTilt = Math.toRadians(5.0);
      double randomHeightVariation = 0.0;
      boolean onlyGenerateTopOfBlock = true;

      PlanarRegionsListExamples.generateCinderBlockField(generator, cinderBlockSize, cinderBlockHeight, (int) Math.round(courseLength / cinderBlockSize),
                                                         (int) Math.round(courseWidth / cinderBlockSize), heightVariation, extrusionLength, startingBlockLength,
                                                         percentageAbsent, minTilt, maxTilt, randomHeightVariation, onlyGenerateTopOfBlock);
      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getQuadrupedEnvironment1()
   {
      // all flat with some absent
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double startingBlockLength = 1.0;
      double cinderBlockSize = 0.15;
      double cinderBlockHeight = 0.02;
      double courseLength = 3.0;
      double courseWidth = 0.75;
      double heightVariation = 0.0;
      double extrusionLength = -0.025;
      double percentageAbsent = 0.2;
      double minTilt = Math.toRadians(0.0);
      double maxTilt = Math.toRadians(5.0);
      double randomHeightVariation = 0.0;
      boolean onlyGenerateTopOfBlock = true;

      PlanarRegionsListExamples.generateCinderBlockField(generator, cinderBlockSize, cinderBlockHeight, (int) Math.round(courseLength / cinderBlockSize),
                                                         (int) Math.round(courseWidth / cinderBlockSize), heightVariation, extrusionLength, startingBlockLength,
                                                         percentageAbsent, minTilt, maxTilt, randomHeightVariation, onlyGenerateTopOfBlock);

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getQuadrupedEnvironment2()
   {
      // large blocks with some height variation
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double startingBlockLength = 1.0;
      double cinderBlockSize = 0.5;
      double cinderBlockHeight = 0.02;
      double courseLength = 3.0;
      double courseWidth = 1.0;
      double heightVariation = 0.03;
      double extrusionLength = -0.05;
      double percentageAbsent = 0.0;
      double minTilt = Math.toRadians(0.0);
      double maxTilt = Math.toRadians(0.0);
      double randomHeightVariation = 0.04;
      boolean onlyGenerateTopOfBlock = true;

      PlanarRegionsListExamples.generateCinderBlockField(generator, cinderBlockSize, cinderBlockHeight, (int) Math.round(courseLength / cinderBlockSize),
                                                         (int) Math.round(courseWidth / cinderBlockSize), heightVariation, extrusionLength, startingBlockLength,
                                                         percentageAbsent, minTilt, maxTilt, randomHeightVariation, onlyGenerateTopOfBlock);
      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getQuadrupedEnvironment3()
   {
      // small footholds with large angle variation
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double startingBlockLength = 1.0;
      double cinderBlockSize = 0.1;
      double cinderBlockHeight = 0.02;
      double courseLength = 3.0;
      double courseWidth = 1.0;
      double heightVariation = 0.0;
      double extrusionLength = -0.01;
      double percentageAbsent = 0.1;
      double minTilt = Math.toRadians(10.0);
      double maxTilt = Math.toRadians(45.0);
      double randomHeightVariation = 0.0;
      boolean onlyGenerateTopOfBlock = true;

      PlanarRegionsListExamples.generateCinderBlockField(generator, cinderBlockSize, cinderBlockHeight, (int) Math.round(courseLength / cinderBlockSize),
                                                         (int) Math.round(courseWidth / cinderBlockSize), heightVariation, extrusionLength, startingBlockLength,
                                                         percentageAbsent, minTilt, maxTilt, randomHeightVariation, onlyGenerateTopOfBlock);

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getSimpleCorridor()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(-1.0, 0.0, 0.0);

      generator.translate(5.0, 0.0, 0.0);
      generator.addRectangle(10.0, 3.0);
      generator.translate(-5.0, 0.0, 0.0);

      generator.translate(0.0, 1.5, 0.0);
      generator.rotate(Math.toRadians(90.0), Axis3D.X);
      generator.addRectangleReferencedAtNegativeXYCorner(10.0, 2.0);
      generator.rotate(Math.toRadians(-90.0), Axis3D.X);
      generator.translate(0.0, -1.5, 0.0);

      generator.translate(10.0, -1.5, 0.0);
      generator.rotate(Math.toRadians(180.0), Axis3D.Z);
      generator.rotate(Math.toRadians(90.0), Axis3D.X);
      generator.addRectangleReferencedAtNegativeXYCorner(10.0, 2.0);
      generator.rotate(Math.toRadians(-90.0), Axis3D.X);
      generator.rotate(Math.toRadians(-180.0), Axis3D.Z);
      generator.translate(-10.0, 1.5, 0.0);

      generator.translate(0.0, 0.0, 1.0);
      generator.rotate(Math.toRadians(90.0), Axis3D.Y);
      generator.addRectangle(2.0, 3.0);
      generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
      generator.translate(0.0, 0.0, -1.0);

      generator.translate(10.0, 0.0, 1.0);
      generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
      generator.addRectangle(2.0, 3.0);
      generator.rotate(Math.toRadians(90.0), Axis3D.Y);
      generator.translate(-10.0, 0.0, -1.0);

      generator.translate(1.0, 0.0, 0.0);

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getCorridorWith1Wall()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.translate(-1.0, 0.0, 0.0);

      generator.translate(3.0, 0.5, 1.0);
      generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
      generator.addRectangle(2.0, 2.0);
      generator.rotate(Math.toRadians(90.0), Axis3D.Y);
      generator.translate(-3.0, -0.5, -1.0);

      generator.translate(1.0, 0.0, 0.0);

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      planarRegionsList.addPlanarRegionsList(getSimpleCorridor());

      return planarRegionsList;
   }

   public static PlanarRegionsList getTrickCorridor()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(-3.0, 0.0, 0.0);

      // floor
      generator.translate(5.0, -1.5, 0.0);
      generator.addRectangle(10.0, 6.0);
      generator.translate(-5.0, 1.5, 0.0);

      drawTrickyCorridorWalls(generator);

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList getTrickCorridorWCutFloor()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(-3.0, 0.0, 0.0);

      // floor
      generator.translate(2.5, -1.5, 0.0);
      generator.addRectangle(5.0, 6.0);
      generator.translate(-2.5, 1.5, 0.0);
      generator.translate(7.5, -1.5, 0.0);
      generator.addRectangle(5.0, 6.0);
      generator.translate(-7.5, 1.5, 0.0);

      drawTrickyCorridorWalls(generator);

      return generator.getPlanarRegionsList();
   }

   private static void drawTrickyCorridorWalls(PlanarRegionsListGenerator generator)
   {
      // left wall
      generator.translate(0.0, 1.5, 0.0);
      generator.rotate(Math.toRadians(90.0), Axis3D.X);
      generator.addRectangleReferencedAtNegativeXYCorner(10.0, 2.0);
      generator.rotate(Math.toRadians(-90.0), Axis3D.X);
      generator.translate(0.0, -1.5, 0.0);

      // obstacle wall
      generator.translate(5.0, 0.5, 1.0);
      generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
      generator.addRectangle(2.0, 2.0);
      generator.rotate(Math.toRadians(90.0), Axis3D.Y);
      generator.translate(-5.0, -0.5, -1.0);

      // dead end wall
      generator.translate(7.0, 0.0, 1.0);
      generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
      generator.addRectangle(2.0, 3.0);
      generator.rotate(Math.toRadians(90.0), Axis3D.Y);
      generator.translate(-7.0, 0.0, -1.0);

      // corridor split wall
      generator.translate(7.0, -1.5, 0.0);
      generator.rotate(Math.toRadians(180.0), Axis3D.Z);
      generator.rotate(Math.toRadians(90.0), Axis3D.X);
      generator.addRectangleReferencedAtNegativeXYCorner(5.0, 2.0);
      generator.rotate(Math.toRadians(-90.0), Axis3D.X);
      generator.rotate(Math.toRadians(-180.0), Axis3D.Z);
      generator.translate(-7.0, 1.5, 0.0);

      // right wall
      generator.translate(10.0, -4.5, 0.0);
      generator.rotate(Math.toRadians(180.0), Axis3D.Z);
      generator.rotate(Math.toRadians(90.0), Axis3D.X);
      generator.addRectangleReferencedAtNegativeXYCorner(10.0, 2.0);
      generator.rotate(Math.toRadians(-90.0), Axis3D.X);
      generator.rotate(Math.toRadians(-180.0), Axis3D.Z);
      generator.translate(-10.0, 4.5, 0.0);

      // close end wall
      generator.translate(0.0, -1.5, 1.0);
      generator.rotate(Math.toRadians(90.0), Axis3D.Y);
      generator.addRectangle(2.0, 6.0);
      generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
      generator.translate(0.0, 1.5, -1.0);

      // opposite end wall
      generator.translate(10.0, -1.5, 1.0);
      generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
      generator.addRectangle(2.0, 6.0);
      generator.rotate(Math.toRadians(90.0), Axis3D.Y);
      generator.translate(-10.0, 0.0, -1.0);

      generator.translate(3.0, 1.5, 0.0);
   }

   public static PlanarRegionsList getTrickCorridorWidened()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double squareSize = 2.3;
      double halfSquareSize = squareSize / 2.0;
      double courseWidth = 3.0 * squareSize;
      double halfCourseWidth = courseWidth / 2.0;
      double courseHeight = 4.0 * squareSize;
      double halfCourseHeight = courseHeight / 2.0;
      double wallHeight = 2.0;

      // draw from center
      offset(generator, halfCourseHeight - (1.2 * squareSize), halfCourseWidth - (2.0 * squareSize), () ->
      {
         // floor
         generator.addRectangle(courseHeight, courseWidth); // referenced from center

         // left wall
         offset(generator, -halfCourseHeight, halfCourseWidth, () ->
         {
            generator.rotate(Math.toRadians(90.0), Axis3D.X);
            generator.addRectangleReferencedAtNegativeXYCorner(courseHeight, wallHeight);
            generator.rotate(Math.toRadians(-90.0), Axis3D.X);
         });

         // right wall
         offset(generator, halfCourseHeight, -halfCourseWidth, () ->
         {
            generator.rotate(Math.toRadians(180.0), Axis3D.Z);
            generator.rotate(Math.toRadians(90.0), Axis3D.X);
            generator.addRectangleReferencedAtNegativeXYCorner(courseHeight, wallHeight);
            generator.rotate(Math.toRadians(-90.0), Axis3D.X);
            generator.rotate(Math.toRadians(-180.0), Axis3D.Z);
         });

         // near wall
         offset(generator, -halfCourseHeight, 0.0, wallHeight / 2.0, () ->
         {
            generator.rotate(Math.toRadians(90.0), Axis3D.Y);
            generator.addRectangle(wallHeight, courseWidth);
            generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
         });

         // far wall
         offset(generator, halfCourseHeight, 0.0, wallHeight / 2.0, () ->
         {
            generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
            generator.addRectangle(wallHeight, courseWidth);
            generator.rotate(Math.toRadians(90.0), Axis3D.Y);
         });

         // inner right wall 1
         offset(generator, 1.0 * squareSize, -(0.5* squareSize), () ->
         {
            generator.rotate(Math.toRadians(180.0), Axis3D.Z);
            generator.rotate(Math.toRadians(90.0), Axis3D.X);
            generator.addRectangleReferencedAtNegativeXYCorner(2.0 * squareSize, wallHeight);
            generator.rotate(Math.toRadians(-90.0), Axis3D.X);
            generator.rotate(Math.toRadians(-180.0), Axis3D.Z);
         });

         // inner far wall 1
         offset(generator, -0.0 * squareSize, 1.0 * squareSize, wallHeight / 2.0, () ->
         {
            generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
            generator.addRectangle(wallHeight, squareSize);
            generator.rotate(Math.toRadians(90.0), Axis3D.Y);
         });

         // inner far wall 2
         offset(generator, 1.0 * squareSize, 0.5 * squareSize, wallHeight / 2.0, () ->
         {
            generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
            generator.addRectangle(wallHeight, 2.0 * squareSize);
            generator.rotate(Math.toRadians(90.0), Axis3D.Y);
         });
      });

      return generator.getPlanarRegionsList();
   }

   public static double MAZE_CORRIDOR_SQUARE_SIZE = 2.3;

   public static PlanarRegionsList getMazeCorridor()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      double squareSize = MAZE_CORRIDOR_SQUARE_SIZE;
      double halfSquareSize = squareSize / 2.0;
      double courseWidth = 4.0 * squareSize;
      double halfCourseWidth = courseWidth / 2.0;
      double courseHeight = 5.0 * squareSize;
      double halfCourseHeight = courseHeight / 2.0;
      double wallHeight = 2.0;

      // draw from center
      offset(generator, halfCourseHeight - halfSquareSize, halfCourseWidth - halfSquareSize, () ->
      {
         // floor
         generator.addRectangle(courseHeight, courseWidth); // referenced from center

         // left wall
         offset(generator, -halfCourseHeight, halfCourseWidth, () ->
         {
            generator.rotate(Math.toRadians(90.0), Axis3D.X);
            generator.addRectangleReferencedAtNegativeXYCorner(courseHeight, wallHeight);
            generator.rotate(Math.toRadians(-90.0), Axis3D.X);
         });

         // right wall
         offset(generator, halfCourseHeight, -halfCourseWidth, () ->
         {
            generator.rotate(Math.toRadians(180.0), Axis3D.Z);
            generator.rotate(Math.toRadians(90.0), Axis3D.X);
            generator.addRectangleReferencedAtNegativeXYCorner(courseHeight, wallHeight);
            generator.rotate(Math.toRadians(-90.0), Axis3D.X);
            generator.rotate(Math.toRadians(-180.0), Axis3D.Z);
         });

         // near wall
         offset(generator, -halfCourseHeight, 0.0, wallHeight / 2.0, () ->
         {
            generator.rotate(Math.toRadians(90.0), Axis3D.Y);
            generator.addRectangle(wallHeight, courseWidth);
            generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
         });

         // far wall
         offset(generator, halfCourseHeight, 0.0, wallHeight / 2.0, () ->
         {
            generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
            generator.addRectangle(wallHeight, courseWidth);
            generator.rotate(Math.toRadians(90.0), Axis3D.Y);
         });

         // inner left wall 1
         offset(generator, -1.5 * squareSize, 1.0 * squareSize, () ->
         {
            generator.rotate(Math.toRadians(90.0), Axis3D.X);
            generator.addRectangleReferencedAtNegativeXYCorner(3.0 * squareSize, wallHeight);
            generator.rotate(Math.toRadians(-90.0), Axis3D.X);
         });

         // inner left wall 2
         offset(generator, 0.5 * squareSize, 0.0, () ->
         {
            generator.rotate(Math.toRadians(90.0), Axis3D.X);
            generator.addRectangleReferencedAtNegativeXYCorner(2.0 * squareSize, wallHeight);
            generator.rotate(Math.toRadians(-90.0), Axis3D.X);
         });

         // inner right wall 1
         offset(generator, 2.5 * squareSize, -squareSize, () ->
         {
            generator.rotate(Math.toRadians(180.0), Axis3D.Z);
            generator.rotate(Math.toRadians(90.0), Axis3D.X);
            generator.addRectangleReferencedAtNegativeXYCorner(3.0 * squareSize, wallHeight);
            generator.rotate(Math.toRadians(-90.0), Axis3D.X);
            generator.rotate(Math.toRadians(-180.0), Axis3D.Z);
         });

         // inner near wall 1
         offset(generator, -0.5 * squareSize, 0.0, wallHeight / 2.0, () ->
         {
            generator.rotate(Math.toRadians(90.0), Axis3D.Y);
            generator.addRectangle(wallHeight, 2.0 * squareSize);
            generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
         });

         // inner near wall 2
         offset(generator, -1.5 * squareSize, 0.5 * squareSize, wallHeight / 2.0, () ->
         {
            generator.rotate(Math.toRadians(90.0), Axis3D.Y);
            generator.addRectangle(wallHeight, squareSize);
            generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
         });

         // inner near wall 3
         offset(generator, -1.5 * squareSize, -1.5 * squareSize, wallHeight / 2.0, () ->
         {
            generator.rotate(Math.toRadians(90.0), Axis3D.Y);
            generator.addRectangle(wallHeight, squareSize);
            generator.rotate(Math.toRadians(-90.0), Axis3D.Y);
         });
      });

      return generator.getPlanarRegionsList();
   }

   private static void offset(PlanarRegionsListGenerator generator, double x, double y, Runnable runnable)
   {
      offset(generator, x, y, 0.0, runnable);
   }

   private static void offset(PlanarRegionsListGenerator generator, double x, double y, double z, Runnable runnable)
   {
      generator.translate(x, y, z);
      runnable.run();
      generator.translate(-x, -y, -z);
   }

   public static PlanarRegionsList getSimplePlatform()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      // floor
      generator.addRectangle(10.0, 6.0);

      // top platform
      generator.translate(0.0, 0.0, 0.46);
      generator.addRectangle(1.14, 1.14);

      generator.identity();
      generator.translate((1.14 + 0.38) / 2.0, 0.0, 0.36);
      generator.addRectangle(0.38, 1.14);

      generator.translate(0.38, 0.0, -0.15);
      generator.addRectangle(0.38, 1.14);

      return generator.getPlanarRegionsList();
   }

   public static PlanarRegionsList flatGround()
   {
      return flatGround(20.0);
   }

   public static PlanarRegionsList flatGround(double size)
   {
      return PlanarRegionsList.flatGround(size);
   }

   public static void main(String[] args)
   {
      String dataSetNameSuffix = "MultiRegionFlatGround";
      DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");
      String date = LocalDateTime.now().format(dateTimeFormatter);

      DataSet dataSet = new DataSet(date + "_" + dataSetNameSuffix, getMultiRegionFlatGround());
      DataSetIOTools.exportDataSet(dataSet);
   }
}