package us.ihmc.footstepPlanning.testTools;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.TwoBollardEnvironment;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;

import java.util.ArrayList;
import java.util.Random;

public class PlannerTestEnvironments
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static final String staircase = "staircase";
   public static final String wall = "wall";
   public static final String overCinderBlockField = "overCinderBlockField";
   public static final String steppingStones = "steppingStones";
   public static final String stepUpsAndDownsScoringDifficult = "stepUpsAndDownsScoringDifficult";
   public static final String stepAfterPitchDown = "stepAfterPitchDown";
   public static final String stepAfterPitchUp = "stepAfterPitchUp";
   public static final String compareStepBeforeGap = "compareStepBeforeGap";
   public static final String simpleStepOnBox = "simpleStepOnBox";
   public static final String simpleStepOnBoxTwo = "simpleStepOnBoxTwo";
   public static final String random = "random";
   public static final String simpleGaps = "simpleGaps";
   public static final String partialGaps = "partialGaps";
   public static final String box = "box";
   public static final String spiralStaircase = "spiralStaircase";
   public static final String hole = "hole";
   public static final String corridor = "corridor";
   public static final String bollards = "bollards";

   public static final PlannerTestData staircaseData = new StaircaseTestData();
   public static final PlannerTestData wallData = new WallTestData();
   public static final PlannerTestData overCinderBlockFieldData = new OverCinderBlockFieldTestData();
   public static final PlannerTestData steppingStonesData = new SteppingStonesTestData();
   public static final PlannerTestData stepUpsAndDownsScoringDifficultData = new StepUpsAndDownsScoringDifficultTestData();
   public static final PlannerTestData stepAfterPitchDownData = new StepAfterPitchDownTestData();
   public static final PlannerTestData stepAfterPitchUpData = new StepAfterPitchUpTestData();
   public static final PlannerTestData compareStepBeforeGapData = new CompareStepBeforeGapTestData();
   public static final PlannerTestData simpleStepOnBoxData = new SimpleStepOnBoxTestData();
   public static final PlannerTestData simpleStepOnBoxTwoData = new SimpleStepOnBoxTwoTestData();
   public static final PlannerTestData randomData = new RandomTestData();
   public static final PlannerTestData simpleGapsData = new SimpleGapsTestData();
   public static final PlannerTestData partialGapsData = new PartialGapsTestData();
   public static final PlannerTestData boxData = new BoxTestData();
   public static final PlannerTestData spiralStaircaseData = new SpiralStaircaseTestData();
   public static final PlannerTestData holeData = new HoleTestData();
   public static final PlannerTestData corridorData = new CorridorTestData();
   public static final PlannerTestData bollardData = new BollardsTestData();


   public static PlannerTestData getTestData(String test)
   {
      switch (test)
      {
      case staircase:
         return staircaseData;
      case wall:
         return wallData;
      case overCinderBlockField:
         return overCinderBlockFieldData;
      case steppingStones:
         return steppingStonesData;
      case stepUpsAndDownsScoringDifficult:
         return stepUpsAndDownsScoringDifficultData;
      case stepAfterPitchDown:
         return stepAfterPitchDownData;
      case stepAfterPitchUp:
         return stepAfterPitchUpData;
      case compareStepBeforeGap:
         return compareStepBeforeGapData;
      case simpleStepOnBox:
         return simpleStepOnBoxData;
      case simpleStepOnBoxTwo:
         return simpleStepOnBoxTwoData;
      case random:
         return randomData;
      case simpleGaps:
         return simpleGapsData;
      case partialGaps:
         return partialGapsData;
      case box:
         return boxData;
      case spiralStaircase:
         return spiralStaircaseData;
      case corridor:
         return corridorData;
      case bollards:
         return bollardData;
      default:
         return holeData;
      }
   }


   private static class StaircaseTestData extends PlannerTestData
   {
      public StaircaseTestData()
      {
         super(staircase);

         Vector3D rotationVector = new Vector3D();
         PlanarRegionsList stairCase = PlanarRegionsListExamples.generateStairCase(rotationVector);

         // define start and goal conditions
         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         RobotSide initialStanceSide = RobotSide.LEFT;

         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(2.0, -0.2, 0.53);

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(stairCase);
         setStartSide(initialStanceSide);
      }
   }

   private static class WallTestData extends PlannerTestData
   {
      public WallTestData()
      {
         super(wall);

         PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
         generator.addRectangle(5.0, 5.0);
         generator.translate(-0.5, 0.5, 0.5);
         generator.rotate(Math.PI / 2.0, Axis.Y);
         generator.addRectangle(1.0, 1.5);
         generator.identity();
         generator.translate(0.5, -0.5, 0.5);
         generator.rotate(Math.PI / 2.0, Axis.Y);
         generator.addRectangle(1.0, 1.5);
         PlanarRegionsList regions = generator.getPlanarRegionsList();

         FootstepPlannerParameters plannerParameters = new DefaultFootstepPlanningParameters();
         // define start and goal conditions
         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         RobotSide initialStanceSide = RobotSide.LEFT;
         initialStanceFootPose.setY(initialStanceSide.negateIfRightSide(plannerParameters.getIdealFootstepWidth() / 2.0));
         initialStanceFootPose.setX(-2.0);

         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(2.0, 0.0, 0.0);

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(regions);
         setStartSide(initialStanceSide);
      }
   }

   private static class OverCinderBlockFieldTestData extends PlannerTestData
   {
      public OverCinderBlockFieldTestData()
      {
         super(overCinderBlockField);

         double startX = 0.0;
         double startY = 0.0;
         double cinderBlockHeight = 0.15;
         double cinderBlockSize = 0.4;
         int courseWidthXInNumberOfBlocks = 21;
         int courseLengthYInNumberOfBlocks = 6;
         double heightVariation = 0.1;
         PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
         generator.translate(startX, startY, -0.001);
         PlanarRegionsListExamples
               .generateCinderBlockField(generator, cinderBlockSize, cinderBlockHeight, courseWidthXInNumberOfBlocks, courseLengthYInNumberOfBlocks,
                                         heightVariation);
         PlanarRegionsList cinderBlockField = generator.getPlanarRegionsList();

         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(9.0, 0.0, 0.0);

         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         initialStanceFootPose.setPosition(0.0, -0.7, 0.0);
         RobotSide initialStanceSide = RobotSide.RIGHT;

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(cinderBlockField);
         setStartSide(initialStanceSide);
      }
   }

   private static class SteppingStonesTestData extends PlannerTestData
   {
      public SteppingStonesTestData()
      {
         super(steppingStones);

         double pathRadius = 3.5;
         PlanarRegionsList cinderBlockField = PlanarRegionsListExamples.generateSteppingStonesEnvironment(pathRadius);

         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(pathRadius + 0.5, pathRadius, 0.0);

         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         initialStanceFootPose.setPosition(0.0, -0.7, 0.0);
         initialStanceFootPose.appendYawRotation(0.5 * Math.PI);
         RobotSide initialStanceSide = RobotSide.RIGHT;

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(cinderBlockField);
         setStartSide(initialStanceSide);
      }
   }

   private static class StepUpsAndDownsScoringDifficultTestData extends PlannerTestData
   {
      public StepUpsAndDownsScoringDifficultTestData()
      {
         super(stepUpsAndDownsScoringDifficult);

         double cinderBlockSize = 0.4;
         PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

         double cinderBlockHeight = 0.15;

         generator.translate(0.0, 0.0, 0.001); // avoid graphical issue
         generator.addRectangle(0.6, 5.0); // standing platform
         generator.translate(0.4, 0.0, 0.0); // forward to first row

         generator.translate(0.0, -0.5 * cinderBlockSize, 0.0);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(0.0, cinderBlockSize, 0.0);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 2, 1);
         generator.translate(cinderBlockSize, -1.5 * cinderBlockSize, 0.05);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(0.0, cinderBlockSize, -0.1);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);

         generator.identity();
         generator.translate(3 * cinderBlockSize, 0.0, 0.001);
         generator.addRectangle(0.6, 5.0);
         PlanarRegionsList cinderBlockField = generator.getPlanarRegionsList();

         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(3 * cinderBlockSize, -0.225, 0.0);

         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         initialStanceFootPose.setPosition(0.0, 0.1, 0.0);
         RobotSide initialStanceSide = RobotSide.LEFT;

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(cinderBlockField);
         setStartSide(initialStanceSide);
      }

   }

   private static class StepAfterPitchUpTestData extends CompareAfterPitchedStep
   {
      public StepAfterPitchUpTestData()
      {
         super(stepAfterPitchUp, true);
      }
   }

   private static class StepAfterPitchDownTestData extends CompareAfterPitchedStep
   {
      public StepAfterPitchDownTestData()
      {
         super(stepAfterPitchDown, false);
      }
   }

   private static class CompareAfterPitchedStep extends PlannerTestData
   {
      public CompareAfterPitchedStep(String name, boolean pitchCinderBack)
      {
         super(name);

         double cinderBlockSize = 0.4;
         double fieldHeight = 0.4;
         PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

         double cinderBlockHeight = 0.15;

         generator.translate(0.0, 0.0, fieldHeight); // avoid graphical issue
         generator.addRectangle(0.6, 5.0); // standing platform
         generator.translate(0.2 + cinderBlockSize, 0.0, 0.0); // forward to first row

         generator.translate(0.0, -0.5 * cinderBlockSize, -cinderBlockHeight);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(0.0, cinderBlockSize, 0.0);
         if (pitchCinderBack)
            PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 2, 1);
         else
            PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 1, 1);

         generator.translate(cinderBlockSize, -1.5 * cinderBlockSize, 0.03);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(0.0, cinderBlockSize, -0.06);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(cinderBlockSize, -cinderBlockSize, 0.06);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(0.0, cinderBlockSize, -0.06);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(cinderBlockSize, -cinderBlockSize, 0.06);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(0.0, cinderBlockSize, -0.06);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(cinderBlockSize, -0.5 * cinderBlockSize, 0.03);
         if (pitchCinderBack)
            PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 2, 1);
         else
            PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 1, 1);
         generator.translate(0.0, cinderBlockSize, 0.0);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);

         generator.translate(cinderBlockSize, -0.5 * cinderBlockSize, 0.03);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(0.0, cinderBlockSize, -0.06);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(cinderBlockSize, -cinderBlockSize, 0.06);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(0.0, cinderBlockSize, -0.06);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);

         generator.identity();
         generator.translate(8 * cinderBlockSize, 0.0, fieldHeight);
         generator.addRectangle(0.6, 5.0);
         PlanarRegionsList cinderBlockField = generator.getPlanarRegionsList();

         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(8 * cinderBlockSize + 0.2, 0.0, fieldHeight);

         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         initialStanceFootPose.setPosition(0.0, 0.1, fieldHeight);
         RobotSide initialStanceSide = RobotSide.LEFT;

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(cinderBlockField);
         setStartSide(initialStanceSide);
      }
   }

   private static class CompareStepBeforeGapTestData extends PlannerTestData
   {
      public CompareStepBeforeGapTestData()
      {
         super(compareStepBeforeGap);

         double cinderBlockSize = 1.0;
         double fieldHeight = 0.4;
         PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

         double cinderBlockHeight = 0.15;

         generator.translate(0.0, 0.0, fieldHeight); // avoid graphical issue
         generator.addRectangle(0.6, 5.0); // standing platform
         generator.translate(0.2 + cinderBlockSize, 0.0, 0.0); // forward to first row

         generator.translate(-0.2, -0.5 * cinderBlockSize, -cinderBlockHeight);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(0.0, cinderBlockSize, 0.0);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(0.2 + cinderBlockSize, -cinderBlockSize, 0.0);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);
         generator.translate(0.0, cinderBlockSize, 0.0);
         PlanarRegionsListExamples.generateSingleCiderBlock(generator, cinderBlockSize, cinderBlockHeight, 0, 0);

         generator.identity();
         generator.translate(3 * cinderBlockSize, 0.0, fieldHeight);
         generator.addRectangle(0.6, 5.0);
         PlanarRegionsList cinderBlockField = generator.getPlanarRegionsList();

         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(3 * cinderBlockSize, 0.0, fieldHeight);

         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         initialStanceFootPose.setPosition(0.0, 0.1, fieldHeight);
         RobotSide initialStanceSide = RobotSide.LEFT;

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(cinderBlockField);
         setStartSide(initialStanceSide);
      }
   }

   private static class SimpleStepOnBoxTestData extends PlannerTestData
   {
      public SimpleStepOnBoxTestData()
      {
         super(simpleStepOnBox);

         // create planar regions
         double stepHeight = 0.2;
         double boxSize = 1.0;
         PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
         generator.translate(1.0 + boxSize / 2.0, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
         generator.translate(0.0, 0.0, 0.001);
         generator.addRectangle(5.0, 5.0); // floor plane

         // define start and goal conditions
         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         RobotSide initialStanceSide = RobotSide.LEFT;
         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(1.0 + boxSize / 2.0, 0.0, stepHeight);

         // run the test
         PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(planarRegionsList);
         setStartSide(initialStanceSide);
      }
   }

   private static class SimpleStepOnBoxTwoTestData extends PlannerTestData
   {
      public SimpleStepOnBoxTwoTestData()
      {
         super(simpleStepOnBoxTwo);

         // create planar regions
         double stepHeight = 0.2;
         double boxSize = 1.0;
         PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
         generator.rotate(Math.PI / 4.0, Axis.Z);
         generator.translate(-6.0 + 1.0 + boxSize / 2.0, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
         generator.translate(0.0, 0.0, 0.001);
         generator.addRectangle(5.0, 5.0); // floor plane

         // define start and goal conditions
         RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
         transformGenerator.rotate(Math.PI / 4.0, Axis.Z);
         RigidBodyTransform transform = transformGenerator.getRigidBodyTransformCopy();
         Point3D startPosition = new Point3D(-6.0, 0.0, 0.0);
         transform.transform(startPosition);

         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         initialStanceFootPose.setPosition(startPosition);
         initialStanceFootPose.setOrientation(new AxisAngle(new Vector3D(0.0, 0.0, 1.0), Math.PI / 4.0));

         Point3D goalPosition = new Point3D(-6.0 + 1.0 + boxSize / 2.0, 0.0, stepHeight);
         transform.transform(goalPosition);

         RobotSide initialStanceSide = RobotSide.LEFT;
         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(goalPosition);
         goalPose.setOrientation(new AxisAngle(new Vector3D(0.0, 0.0, 1.0), Math.PI / 4.0));

         // run the test
         PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(planarRegionsList);
         setStartSide(initialStanceSide);
      }
   }

   private static class RandomTestData extends PlannerTestData
   {
      public RandomTestData()
      {
         super(random);

         // define start and goal conditions
         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         RobotSide initialStanceSide = RobotSide.LEFT;
         FramePose3D goalPose = new FramePose3D(worldFrame);
         initialStanceFootPose.setPosition(-6.0, 0.0, 0.0);
         goalPose.setPosition(6.0, 0.0, 0.0);

         // run the test
         Random randomNumber = new Random(42747621889239430L);
         PlanarRegionsList planarRegionsList = generateRandomTerrain(randomNumber);

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(planarRegionsList);
         setStartSide(initialStanceSide);
      }
   }

   private static PlanarRegionsList generateRandomTerrain(Random random)
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

   private static class SimpleGapsTestData extends PlannerTestData
   {
      public SimpleGapsTestData()
      {
         super(simpleGaps);

         // create planar regions
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

         //      generator.identity();
         //      generator.translate(0.0, 0.0, 0.001);
         //      generator.addRectangle(5.0, 5.0); // floor plane

         // define start and goal conditions
         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         initialStanceFootPose.setPosition(0.0, 0.0, boxHeight);
         RobotSide initialStanceSide = RobotSide.LEFT;
         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(numberOfGaps * (boxSize + gapSize), 0.0, boxHeight);
         goalPose.setOrientation(new AxisAngle(new Vector3D(0.0, 0.0, 1.0), Math.PI / 4.0));

         // run the test
         PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(planarRegionsList);
         setStartSide(initialStanceSide);
      }
   }

   private static class PartialGapsTestData extends PlannerTestData
   {
      public PartialGapsTestData()
      {
         super(partialGaps);

         double absAngle = Math.toRadians(15.0);

         PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
         generator.translate(0.3, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(0.6, 1.0, 0.5);
         generator.translate(0.55, 0.0, 0.0);
         for (int i = 0; i < 10; i++)
         {
            double rotationAngle = i % 2 == 0 ? -absAngle : absAngle;
            generator.rotate(rotationAngle, Axis.Y);
            generator.addCubeReferencedAtBottomMiddle(0.1, 1.0, 0.5);
            generator.rotate(-rotationAngle, Axis.Y);
            double translation = i % 2 == 0 ? -0.05 : 0.45;
            generator.translate(translation, 0.0, 0.0);
         }
         generator.translate(0.1, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(0.6, 1.0, 0.5);

         // define start and goal conditions
         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         initialStanceFootPose.setPosition(0.3, 0.0, 0.5);
         RobotSide initialStanceSide = RobotSide.LEFT;
         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(3.0, 0.0, 0.5);

         PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(planarRegionsList);
         setStartSide(initialStanceSide);
      }

   }

   private static class BoxTestData extends PlannerTestData
   {
      public BoxTestData()
      {
         super(box);

         PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
         generator.translate(2.0, 0.0, 0.0001);
         generator.addRectangle(5.0, 5.0);
         generator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.0);
         PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

         // define start and goal conditions
         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         initialStanceFootPose.setPosition(0.0, 0.15, 0.0);
         RobotSide initialStanceSide = RobotSide.LEFT;
         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(4.0, 0.0, 0.0);

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(planarRegionsList);
         setStartSide(initialStanceSide);
      }
   }

   private static class SpiralStaircaseTestData extends PlannerTestData
   {
      public SpiralStaircaseTestData()
      {
         super(spiralStaircase);

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
         PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

         // define start and goal conditions
         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         initialStanceFootPose.setPosition(0.0, 0.15, 0.0);
         RobotSide initialStanceSide = RobotSide.LEFT;
         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(-0.1, 1.25, stepHeight * (circleVertices - 1));

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(planarRegionsList);
         setStartSide(initialStanceSide);

      }

   }

   private static class HoleTestData extends PlannerTestData
   {
      public HoleTestData()
      {
         super(hole);

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

         PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

         // define start and goal conditions
         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         initialStanceFootPose.setPosition(0.0, 0.15, 0.0);
         RobotSide initialStanceSide = RobotSide.LEFT;
         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(1.3, 0.0, 0.0);

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(planarRegionsList);
         setStartSide(initialStanceSide);
      }
   }

   private static class CorridorTestData extends PlannerTestData
   {
      public CorridorTestData()
      {
         super(corridor);

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
         PlanarRegionsList regions = generator.getPlanarRegionsList();

         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         RobotSide initialStanceSide = RobotSide.LEFT;
         initialStanceFootPose.setY(initialStanceSide.negateIfRightSide(0.15));
         initialStanceFootPose.setX(-2.0);

         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(corridorStartDistance + corridorLength + 1.0, 0.0, 0.0);

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(regions);
         setStartSide(initialStanceSide);
      }
   }

   private static class BollardsTestData extends PlannerTestData
   {
      public BollardsTestData()
      {
         super(bollards);

         double bollardSeparation = 0.85;
         PlanarRegionsList regions = new TwoBollardEnvironment(bollardSeparation).getPlanarRegionsList();

         FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
         RobotSide initialStanceSide = RobotSide.LEFT;
         initialStanceFootPose.setY(initialStanceSide.negateIfRightSide(0.15));
         initialStanceFootPose.setX(-1.5);

         FramePose3D goalPose = new FramePose3D(worldFrame);
         goalPose.setPosition(1.5, 0.0, 0.0);

         setStartPose(initialStanceFootPose);
         setGoalPose(goalPose);
         setPlanarRegions(regions);
         setStartSide(initialStanceSide);
      }
   }

   public static class PlannerTestData
   {
      private final String name;

      private final FramePose3D goalPose = new FramePose3D();
      private final FramePose3D startPose = new FramePose3D();

      private PlanarRegionsList planarRegionsList;
      private RobotSide startSide;

      public PlannerTestData(String name)
      {
         this.name = name;
      }

      public void setGoalPose(Pose3DReadOnly goalPose)
      {
         this.goalPose.set(goalPose);
      }

      public void setStartPose(Pose3DReadOnly startPose)
      {
         this.startPose.set(startPose);
      }

      public void setPlanarRegions(PlanarRegionsList planarRegionsList)
      {
         this.planarRegionsList = planarRegionsList;
      }

      public void setStartSide(RobotSide startSide)
      {
         this.startSide = startSide;
      }

      public Point3D getStartPosition()
      {
         return new Point3D(startPose.getPosition());
      }

      public Quaternion getStartOrientation()
      {
         return new Quaternion(startPose.getOrientation());
      }

      public Point3D getGoalPosition()
      {
         return new Point3D(goalPose.getPosition());
      }

      public Quaternion getGoalOrientation()
      {
         return new Quaternion(goalPose.getOrientation());
      }

      public FramePose3D getGoalPose()
      {
         return goalPose;
      }

      public FramePose3D getStartPose()
      {
         return startPose;
      }

      public PlanarRegionsList getPlanarRegionsList()
      {
         return planarRegionsList;
      }

      public RobotSide getStartSide()
      {
         return startSide;
      }
   }
}
