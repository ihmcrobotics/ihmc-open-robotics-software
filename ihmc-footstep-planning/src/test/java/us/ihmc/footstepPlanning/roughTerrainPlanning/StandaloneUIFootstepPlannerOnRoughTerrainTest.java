package us.ihmc.footstepPlanning.roughTerrainPlanning;

import javafx.application.Application;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI;
import us.ihmc.footstepPlanning.ui.StandaloneFootstepPlannerUI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.Assert.assertTrue;
import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.*;

public abstract class StandaloneUIFootstepPlannerOnRoughTerrainTest extends Application
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected static StandaloneFootstepPlannerUI ui;
   protected boolean keepUIUp = false;

   public abstract FootstepPlannerType getPlannerType();


   @ContinuousIntegrationTest(estimatedDuration = 20)
   @Test(timeout = 30000000)
   public void test()
   {
      List<String> testsToRun = getTestsToRun();
      for (String test : testsToRun)
      {
         PrintTools.info("Running test " + test);
         runTestAndAssert(testDataSets.get(test));
         PrintTools.info("Test " + test + " passed.");
         ThreadTools.sleep(100);
      }
   }



   private PlanarRegionsList generateRandomTerrain(Random random)
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

   private void runTestAndAssert(PlannerTestData testData)
   {
      Messager messager = ui.getMessager();

      submitInfoToUI(testData);

      ThreadTools.sleep(10);

      AtomicReference<Boolean> receivedPlan = new AtomicReference<>(false);
      AtomicReference<Boolean> receivedResult = new AtomicReference<>(false);
      messager.registerTopicListener(FootstepPlanTopic, request -> receivedPlan.set(true));
      messager.registerTopicListener(PlanningResultTopic, request -> receivedResult.set(true));
      AtomicReference<FootstepPlan> footstepPlanReference = messager.createInput(FootstepPlanTopic);
      AtomicReference<FootstepPlanningResult> footstepPlanningResult = messager.createInput(PlanningResultTopic);

      messager.submitMessage(ComputePathTopic, true);

      while (!receivedPlan.get() && !receivedResult.get())
      {
      }

      ThreadTools.sleep(10);

      assertTrue("Planning result is invalid, result was " + footstepPlanningResult.get(), footstepPlanningResult.get().validForExecution());
      assertTrue(PlannerTools.isGoalNextToLastStep(testData.getGoalPose(), footstepPlanReference.get()));

      if (keepUIUp)
         ThreadTools.sleepForever();
   }

   private void submitInfoToUI(PlannerTestData testData)
   {
      JavaFXMessager messager = ui.getMessager();

      messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlanarRegionDataTopic, testData.getPlanarRegionsList());
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.GoalPositionTopic, testData.getGoalPosition());
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.GoalOrientationTopic, testData.getGoalOrientation());
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.StartPositionTopic, testData.getStartPosition());
      messager.submitMessage(FootstepPlannerUserInterfaceAPI.StartOrientationTopic, testData.getStartOrientation());

      messager.submitMessage(PlannerTypeTopic, getPlannerType());
   }














   private static final String staircase = "staircase";
   private static final String wall = "wall";
   private static final String overCinderBlockField = "overCinderBlockField";
   private static final String steppingStones = "steppingStones";
   private static final String stepUpsAndDownsScoringDifficult = "stepUpsAndDownsScoringDifficult";
   private static final String stepAfterPitchDown = "stepAfterPitchDown";
   private static final String stepAfterPitchUp = "stepAfterPitchUp";
   private static final String compareStepBeforeGap = "compareStepBeforeGap";
   private static final String simpleStepOnBox = "simpleStepOnBox";
   private static final String simpleStepOnBoxTwo = "simpleStepOnBoxTwo";
   private static final String random = "random";
   private static final String simpleGaps = "simpleGaps";
   private static final String partialGaps = "partialGaps";
   private static final String box = "box";
   private static final String spiralStaircase = "spiralStaircase";
   private static final String hole = "hole";

   public List<String> getTestsToRun()
   {
      List<String> tests = new ArrayList<>();
      tests.add(staircase);
      tests.add(wall);
      tests.add(overCinderBlockField);
      tests.add(steppingStones);
      tests.add(stepUpsAndDownsScoringDifficult);
      tests.add(stepAfterPitchDown);
      tests.add(stepAfterPitchUp);
      tests.add(compareStepBeforeGap);
      tests.add(simpleStepOnBox);
      tests.add(simpleStepOnBoxTwo);
      tests.add(random);
      tests.add(simpleGaps);
//      tests.add(partialGaps);
      tests.add(box);
//      tests.add(spiralStaircase);
      tests.add(hole);

      return tests;
   }

      private static HashMap<String, PlannerTestData> testDataSets = new HashMap<>();
   {
      testDataSets.put(staircase, new StaircaseTestData());
      testDataSets.put(wall, new WallTestData());
      testDataSets.put(overCinderBlockField, new OverCinderBlockFieldTestData());
      testDataSets.put(steppingStones, new SteppingStonesTestData());
      testDataSets.put(stepUpsAndDownsScoringDifficult, new StepUpsAndDownsScoringDifficultTestData());
      testDataSets.put(stepAfterPitchUp, new StepAfterPitchUpTestData());
      testDataSets.put(stepAfterPitchDown, new StepAfterPitchDownTestData());
      testDataSets.put(compareStepBeforeGap, new CompareStepBeforeGapTestData());
      testDataSets.put(simpleStepOnBox, new SimpleStepOnBoxTestData());
      testDataSets.put(simpleStepOnBoxTwo, new SimpleStepOnBoxTwoTestData());
      testDataSets.put(random, new RandomTestData());
      testDataSets.put(simpleGaps, new SimpleGapsTestData());
      testDataSets.put(partialGaps, new PartialGapsTestData());
      testDataSets.put(box, new BoxTestData());
      testDataSets.put(spiralStaircase, new SpiralStaircaseTestData());
      testDataSets.put(hole, new HoleTestData());
   }

   private class StaircaseTestData extends PlannerTestData
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

   private class WallTestData extends PlannerTestData
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

   private class OverCinderBlockFieldTestData extends PlannerTestData
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

   private class SteppingStonesTestData extends PlannerTestData
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

   private class StepUpsAndDownsScoringDifficultTestData extends PlannerTestData
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

   private class StepAfterPitchUpTestData extends CompareAfterPitchedStep
   {
      public StepAfterPitchUpTestData()
      {
         super(stepAfterPitchUp, true);
      }
   }

   private class StepAfterPitchDownTestData extends CompareAfterPitchedStep
   {
      public StepAfterPitchDownTestData()
      {
         super(stepAfterPitchDown, false);
      }
   }

   private class CompareAfterPitchedStep extends PlannerTestData
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

   private class CompareStepBeforeGapTestData extends PlannerTestData
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

   private class SimpleStepOnBoxTestData extends PlannerTestData
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

   private class SimpleStepOnBoxTwoTestData extends PlannerTestData
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

   private class RandomTestData extends PlannerTestData
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

   private class SimpleGapsTestData extends PlannerTestData
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

   private class PartialGapsTestData extends PlannerTestData
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

   private class BoxTestData extends PlannerTestData
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

   private class SpiralStaircaseTestData extends PlannerTestData
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

   private class HoleTestData extends PlannerTestData
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

   private class PlannerTestData
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

      public double getStartOrientation()
      {
         return startPose.getYaw();
      }

      public Point3D getGoalPosition()
      {
         return new Point3D(goalPose.getPosition());
      }

      public double getGoalOrientation()
      {
         return goalPose.getYaw();
      }

      public FramePose3D getGoalPose()
      {
         return goalPose;
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
