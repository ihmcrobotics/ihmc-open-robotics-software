package us.ihmc.footstepPlanning.roughTerrainPlanning;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.footstepPlanning.testTools.PlanningTest;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;

public abstract class FootstepPlannerOnRoughTerrainTest implements PlanningTest
{
   private static final Random random = new Random(42747621889239430L);
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public void testOnStaircase()
   {
      testOnStaircase(new Vector3D(), true);
   }

   public void testOnStaircase(Vector3D rotationVector, boolean assertPlannerReturnedResult)
   {
      PlanarRegionsList stairCase = PlanarRegionsListExamples.generateStairCase(rotationVector);

      // define start and goal conditions
      FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
      RobotSide initialStanceSide = RobotSide.LEFT;

      FramePose3D goalPose = new FramePose3D(worldFrame);
      goalPose.setPosition(2.0, -0.2, 0.53);

      // run the test
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, stairCase, assertPlannerReturnedResult);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(stairCase, footstepPlan, goalPose);

      if(assertPlannerReturnedResult)
         assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan));
   }

   public void testWithWall(boolean assertPlannerReturnedResult)
   {
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

      // define start and goal conditions
      FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
      RobotSide initialStanceSide = RobotSide.LEFT;
      initialStanceFootPose.setY(initialStanceSide.negateIfRightSide(getParameters().getIdealFootstepWidth() / 2.0));
      initialStanceFootPose.setX(-2.0);

      FramePose3D goalPose = new FramePose3D(worldFrame);
      goalPose.setPosition(2.0, 0.0, 0.0);

      // run the test
      FootstepPlanner planner = getPlanner();
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(planner, initialStanceFootPose, initialStanceSide, goalPose, regions, assertPlannerReturnedResult);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(regions, footstepPlan, goalPose);

      if(assertPlannerReturnedResult)
         assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan));
   }

   public void testOverCinderBlockField()
   {
      testOverCinderBlockField(true);
   }

   public void testOverCinderBlockField(boolean assertPlannerReturnedResult)
   {
      double startX = 0.0;
      double startY = 0.0;
      double cinderBlockHeight = 0.15;
      double cinderBlockSize = 0.4;
      int courseWidthXInNumberOfBlocks = 21;
      int courseLengthYInNumberOfBlocks = 6;
      double heightVariation = 0.1;
      PlanarRegionsList cinderBlockField = PlanarRegionsListExamples.generateCinderBlockField(startX, startY, cinderBlockSize, cinderBlockHeight, courseWidthXInNumberOfBlocks, courseLengthYInNumberOfBlocks, heightVariation);

      FramePose3D goalPose = new FramePose3D(worldFrame);
      goalPose.setPosition(9.0, 0.0, 0.0);

      FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
      initialStanceFootPose.setPosition(0.0, -0.7, 0.0);
      RobotSide initialStanceSide = RobotSide.RIGHT;

      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, cinderBlockField, assertPlannerReturnedResult);

      if (visualize())
      {
         PlanningTestTools.visualizeAndSleep(cinderBlockField, footstepPlan, goalPose);
      }

      assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan));
   }

   public void testSteppingStones(boolean assertPlannerReturnedResult)
   {
      double pathRadius = 3.5;
      PlanarRegionsList cinderBlockField = PlanarRegionsListExamples.generateSteppingStonesEnvironment(pathRadius);

      FramePose3D goalPose = new FramePose3D(worldFrame);
      goalPose.setPosition(pathRadius + 0.5, pathRadius, 0.0);

      FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
      initialStanceFootPose.setPosition(0.0, -0.7, 0.0);
      initialStanceFootPose.appendYawRotation(0.5 * Math.PI);
      RobotSide initialStanceSide = RobotSide.RIGHT;

      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, cinderBlockField, assertPlannerReturnedResult);

      if (visualize())
      {
         PlanningTestTools.visualizeAndSleep(cinderBlockField, footstepPlan, goalPose);
      }

      assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan));
   }

   public void testStepUpsAndDownsScoringDifficult(boolean assertPlannerReturnedResult)
   {
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

      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, cinderBlockField, assertPlannerReturnedResult);

      if (visualize())
      {
         PlanningTestTools.visualizeAndSleep(cinderBlockField, footstepPlan, goalPose);
      }

      assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan));
   }

   public void testCompareAfterPitchedStep(boolean assertPlannerReturnedResult, boolean pitchCinderBack)
   {
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

      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, cinderBlockField, assertPlannerReturnedResult);

      if (visualize())
      {
         PlanningTestTools.visualizeAndSleep(cinderBlockField, footstepPlan, goalPose);
      }

      assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan, 0.06));
   }

   public void testCompareStepBeforeGap(boolean assertPlannerReturnedResult)
   {
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

      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, cinderBlockField, assertPlannerReturnedResult);

      if (visualize())
      {
         PlanningTestTools.visualizeAndSleep(cinderBlockField, footstepPlan, goalPose);
      }

      assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan));
   }

   public void testSimpleStepOnBox()
   {
      testSimpleStepOnBox(true);
   }

   public void testSimpleStepOnBox(boolean assertPlannerReturnedResult)
   {
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
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList, assertPlannerReturnedResult);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
      assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan));
   }


   public void testSimpleStepOnBoxTwo(boolean assertPlannerReturnedResult)
   {
      // create planar regions
      double stepHeight = 0.2;
      double boxSize = 1.0;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.rotate(Math.PI/4.0, Axis.Z);
      generator.translate(-6.0 + 1.0 + boxSize / 2.0, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(5.0, 5.0); // floor plane

      // define start and goal conditions
      RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
      transformGenerator.rotate(Math.PI/4.0, Axis.Z);
      RigidBodyTransform transform = transformGenerator.getRigidBodyTransformCopy();
      Point3D startPosition = new Point3D(-6.0, 0.0, 0.0);
      transform.transform(startPosition);

      FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
      initialStanceFootPose.setPosition(startPosition);
      initialStanceFootPose.setOrientation(new AxisAngle(new Vector3D(0.0, 0.0, 1.0), Math.PI/4.0));

      Point3D goalPosition = new Point3D(-6.0 + 1.0 + boxSize / 2.0, 0.0, stepHeight);
      transform.transform(goalPosition);

      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose3D goalPose = new FramePose3D(worldFrame);
      goalPose.setPosition(goalPosition);
      goalPose.setOrientation(new AxisAngle(new Vector3D(0.0, 0.0, 1.0), Math.PI/4.0));

      // run the test
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList, assertPlannerReturnedResult);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
      assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan));
   }

   public void testRandomEnvironment()
   {
      testRandomEnvironment(true);
   }

   public void testRandomEnvironment(boolean assertPlannerReturnedResult)
   {
      // define start and goal conditions
      FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose3D goalPose = new FramePose3D(worldFrame);
      initialStanceFootPose.setPosition(-6.0, 0.0, 0.0);
      goalPose.setPosition(6.0, 0.0, 0.0);

      // run the test
      PlanarRegionsList planarRegionsList = generateRandomTerrain(random);
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList, assertPlannerReturnedResult);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
   }

   public void testSimpleGaps()
   {
      testSimpleGaps(true);
   }

   public void testSimpleGaps(boolean assertPlannerReturnedResult)
   {
      // create planar regions
      double boxHeight = 0.2;
      double boxSize = 0.87;
      double gapSize = 0.2;
      int numberOfGaps = 6;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, boxHeight);

      for (int i=0; i<numberOfGaps; i++)
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
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList, assertPlannerReturnedResult);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
      assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan));
   }

   public void testPartialGaps()
   {
      testPartialGaps(true);
   }

   public void testPartialGaps(boolean assertPlannerReturnedResult)
   {
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

      // run the test
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList, assertPlannerReturnedResult);
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
      assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan));
   }

   public void testWalkingAroundBox()
   {
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

      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList,
            !visualize());
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
      assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan));
   }

   public void testSpiralStaircase()
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
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      // define start and goal conditions
      FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
      initialStanceFootPose.setPosition(0.0, 0.15, 0.0);
      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose3D goalPose = new FramePose3D(worldFrame);
      goalPose.setPosition(-0.1, 1.25, stepHeight * (circleVertices - 1));

      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList,
            !visualize());
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
      assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan));
   }

   public void testWalkingAroundHole()
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


      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      // define start and goal conditions
      FramePose3D initialStanceFootPose = new FramePose3D(worldFrame);
      initialStanceFootPose.setPosition(0.0, 0.15, 0.0);
      RobotSide initialStanceSide = RobotSide.LEFT;
      FramePose3D goalPose = new FramePose3D(worldFrame);
      goalPose.setPosition(1.3, 0.0, 0.0);

      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(getPlanner(), initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList,
            !visualize());
      if (visualize())
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose);
      assertTrue(PlanningTestTools.isGoalNextToLastStep(goalPose, footstepPlan));
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

   protected FootstepPlannerParameters getParameters()
   {
      return new DefaultFootstepPlanningParameters();
   }
}
