package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepValidityMetric;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnapper;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.SimpleFootstepSnapper;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationToolkit.visualizers.FootstepVisualizer;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class FootstepGeneratorsTest
{
   private static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   private static final double GROUND_HEIGHT = 0.0;
   private static final Vector3d PLANE_NORMAL = new Vector3d(0.0, 0.0, 1.0);

   double xToAnkle = -0.15;
   double yToAnkle = 0.02;
   double zToAnkle = 0.21;
   double footForward = 0.1;
   double footBack = 0.05;
   double footSide = 0.05;
   double coefficientOfFriction = 0.0;
   FootSpoof leftContactableFoot = new FootSpoof("leftFoot", xToAnkle, yToAnkle, zToAnkle, footForward, footBack, footSide, coefficientOfFriction);
   FootSpoof rightContactableFoot = new FootSpoof("rightFoot", xToAnkle, yToAnkle, zToAnkle, footForward, footBack, footSide, coefficientOfFriction);
   SideDependentList<FootSpoof> contactableFeet = new SideDependentList<FootSpoof>();

   {
      contactableFeet.set(RobotSide.LEFT, leftContactableFoot);
      contactableFeet.set(RobotSide.RIGHT, rightContactableFoot);
   }

   SideDependentList<RigidBody> feet = new SideDependentList<RigidBody>(leftContactableFoot.getRigidBody(), rightContactableFoot.getRigidBody());
   SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<ReferenceFrame>(leftContactableFoot.getSoleFrame(), rightContactableFoot.getSoleFrame());

   double stepLength = 0.5;
   double stepWidth = 0.2;
   double maxOpenHipAngle = Math.PI / 6;
   private double maxCloseHipAngle = maxOpenHipAngle / 2;
   SimplePathParameters pathType = new SimplePathParameters(stepLength, stepWidth, 0.0, maxOpenHipAngle, maxCloseHipAngle, 0.3);
   SimplePathParameters pathType2 = new SimplePathParameters(stepLength, stepWidth, 0.0, maxOpenHipAngle, maxCloseHipAngle, 0.3); // for second segment of TwoSegmentPath

   double translationalForwardStepLength = stepLength;
   double translationalBackwardStepLength = stepLength / 4;
   double translationalSidewardStepLength = stepLength / 2;
   double translationalNominalStepWidth = stepWidth;
   double translationalMinimumStepWidth = 0.15;
   SimpleTranslationalPathParameters translationalPathType = new SimpleTranslationalPathParameters(translationalForwardStepLength,
         translationalBackwardStepLength, translationalSidewardStepLength, translationalNominalStepWidth, translationalMinimumStepWidth);

   private double initialDeltaFeetLocalX = 0.0;
   private double initialDeltaFeetLocalY = stepWidth;
   private double initialDeltaFeetYaw = 0.0;

   enum PathOrientation
   {
      FORWARD(0), REVERSE(180), LEFT(-90), RIGHT(90); // walk leftward  faces right so cw 90, walk rightward faces left so ccw 90 = +90

      private double pathOrientation; // radians, direction facing relative to path heading

      PathOrientation(double orientationDegreesRelativeToPathHeading)
      {
         pathOrientation = Math.toRadians(orientationDegreesRelativeToPathHeading);
      }

      double getOrientationRadians()
      {
         return pathOrientation;
      }

      static PathOrientation[] nonForwardValues()
      {
         return new PathOrientation[] { REVERSE, LEFT, RIGHT };
      }

      static PathOrientation[] leftRightValues()
      {
         return new PathOrientation[] { LEFT, RIGHT };
      }

      static PathOrientation[] forwardReverseValues()
      {
         return new PathOrientation[] { FORWARD, REVERSE };
      }
   }

   enum Visualization
   {
      NO_VISUALIZATION, VISUALIZE
   }

//   private Visualization allowVisualization = Visualization.VISUALIZE;    // Allow visualization for individual tests
   private Visualization allowVisualization = Visualization.NO_VISUALIZATION; // No visualizations for any tests for committing tests

   private boolean forceVisualizeAll = false;
   private double endPositionTolerance = 1e-13;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void noVisualizationsForBambooTest()
   {
      assertTrue("Do not allow visualizations for committing to bamboo", allowVisualization == Visualization.NO_VISUALIZATION);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.7)
	@Test(timeout=300000)
   public void stepInPlaceTest()
   {
//      Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double endX = startX;
      double endY = startY;

      ArrayList<Footstep> footsteps;
      String testDescription;
      String testDescriptionSpecific;

      for (double startYaw = 0; startYaw <= Math.PI * 2; startYaw += Math.PI / 8)
      {
         double endYaw = startYaw;

         for (PathOrientation pathOrientation : PathOrientation.values())
         {
            testDescription = String.format("Step in place, theta = %.2f " + pathOrientation.toString(), Math.toDegrees(startYaw));
            testDescriptionSpecific = testDescription + " , (turnStraight)";
            footsteps = testTurnStraightFootstepGenerator(testDescriptionSpecific, startX, startY, startYaw, RobotSide.RIGHT, pathOrientation, endX, endY, vis);
            assertEquals("Should have exactly 2 footsteps for step in place. " + testDescriptionSpecific, 2, footsteps.size());
            footsteps = testTurnStraightFootstepGenerator(testDescriptionSpecific, startX, startY, startYaw, RobotSide.LEFT, pathOrientation, endX, endY, vis);
            assertEquals("Should have exactly 2 footsteps for step in place. " + testDescriptionSpecific, 2, footsteps.size());
            footsteps = testTurnStraightFootstepGenerator(testDescriptionSpecific, startX, startY, startYaw, null, pathOrientation, endX, endY, vis);
            assertEquals("Should have exactly 2 footsteps for step in place. " + testDescriptionSpecific, 2, footsteps.size());

            testDescriptionSpecific = testDescription + " , (turnStraightTurn)";
            footsteps = testTurnStraightTurnFootstepGenerator(testDescriptionSpecific, startX, startY, startYaw, RobotSide.RIGHT, pathOrientation, endX, endY,
                  endYaw, vis);
            assertEquals("Should have exactly 2 footsteps for step in place. " + testDescriptionSpecific, 2, footsteps.size());
            footsteps = testTurnStraightTurnFootstepGenerator(testDescriptionSpecific, startX, startY, startYaw, RobotSide.LEFT, pathOrientation, endX, endY,
                  endYaw, vis);
            assertEquals("Should have exactly 2 footsteps for step in place. " + testDescriptionSpecific, 2, footsteps.size());
            footsteps = testTurnStraightTurnFootstepGenerator(testDescriptionSpecific, startX, startY, startYaw, null, pathOrientation, endX, endY, endYaw, vis);
            assertEquals("Should have exactly 2 footsteps for step in place. " + testDescriptionSpecific, 2, footsteps.size());

            testDescriptionSpecific = testDescription + " , (twoSegment)";
            footsteps = testTwoSegmentFootstepGenerator(testDescriptionSpecific, startX, startY, startYaw, RobotSide.RIGHT, pathOrientation, startX, startY,
                  null, pathOrientation, endX, endY, vis);
            assertEquals("Should have exactly 2 footsteps for step in place. " + testDescriptionSpecific, 2, footsteps.size());
            footsteps = testTwoSegmentFootstepGenerator(testDescriptionSpecific, startX, startY, startYaw, RobotSide.LEFT, pathOrientation, startX, startY,
                  null, pathOrientation, endX, endY, vis);
            assertEquals("Should have exactly 2 footsteps for step in place. " + testDescriptionSpecific, 2, footsteps.size());
            footsteps = testTwoSegmentFootstepGenerator(testDescriptionSpecific, startX, startY, startYaw, null, pathOrientation, startX, startY, null,
                  pathOrientation, endX, endY, vis);
            assertEquals("Should have exactly 2 footsteps for step in place. " + testDescriptionSpecific, 2, footsteps.size());

         }

         testDescription = String.format("Step in place, theta = %.2f", Math.toDegrees(startYaw));

         testDescriptionSpecific = testDescription + " , (translation)";
         footsteps = testTranslationFootstepGenerator(testDescriptionSpecific, startX, startY, startYaw, null, endX, endY, vis);
         assertEquals("Should have exactly 2 footsteps for step in place. " + testDescriptionSpecific, 2, footsteps.size());
         footsteps = testTranslationFootstepGenerator(testDescriptionSpecific, startX, startY, startYaw, RobotSide.LEFT, endX, endY, vis);
         assertEquals("Should have exactly 2 footsteps for step in place. " + testDescriptionSpecific, 2, footsteps.size());
         footsteps = testTranslationFootstepGenerator(testDescriptionSpecific, startX, startY, startYaw, RobotSide.RIGHT, endX, endY, vis);
         assertEquals("Should have exactly 2 footsteps for step in place. " + testDescriptionSpecific, 2, footsteps.size());
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void walkForwardsTest()
   {
      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double startYaw = 0.0;
      double endX = 3.0;
      double endY = 0.0;

      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      testTurnStraightFootstepGenerator("Walk forwards, right stance first", startX, startY, startYaw, RobotSide.RIGHT, endX, endY, vis);
      testTurnStraightFootstepGenerator("Walk forwards, left stance first", startX, startY, startYaw, RobotSide.LEFT, endX, endY, vis);
      testTurnStraightFootstepGenerator("Walk forwards, auto stance", startX, startY, startYaw, null, endX, endY, vis);
      double endYaw = startYaw;
      testTurnStraightTurnFootstepGenerator("Walk forwards, right stance first", startX, startY, startYaw, RobotSide.RIGHT, endX, endY, endYaw, vis);
      testTurnStraightTurnFootstepGenerator("Walk forwards, left stance first", startX, startY, startYaw, RobotSide.LEFT, endX, endY, endYaw, vis);
      testTurnStraightTurnFootstepGenerator("Walk forwards, auto stance", startX, startY, startYaw, null, endX, endY, endYaw, vis);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout=300000)
   public void walkLeftwardTest()
   {
      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      PathOrientation pathDirection = PathOrientation.LEFT;
      double startX = 0.0;
      double startY = 0.0;
      double startYaw = Math.PI / 2; // pathDirection.getOrientationRadians();
      double endX = -3.0;
      double endY = 0.0;

      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      String testDescription = "Walk " + pathDirection.toString();
      testTurnStraightFootstepGenerator("RStance, turn staight, " + testDescription, startX, startY, startYaw, RobotSide.RIGHT, pathDirection, endX, endY, vis);
      testTurnStraightFootstepGenerator("LStance, turn staight, " + testDescription, startX, startY, startYaw, RobotSide.LEFT, pathDirection, endX, endY, vis);
      testTurnStraightFootstepGenerator("Auto Stance, turn staight, " + testDescription, startX, startY, startYaw, null, pathDirection, endX, endY, vis);
      double endYaw = startYaw;
      testTurnStraightTurnFootstepGenerator("RStance, turn staight turn, " + testDescription, startX, startY, startYaw, RobotSide.RIGHT, pathDirection, endX,
            endY, endYaw, vis);
      testTurnStraightTurnFootstepGenerator("LStance, turn staight turn, " + testDescription, startX, startY, startYaw, RobotSide.LEFT, pathDirection, endX,
            endY, endYaw, vis);
      testTurnStraightTurnFootstepGenerator("Auto Stance, turn staight turn, " + testDescription, startX, startY, startYaw, null, pathDirection, endX, endY,
            endYaw, vis);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void walkRightwardTest()
   {
      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      PathOrientation pathDirection = PathOrientation.RIGHT;
      double startX = 0.0;
      double startY = 0.0;
      double startYaw = pathDirection.getOrientationRadians();
      double endX = 3.0;
      double endY = 0.0;

      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      String testDescription = "Walk " + pathDirection.toString();
      testTurnStraightFootstepGenerator("RStance, turn staight, " + testDescription, startX, startY, startYaw, RobotSide.RIGHT, pathDirection, endX, endY, vis);
      testTurnStraightFootstepGenerator("LStance, turn staight, " + testDescription, startX, startY, startYaw, RobotSide.LEFT, pathDirection, endX, endY, vis);
      testTurnStraightFootstepGenerator("Auto Stance, turn staight, " + testDescription, startX, startY, startYaw, null, pathDirection, endX, endY, vis);
      double endYaw = startYaw;
      testTurnStraightTurnFootstepGenerator("RStance, turn staight turn, " + testDescription, startX, startY, startYaw, RobotSide.RIGHT, pathDirection, endX,
            endY, endYaw, vis);
      testTurnStraightTurnFootstepGenerator("LStance, turn staight turn, " + testDescription, startX, startY, startYaw, RobotSide.LEFT, pathDirection, endX,
            endY, endYaw, vis);
      testTurnStraightTurnFootstepGenerator("Auto Stance, turn staight turn, " + testDescription, startX, startY, startYaw, null, pathDirection, endX, endY,
            endYaw, vis);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void walkBackwardTest()
   {
      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      PathOrientation pathDirection = PathOrientation.REVERSE;
      double startX = 0.0;
      double startY = 0.0;
      double startYaw = pathDirection.getOrientationRadians();
      double endX = 3.0;
      double endY = 0.0;

      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      String testDescription = "Walk " + pathDirection.toString();
      testTurnStraightFootstepGenerator("RStance, turn staight, " + testDescription, startX, startY, startYaw, RobotSide.RIGHT, pathDirection, endX, endY, vis);
      testTurnStraightFootstepGenerator("LStance, turn staight, " + testDescription, startX, startY, startYaw, RobotSide.LEFT, pathDirection, endX, endY, vis);
      testTurnStraightFootstepGenerator("Auto Stance, turn staight, " + testDescription, startX, startY, startYaw, null, pathDirection, endX, endY, vis);
      double endYaw = startYaw;
      testTurnStraightTurnFootstepGenerator("RStance, turn staight turn, " + testDescription, startX, startY, startYaw, RobotSide.RIGHT, pathDirection, endX,
            endY, endYaw, vis);
      testTurnStraightTurnFootstepGenerator("LStance, turn staight turn, " + testDescription, startX, startY, startYaw, RobotSide.LEFT, pathDirection, endX,
            endY, endYaw, vis);
      testTurnStraightTurnFootstepGenerator("Auto Stance, turn staight turn, " + testDescription, startX, startY, startYaw, null, pathDirection, endX, endY,
            endYaw, vis);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void walkForwardsDifferentStartTest()
   {
      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = -0.5;
      double startY = 0.5;
      double startYaw = 0.0;
      double endX = 3.0;
      double endY = startY;

      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      testTurnStraightFootstepGenerator("Forwards not on zero, right stance first", startX, startY, startYaw, RobotSide.RIGHT, endX, endY, vis);
      testTurnStraightFootstepGenerator("Forwards not on zero, left stance first", startX, startY, startYaw, RobotSide.LEFT, endX, endY, vis);
      testTurnStraightFootstepGenerator("Forwards not on zero, auto stance first", startX, startY, startYaw, null, endX, endY, vis);

      // Visualization vis2 = Visualization.VISUALIZE;
      Visualization vis2 = Visualization.NO_VISUALIZATION;

      double endYaw = startYaw;
      testTurnStraightTurnFootstepGenerator("Forwards not on zero, right stance first", startX, startY, startYaw, RobotSide.RIGHT, endX, endY, endYaw, vis2);
      testTurnStraightTurnFootstepGenerator("Forwards not on zero, left stance first", startX, startY, startYaw, RobotSide.LEFT, endX, endY, endYaw, vis2);
      testTurnStraightTurnFootstepGenerator("Forwards not on zero, auto stance first", startX, startY, startYaw, null, endX, endY, endYaw, vis2);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout=300000)
   public void turningThenStraightFootstepGenerator_noTurnVaryingStartOrientationTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = -0.5;
      double startY = 0.5;
      double distance = 3;
      double yawDelta = Math.PI / 8.0;
      for (double angle = yawDelta; angle < Math.PI * 2 + yawDelta; angle += yawDelta)
      {
         for (PathOrientation pathOrientation : PathOrientation.values())
         {
            double startYaw = angle + pathOrientation.getOrientationRadians();
            double endX = startX + distance * Math.cos(angle);
            double endY = startY + distance * Math.sin(angle);

            testTurnStraightFootstepGenerator(pathOrientation + ", right stance, path angle =  " + Math.toDegrees(angle), startX, startY, startYaw,
                  RobotSide.RIGHT, pathOrientation, endX, endY, vis);
            testTurnStraightFootstepGenerator(pathOrientation + ", left stance, path angle =  " + Math.toDegrees(angle), startX, startY, startYaw,
                  RobotSide.LEFT, pathOrientation, endX, endY, vis);
            testTurnStraightFootstepGenerator(pathOrientation + ", auto stance, path angle =  " + Math.toDegrees(angle), startX, startY, startYaw, null,
                  pathOrientation, endX, endY, vis);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void turningThenStraightFootstepGenerator_varyInitialTurnsThenForwardsToSameEndPointTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = -0.5;
      double startY = 0.5;
      double endX = 1.5;
      double endY = -3.0;
      double yawDelta = Math.PI / 8.0;
      for (double angle = yawDelta; angle < Math.PI * 2 + yawDelta; angle += yawDelta)
      {
         double startYaw = angle;

         testTurnStraightFootstepGenerator("Forwards, right stance, turn from yaw =  " + Math.toDegrees(angle), startX, startY, startYaw, RobotSide.RIGHT,
               endX, endY, vis);
         testTurnStraightFootstepGenerator("Forwards, left stance, turn from yaw =  " + Math.toDegrees(angle), startX, startY, startYaw, RobotSide.LEFT, endX,
               endY, vis);
         testTurnStraightFootstepGenerator("Forwards, auto stance, turn from yaw =  " + Math.toDegrees(angle), startX, startY, startYaw, null, endX, endY, vis);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout=300000)
   public void turningThenStraightFootstepGenerator_varyInitialTurnsThenNonForwardsToSameEndPointTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = -0.5;
      double startY = 0.5;
      double endX = 1.5;
      double endY = -3.0;
      double yawDelta = Math.PI / 8.0;
      for (double angle = yawDelta; angle < Math.PI * 2 + yawDelta; angle += yawDelta)
      {
         double startYaw = angle;

         for (PathOrientation pathOrientation : PathOrientation.nonForwardValues())
         {
            String message = pathOrientation.toString() + ", start yaw =  " + Math.toDegrees(angle);
            testTurnStraightFootstepGenerator("RStance, " + message, startX, startY, startYaw, RobotSide.RIGHT, pathOrientation, endX, endY, vis);
            testTurnStraightFootstepGenerator("LStance, " + message, startX, startY, startYaw, RobotSide.LEFT, pathOrientation, endX, endY, vis);
            testTurnStraightFootstepGenerator("AutoStance, " + message, startX, startY, startYaw, null, pathOrientation, endX, endY, vis);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout=300000)
   public void smallTurnWithSmallDisplacementTest()
   {
//        Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0;
      double startY = 0;
      double yawDelta = maxOpenHipAngle / 2 - 1e-14;
      double r = stepLength * .5;
      double endX = r * Math.cos(yawDelta);
      double endY = r * Math.sin(yawDelta);

      ArrayList<Footstep> footsteps;

      for (PathOrientation pathOrientation : PathOrientation.values())
      {
         if (pathOrientation == PathOrientation.REVERSE)
         {
            endY = -endY;
            yawDelta = -yawDelta;
         }

         double startYaw = pathOrientation.getOrientationRadians();
         String message = pathOrientation.toString() + " small step with small turn";
         String testDescription = "AutoStance TS, " + message;
         footsteps = testTurnStraightFootstepGenerator(testDescription, startX, startY, startYaw, null, pathOrientation, endX, endY, vis);

         // Should step straight to final end point with no intermediate steps for test to be valid (e.g. testing the right scenario)
         // This is a check for the test not a test for the generator
         assertEquals("Should have only two steps for test: " + testDescription, 2, footsteps.size());

         testDescription = "AutoStance TST, " + message;
         footsteps = testTurnStraightTurnFootstepGenerator(testDescription, startX, startY, startYaw, null, pathOrientation, endX, endY, yawDelta + startYaw,
               vis);
         assertEquals("Should have only two steps for test: " + testDescription, 2, footsteps.size());
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout=300000)
   public void negativePathS_LeftRightPathsWithSmallTurnAndLargerInitialStance()
   {
      //    Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0;
      double startY = 0;
      double yawDelta = maxOpenHipAngle * 2.9 / 3;
      double r = 2.5 * stepLength;
      double endX = r * Math.cos(yawDelta);
      double endY = r * Math.sin(yawDelta);

      initialDeltaFeetLocalY = stepWidth + 0.1 * stepLength;

      PathOrientation pathOrientation = PathOrientation.RIGHT;

      double startYaw = pathOrientation.getOrientationRadians();
      String message = pathOrientation.toString() + " small turn";
      String testDescription = "AutoStance TS, " + message;
      testTurnStraightFootstepGenerator(testDescription, startX, startY, startYaw, null, pathOrientation, endX, endY, vis);
      testDescription = "AutoStance TST, " + message;
      testTurnStraightTurnFootstepGenerator(testDescription, startX, startY, startYaw, null, pathOrientation, endX, endY, yawDelta + startYaw, vis);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout=300000)
   public void TurnStraightTurn_turnInPlaceTest()
   {
      // Visualization visr = Visualization.VISUALIZE;
      Visualization visr = Visualization.NO_VISUALIZATION;

      // Visualization visl = Visualization.VISUALIZE;
      Visualization visl = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.2;
      double startY = -0.75;
      double endX = startX; // if +.1 then passes!
      double endY = startY;
      double startYaw = Math.toRadians(30);
      double yawDelta = Math.PI / 8.0;
      double endTest = Math.PI * 2 + yawDelta;
      for (double angle = yawDelta; angle < endTest; angle += yawDelta)
      {
         double endYaw = angle;

         for (PathOrientation pathOrientation : PathOrientation.values())
         {
            String description = String.format("Turn in place %.2f to %.2f, Walk " + pathOrientation.toString(), Math.toDegrees(startYaw),
                  Math.toDegrees(angle));
            testTurnStraightTurnFootstepGenerator("RStance, " + description, startX, startY, startYaw, RobotSide.RIGHT, pathOrientation, endX, endY, endYaw,
                  visr);
            testTurnStraightTurnFootstepGenerator("LStance, " + description, startX, startY, startYaw, RobotSide.LEFT, pathOrientation, endX, endY, endYaw,
                  visl);
            testTurnStraightTurnFootstepGenerator("AutoStance, " + description, startX, startY, startYaw, null, pathOrientation, endX, endY, endYaw, visl);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void TurnStraightTurn_varyInitialTurnsToSameFinalPoseTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = -0.5;
      double startY = -0.75;
      double endX = 3.0;
      double endY = 0.25;
      double endYaw = Math.toRadians(90);
      double yawDelta = Math.PI / 8.0;
      for (double angle = yawDelta; angle < Math.PI * 2 + yawDelta; angle += yawDelta)
      {
         double startYaw = angle;

         String description = String.format("Turn from %.2f and forward to same final pose", Math.toDegrees(startYaw));
         testTurnStraightTurnFootstepGenerator("RStance, " + description, startX, startY, startYaw, RobotSide.RIGHT, endX, endY, endYaw, vis);
         testTurnStraightTurnFootstepGenerator("LStance, " + description, startX, startY, startYaw, RobotSide.LEFT, endX, endY, endYaw, vis);
         testTurnStraightTurnFootstepGenerator("AutoStance, " + description, startX, startY, startYaw, null, endX, endY, endYaw, vis);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout=300000)
   public void TurnStraightTurn_nonForwardTestInitialTurnsNoFinalTurnsToSameEndPointTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = -0.5;
      double startY = -0.75;
      double endX = 3.0;
      double endY = 0.25;
      double pathDirection = Math.atan2(endY - startY, endX - startX);

      double yawDelta = Math.PI / 8.0;
      for (double angle = yawDelta; angle < Math.PI * 2 + yawDelta; angle += yawDelta)
      {
         double startYaw = angle;

         for (PathOrientation pathOrientation : PathOrientation.nonForwardValues())
         {
            double endYaw = pathDirection + pathOrientation.getOrientationRadians();
            String description = String.format("Turn from %.2f " + pathOrientation.toString() + " to same final position", Math.toDegrees(startYaw));
            testTurnStraightTurnFootstepGenerator("RStance, " + description, startX, startY, startYaw, RobotSide.RIGHT, pathOrientation, endX, endY, endYaw,
                  vis);
            testTurnStraightTurnFootstepGenerator("LStance, " + description, startX, startY, startYaw, RobotSide.LEFT, pathOrientation, endX, endY, endYaw, vis);
            testTurnStraightTurnFootstepGenerator("AutoStance, " + description, startX, startY, startYaw, null, pathOrientation, endX, endY, endYaw, vis);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void TurnStraightTurn_varyFinalOrientationOnlyTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = -0.5;
      double startY = -0.75;
      double endX = 3.0;
      double endY = 0.25;
      double startYaw = Math.toRadians(240);
      double yawDelta = Math.PI / 8.0;
      for (double angle = yawDelta; angle < Math.PI * 2 + yawDelta; angle += yawDelta)
      {
         double endYaw = angle;

         String description = String.format("End facing %.2f", Math.toDegrees(endYaw));
         testTurnStraightTurnFootstepGenerator("RStance, " + description, startX, startY, startYaw, RobotSide.RIGHT, endX, endY, endYaw, vis);
         testTurnStraightTurnFootstepGenerator("LStance, " + description, startX, startY, startYaw, RobotSide.LEFT, endX, endY, endYaw, vis);
         testTurnStraightTurnFootstepGenerator("AutoStance, " + description, startX, startY, startYaw, null, endX, endY, endYaw, vis);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.5)
	@Test(timeout=300000)
   public void TurnStraightTurn_varyFinalOrientationOnlyNonForwardTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = -0.5;
      double startY = -0.75;
      double endX = 3.0;
      double endY = 0.25;
      double pathDirection = Math.atan2(endY - startY, endX - startX);

      double yawDelta = Math.PI / 8.0;
      for (double angle = yawDelta; angle < Math.PI * 2 + yawDelta; angle += yawDelta)
      {
         double endYaw = angle;

         for (PathOrientation pathOrientation : PathOrientation.nonForwardValues())
         {
            double startYaw = pathDirection + pathOrientation.getOrientationRadians();
            String description = String.format(pathOrientation.toString() + " End facing %.2f", Math.toDegrees(endYaw));
            testTurnStraightTurnFootstepGenerator("RStance, " + description, startX, startY, startYaw, RobotSide.RIGHT, pathOrientation, endX, endY, endYaw,
                  vis);
            testTurnStraightTurnFootstepGenerator("LStance, " + description, startX, startY, startYaw, RobotSide.LEFT, pathOrientation, endX, endY, endYaw, vis);
            testTurnStraightTurnFootstepGenerator("AutoStance, " + description, startX, startY, startYaw, null, pathOrientation, endX, endY, endYaw, vis);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout=300000)
   public void TurnStraightTurn_varyFinalPositionWithSameOrientationsTest()
   {
      // Visualization visr = Visualization.VISUALIZE;
      Visualization visr = Visualization.NO_VISUALIZATION;

      // Visualization visl = Visualization.VISUALIZE;
      Visualization visl = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = -0.5;
      double startY = 0.5;
      double distance = 3;
      double startYaw = Math.random() * 2 * Math.PI;
      double endYaw = Math.random() * 2 * Math.PI;

      double yawDelta = Math.PI / 8.0;
      double endRange = Math.PI * 2 + yawDelta;
      for (double angle = yawDelta; angle < endRange; angle += yawDelta)
      {
         // double angle = Math.toRadians(45.0);
         // startYaw = Math.toRadians(179.48);
         // endYaw = Math.toRadians(44.65);

         double endX = startX + distance * Math.cos(angle);
         double endY = startY + distance * Math.sin(angle);

         for (PathOrientation pathOrientation : PathOrientation.values())
         {
            String description = String.format(pathOrientation.toString() + " Path direction = %.2f, const start %.2f, const end %.2f", Math.toDegrees(angle),
                  Math.toDegrees(startYaw), Math.toDegrees(endYaw));
            testTurnStraightTurnFootstepGenerator("RStance, " + description, startX, startY, startYaw, RobotSide.RIGHT, pathOrientation, endX, endY, endYaw,
                  visr);
            testTurnStraightTurnFootstepGenerator("LStance, " + description, startX, startY, startYaw, RobotSide.LEFT, pathOrientation, endX, endY, endYaw,
                  visl);
            testTurnStraightTurnFootstepGenerator("AutoStance, " + description, startX, startY, startYaw, null, pathOrientation, endX, endY, endYaw, visl);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void TurnStraightTurn_finalYawCloseToPathAngleTest()
   {
      // Visualization visr = Visualization.VISUALIZE;
      Visualization visr = Visualization.NO_VISUALIZATION;

      // Visualization visl = Visualization.VISUALIZE;
      Visualization visl = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = -0.5;
      double startY = 0.5;
      double distance = 3;

      double angle = Math.toRadians(45.0);
      double startYaw = Math.toRadians(179.48);
      double endYaw = Math.toRadians(44.65);

      double endX = startX + distance * Math.cos(angle);
      double endY = startY + distance * Math.sin(angle);

      String description = String.format("Path direction = %.2f, start yaw %.2f, end yaw %.2f", Math.toDegrees(angle), Math.toDegrees(startYaw),
            Math.toDegrees(endYaw));
      testTurnStraightTurnFootstepGenerator("RStance, " + description, startX, startY, startYaw, RobotSide.RIGHT, endX, endY, endYaw, visr);
      testTurnStraightTurnFootstepGenerator("LStance, " + description, startX, startY, startYaw, RobotSide.LEFT, endX, endY, endYaw, visl);
      testTurnStraightTurnFootstepGenerator("AutoStance, " + description, startX, startY, startYaw, null, endX, endY, endYaw, visl);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout=300000)
   public void TurnStraightTurn_180PathVaryingOrientationsCheckMiddleOrientationTest()
   {
      // Visualization visr = Visualization.VISUALIZE;
      Visualization visr = Visualization.NO_VISUALIZATION;

      // Visualization visl = Visualization.VISUALIZE;
      Visualization visl = Visualization.NO_VISUALIZATION;

      // Visualization visa = Visualization.VISUALIZE;
      Visualization visa = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = -0.5;
      double startY = 0.5;
      double distance = 3;
      double endX = startX - distance;
      double endY = startY;
      double yawDelta = Math.PI / 8.0;

      double endRange = Math.PI * 2 + yawDelta;

      ArrayList<Footstep> footsteps;
      for (double angle = yawDelta; angle < endRange; angle += yawDelta)
      {
         double startYaw = angle;
         double endYaw = angle;

         for (PathOrientation pathOrientation : PathOrientation.values())
         {
            String description = String.format(pathOrientation.toString() + " 180 Path direction, yaw start = yaw end = %.2f", Math.toDegrees(angle),
                  Math.toDegrees(endYaw));
            String testDescription = "RStance, " + description;
            footsteps = testTurnStraightTurnFootstepGenerator(testDescription, startX, startY, startYaw, RobotSide.RIGHT, pathOrientation, endX, endY, endYaw,
                  visr);
            double expectedMidPathStepYaw = Math.PI + pathOrientation.getOrientationRadians();
            assertMiddleStepsPointingCorrectly(testDescription + " Path orientation check", footsteps, expectedMidPathStepYaw);

            testDescription = "LStance, " + description;
            footsteps = testTurnStraightTurnFootstepGenerator(testDescription, startX, startY, startYaw, RobotSide.LEFT, pathOrientation, endX, endY, endYaw,
                  visl);
            assertMiddleStepsPointingCorrectly(testDescription + " Path orientation check", footsteps, expectedMidPathStepYaw);

            testDescription = "AutoStance, " + description;
            footsteps = testTurnStraightTurnFootstepGenerator(testDescription, startX, startY, startYaw, null, pathOrientation, endX, endY, endYaw, visa);
            assertMiddleStepsPointingCorrectly(testDescription + " Path orientation check", footsteps, expectedMidPathStepYaw);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 3.7)
	@Test(timeout=300000)
   public void TwoSegment_RandomTransitionTest()
   {
//       Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      Random random = new Random(1234L);
      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      for (int randTest = 0; randTest < 25; randTest++)
      {
         double startX = random.nextDouble();
         double startY = random.nextDouble();
         double startYaw = random.nextDouble() * 2 * Math.PI;
         double midX = random.nextDouble();
         double midY = random.nextDouble();
         double endX = random.nextDouble();
         double endY = random.nextDouble();

         for (PathOrientation pathOrientation1 : PathOrientation.values())
         {
            for (PathOrientation pathOrientation2 : PathOrientation.values())
            {
               String testMessage = String.format("Rand2seg, " + pathOrientation1.toString() + " then " + pathOrientation2.toString()
                     + ", xy(Yaw) %.2f, %.2f, %.2f, to %.2f, %.2f, to %.2f, %.2f", startX, startY, startYaw, midX, midY, endX, endY);
               testTwoSegmentFootstepGenerator("Rstance " + testMessage, startX, startY, startYaw, RobotSide.RIGHT, pathOrientation1, midX, midY, null,
                     pathOrientation2, endX, endY, vis);
               testTwoSegmentFootstepGenerator("Lstance " + testMessage, startX, startY, startYaw, RobotSide.LEFT, pathOrientation1, midX, midY, null,
                     pathOrientation2, endX, endY, vis);
               testTwoSegmentFootstepGenerator("RRstance " + testMessage, startX, startY, startYaw, RobotSide.RIGHT, pathOrientation1, midX, midY,
                     RobotSide.RIGHT, pathOrientation2, endX, endY, vis);
               testTwoSegmentFootstepGenerator("LLstance " + testMessage, startX, startY, startYaw, RobotSide.LEFT, pathOrientation1, midX, midY,
                     RobotSide.LEFT, pathOrientation2, endX, endY, vis);
               testTwoSegmentFootstepGenerator("RLstance " + testMessage, startX, startY, startYaw, RobotSide.RIGHT, pathOrientation1, midX, midY,
                     RobotSide.LEFT, pathOrientation2, endX, endY, vis);
               testTwoSegmentFootstepGenerator("LRstance " + testMessage, startX, startY, startYaw, RobotSide.LEFT, pathOrientation1, midX, midY,
                     RobotSide.RIGHT, pathOrientation2, endX, endY, vis);
               testTwoSegmentFootstepGenerator("Autostance " + testMessage, startX, startY, startYaw, null, pathOrientation1, midX, midY, null,
                     pathOrientation2, endX, endY, vis);
            }
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void TwoSegment_MixedPathOrientationsTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double midX = 0.0;
      double midY = 2.0;
      double endX = 2.0;
      double endY = 2.0;

      for (PathOrientation pathOrientation1 : PathOrientation.values())
      {
         double startYaw = pathOrientation1.getOrientationRadians() + Math.PI / 2;

         for (PathOrientation pathOrientation2 : PathOrientation.values())
         {
            String testMessage = String.format("Rstance, Rand2seg, " + pathOrientation1.toString() + " then " + pathOrientation2.toString()
                  + ", xy(Yaw) %.2f, %.2f, %.2f, to %.2f, %.2f, to %.2f, %.2f", startX, startY, startYaw, midX, midY, endX, endY);
            ArrayList<Footstep> footsteps = testTwoSegmentFootstepGenerator(testMessage, startX, startY, startYaw, RobotSide.RIGHT, pathOrientation1, midX,
                  midY, null, pathOrientation2, endX, endY, vis);

            // initial yaw chosen so that there is no initial rotation: correct orientation of 3rd and 4th steps should be the same if path yaw correct
            double expectedStartYaw = startYaw;
            FrameOrientation2d expectedStartOrientation = new FrameOrientation2d(WORLD_FRAME, expectedStartYaw);
            assertStepIsPointingCorrectly("Path yaw test on third step: " + testMessage, footsteps.get(2), expectedStartOrientation);
            assertStepIsPointingCorrectly("Path yaw test on fourth step: " + testMessage, footsteps.get(3), expectedStartOrientation);

            // no rotation at end, so correct yaw achieved by correct path orientation
            // Note that this is a redundant test to ensure testTwoSegmentFootstepGenerator test is correct as well...
            double expectedEndYaw = pathOrientation2.getOrientationRadians();
            FrameOrientation2d expectedEndOrientation = new FrameOrientation2d(WORLD_FRAME, expectedEndYaw);
            assertLastTwoStepsPointingCorrectly("Path yaw test on last two steps: " + testMessage, footsteps, expectedEndOrientation);

         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.5)
	@Test(timeout=300000)
   public void VaryingLengthTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double endY = 0.0;

      for (PathOrientation pathDirection : PathOrientation.values())
      {
         double startYaw = pathDirection.getOrientationRadians();
         double endYaw = startYaw;

         for (double endX = 0.0; endX <= 2 * stepLength; endX += stepLength / 10)
         {
            String testDescription = "Walk " + pathDirection.toString() + " " + String.format("%.2f", endX);
            testTurnStraightFootstepGenerator(testDescription + " RStance, turn staight", startX, startY, startYaw, RobotSide.RIGHT, pathDirection, endX, endY,
                  vis);
            testTurnStraightFootstepGenerator(testDescription + " LStance, turn staight", startX, startY, startYaw, RobotSide.LEFT, pathDirection, endX, endY,
                  vis);
            testTurnStraightFootstepGenerator(testDescription + " AutoStance, turn staight", startX, startY, startYaw, null, pathDirection, endX, endY, vis);
            testTurnStraightTurnFootstepGenerator(testDescription + " RStance, turn staight turn", startX, startY, startYaw, RobotSide.RIGHT, pathDirection,
                  endX, endY, endYaw, vis);
            testTurnStraightTurnFootstepGenerator(testDescription + " LStance, turn staight turn", startX, startY, startYaw, RobotSide.LEFT, pathDirection,
                  endX, endY, endYaw, vis);
            testTurnStraightTurnFootstepGenerator(testDescription + " AutoStance, turn staight turn", startX, startY, startYaw, null, pathDirection, endX,
                  endY, endYaw, vis);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void ForwardAutoStanceSideSelectionTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double pathR = 3.0;
      ArrayList<Footstep> footsteps;

      PathOrientation pathDirection = PathOrientation.FORWARD;

      double startYaw = pathDirection.getOrientationRadians();
      double endYaw = startYaw;

      double yawStep = 2 * maxOpenHipAngle / 10;
      for (double pathYaw = -maxOpenHipAngle + yawStep; pathYaw <= (maxOpenHipAngle - yawStep); pathYaw += yawStep)
      {
         double endX = pathR * Math.cos(pathYaw);
         double endY = pathR * Math.sin(pathYaw);

         RobotSide expectedSide = null;
         if (pathYaw > yawStep / 2)
            expectedSide = RobotSide.RIGHT;
         else if (pathYaw < -yawStep / 2)
            expectedSide = RobotSide.LEFT;
         else
            expectedSide = null;

         String testDescription = pathDirection.toString() + " " + String.format("angle = %.2f", Math.toDegrees(pathYaw));
         footsteps = testTurnStraightFootstepGenerator(testDescription + " AutoStance, turn staight", startX, startY, startYaw, null, pathDirection, endX,
               endY, vis);
         assertStepSide(testDescription, footsteps.get(0), expectedSide);
         footsteps = testTurnStraightTurnFootstepGenerator(testDescription + " AutoStance, turn staight turn", startX, startY, startYaw, null, pathDirection,
               endX, endY, endYaw, vis);
         assertStepSide(testDescription, footsteps.get(0), expectedSide);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void BackwardAutoStanceSideSelectionTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double pathR = 3.0;
      ArrayList<Footstep> footsteps;

      PathOrientation pathDirection = PathOrientation.REVERSE;

      double startYaw = pathDirection.getOrientationRadians();
      double endYaw = startYaw;

      double yawStep = 2 * maxOpenHipAngle / 10;
      for (double pathYaw = -maxOpenHipAngle + yawStep; pathYaw <= (maxOpenHipAngle - yawStep); pathYaw += yawStep)
      {
         double endX = pathR * Math.cos(pathYaw);
         double endY = pathR * Math.sin(pathYaw);

         RobotSide expectedSide = null;
         if (pathYaw > yawStep / 2)
            expectedSide = RobotSide.LEFT;
         else if (pathYaw < -yawStep / 2)
            expectedSide = RobotSide.RIGHT;
         else
            expectedSide = null;

         String testDescription = pathDirection.toString() + " " + String.format("angle = %.2f", pathYaw);
         footsteps = testTurnStraightFootstepGenerator(testDescription + " AutoStance, turn staight", startX, startY, startYaw, null, pathDirection, endX,
               endY, vis);
         assertStepSide(testDescription, footsteps.get(0), expectedSide);
         footsteps = testTurnStraightTurnFootstepGenerator(testDescription + " AutoStance, turn staight turn", startX, startY, startYaw, null, pathDirection,
               endX, endY, endYaw, vis);
         assertStepSide(testDescription, footsteps.get(0), expectedSide);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void LeftRightAutoStanceSideSelectionTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double pathR = 3.0;
      ArrayList<Footstep> footsteps;

      for (PathOrientation pathDirection : PathOrientation.leftRightValues())
      {
         double startYaw = pathDirection.getOrientationRadians();
         double endYaw = startYaw;

         double yawStep = 2 * maxOpenHipAngle / 10;
         for (double pathYaw = -maxOpenHipAngle + yawStep; pathYaw <= (maxOpenHipAngle - yawStep); pathYaw += yawStep)
         {
            double endX = pathR * Math.cos(pathYaw);
            double endY = pathR * Math.sin(pathYaw);

            RobotSide expectedSide = null;
            if (pathDirection == PathOrientation.LEFT)
               expectedSide = RobotSide.LEFT;
            else
               expectedSide = RobotSide.RIGHT;

            String testDescription = pathDirection.toString() + " " + String.format("angle = %.2f", pathYaw);
            footsteps = testTurnStraightFootstepGenerator(testDescription + " AutoStance, turn staight", startX, startY, startYaw, null, pathDirection, endX,
                  endY, vis);
            assertStepSide(testDescription, footsteps.get(0), expectedSide);
            footsteps = testTurnStraightTurnFootstepGenerator(testDescription + " AutoStance, turn staight turn", startX, startY, startYaw, null,
                  pathDirection, endX, endY, endYaw, vis);
            assertStepSide(testDescription, footsteps.get(0), expectedSide);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void turningAutoStanceSideSelectionTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double pathR = 3.0;
      ArrayList<Footstep> footsteps;

      for (PathOrientation pathDirection : PathOrientation.values())
      {
         double endYaw = pathDirection.getOrientationRadians();
         double endX = pathR;
         double endY = 0.0;

         double yawStep = 2 * maxOpenHipAngle;
         for (int sign = -1; sign < 2; sign += 2)
         {
            double startYaw = pathDirection.getOrientationRadians() + sign * yawStep;

            RobotSide expectedSide = null;
            if (sign > 0)
               expectedSide = RobotSide.RIGHT;
            else
               expectedSide = RobotSide.LEFT;

            String testDescription = pathDirection.toString() + " " + String.format("angle = %.2f", sign * yawStep);
            footsteps = testTurnStraightFootstepGenerator(testDescription + " AutoStance, turn staight", startX, startY, startYaw, null, pathDirection, endX,
                  endY, vis);
            assertStepSide(testDescription, footsteps.get(0), expectedSide);

            footsteps = testTurnStraightTurnFootstepGenerator(testDescription + " AutoStance, turn staight turn", startX, startY, startYaw, null,
                  pathDirection, endX, endY, endYaw, vis);
            assertStepSide(testDescription, footsteps.get(0), expectedSide);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.5)
	@Test(timeout=300000)
   public void turningNonStandardInitialConditionsAutoStanceSideSelectionTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double endX = 2.0;
      double endY = 0.0;
      double startYawStep = Math.PI / 3;
      ArrayList<Footstep> footsteps;

      double midDelta = (maxOpenHipAngle + maxCloseHipAngle) / 2;
      double[] feetDeltaYawArray = { maxOpenHipAngle, maxCloseHipAngle, midDelta + 0.1 - maxCloseHipAngle, midDelta - 0.1 - maxCloseHipAngle, 0,
            -maxCloseHipAngle };
      RobotSide[] expectedRightOpeningStartFootArray = { RobotSide.LEFT, RobotSide.LEFT, RobotSide.LEFT, RobotSide.RIGHT, RobotSide.RIGHT, RobotSide.RIGHT };

      for (PathOrientation pathDirection : PathOrientation.values())
      {
         double endYaw = pathDirection.getOrientationRadians();

         for (int sign = 1; sign > -2; sign -= 2)
         {
            double startYaw = pathDirection.getOrientationRadians() + sign * startYawStep;
            RobotSide openingSide = (sign > 0) ? RobotSide.RIGHT : RobotSide.LEFT;

            for (int deltaYawIndex = 0; deltaYawIndex < feetDeltaYawArray.length; deltaYawIndex++)
            {
               initialDeltaFeetYaw = feetDeltaYawArray[deltaYawIndex];

               RobotSide expectedSide = expectedRightOpeningStartFootArray[deltaYawIndex];
               if (openingSide == RobotSide.LEFT)
                  expectedSide = expectedSide.getOppositeSide();

               String testDescription = pathDirection.toString() + " "
                     + String.format("angle = %.2f, feet delta angle = %.2f", sign * startYawStep, initialDeltaFeetYaw);
               footsteps = testTurnStraightFootstepGenerator(testDescription + " AutoStance, turn staight", startX, startY, startYaw, null, pathDirection,
                     endX, endY, vis);
               assertStepSide(testDescription, footsteps.get(0), expectedSide);

               footsteps = testTurnStraightTurnFootstepGenerator(testDescription + " AutoStance, turn staight turn", startX, startY, startYaw, null,
                     pathDirection, endX, endY, endYaw, vis);
               assertStepSide(testDescription, footsteps.get(0), expectedSide);
            }
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void leftRightNonStandardInitialConditionsAutoStanceSideSelectionTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startY = 0.0;
      double endX = stepLength * 4; // Make this evenly divisible by stepLength to get max length steps (stepLength)
      double endY = 0.0;

      ArrayList<Footstep> footsteps;

      double[] feetDeltaYArray = { stepWidth, stepLength * 3 / 8 + stepWidth, stepLength * 5 / 8 + stepWidth, stepLength + stepWidth };
      RobotSide[] expectedRightOpeningStartFootArray = { RobotSide.RIGHT, RobotSide.RIGHT, RobotSide.LEFT, RobotSide.LEFT };
      double[] startXArray = { -stepLength, -stepLength * 13 / 16, -stepLength * 5 / 16, -stepLength / 2 }; // Make it so first footstep/stance centers around 0.0

      for (PathOrientation pathDirection : PathOrientation.leftRightValues())
      {
         double startYaw = pathDirection.getOrientationRadians();
         double endYaw = pathDirection.getOrientationRadians();

         for (int deltaYIndex = 0; deltaYIndex < feetDeltaYArray.length; deltaYIndex++)
         {
            double startX = startXArray[deltaYIndex];
            initialDeltaFeetLocalY = feetDeltaYArray[deltaYIndex];

            RobotSide expectedSide = expectedRightOpeningStartFootArray[deltaYIndex];
            if (pathDirection == PathOrientation.LEFT)
               expectedSide = expectedSide.getOppositeSide();

            String testDescription = pathDirection.toString() + String.format(" feet inital delta y = %.2f", initialDeltaFeetLocalY);
            footsteps = testTurnStraightFootstepGenerator(testDescription + " AutoStance, turn staight", startX, startY, startYaw, null, pathDirection, endX,
                  endY, vis);
            assertStepSide(testDescription, footsteps.get(0), expectedSide);

            footsteps = testTurnStraightTurnFootstepGenerator(testDescription + " AutoStance, turn staight turn", startX, startY, startYaw, null,
                  pathDirection, endX, endY, endYaw, vis);
            assertStepSide(testDescription, footsteps.get(0), expectedSide);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.6)
	@Test(timeout=300000)
   public void leftRightWrongOutwardFirstStepTest()
   {
      // DRC parameters:
      stepLength = 0.25;
      stepWidth = 0.21;
      maxOpenHipAngle = Math.toRadians(25);
      maxCloseHipAngle = Math.toRadians(8);
      double turnStepWidth = 0.4;
      pathType = new SimplePathParameters(stepLength, stepWidth, 0.0, maxOpenHipAngle, maxCloseHipAngle, turnStepWidth);

      //    Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startY = 0.0;

      ArrayList<Footstep> footsteps;

      initialDeltaFeetLocalY = 0.4157;
      initialDeltaFeetYaw = -0.0434641623; // This gave the low turn overshoot fraction. + did not.
      double endX = (3.602 - 1.3563);
      assertTrue("precondition test", initialDeltaFeetLocalY - stepWidth < stepLength);

      // maxOpenHipAngle
      double maxY = endX * Math.tan(0.009401578);
      for (double endY = -maxY; endY <= maxY; endY += maxY / 5)
      {
         for (PathOrientation pathDirection : PathOrientation.leftRightValues())
         {
            double startYaw = Math.PI / 2; // pathDirection.getOrientationRadians();
            double endYaw = Math.PI / 2; // pathDirection.getOrientationRadians();

            RobotSide expectedSide = (initialDeltaFeetLocalY < stepWidth + stepLength / 2) ? RobotSide.RIGHT : RobotSide.LEFT;
            if (pathDirection == PathOrientation.LEFT)
            {
               expectedSide = expectedSide.getOppositeSide();

            }

            double startX = 0;

            //          if (pathDirection == PathOrientation.LEFT)
            //          {
            //             vis = Visualization.VISUALIZE;
            //          }

            double endXDirection = (pathDirection == PathOrientation.LEFT) ? -endX : endX;
            String testDescription = pathDirection.toString()
                  + String.format(" endY = %.2f, Dyinit = %.2f, %%init = %.1f%%", endY, initialDeltaFeetLocalY, (initialDeltaFeetLocalY - stepWidth)
                        / stepLength);
            String testMessage = testDescription + " AutoStance, turn staight";
            footsteps = testTurnStraightFootstepGenerator(testMessage, startX, startY, startYaw, null, pathDirection, endXDirection, endY, vis);
            assertStepSide(testDescription, footsteps.get(0), expectedSide);
            String message = "don't step too far, step: " + footsteps.get(0).getX() + ", allowed region: +/- " + stepWidth / 2;
            assertTrue(message, (footsteps.get(0).getX() <= stepWidth / 2) && (footsteps.get(0).getX() >= -stepWidth / 2));

            vis = Visualization.NO_VISUALIZATION;
            testMessage = testDescription + " AutoStance, turn staight turn";
            footsteps = testTurnStraightTurnFootstepGenerator(testMessage, startX, startY, startYaw, null, pathDirection, endXDirection, endY, endYaw, vis);
            assertStepSide(testDescription, footsteps.get(0), expectedSide);
            message = "don't step too far, step: " + footsteps.get(0).getX() + ", allowed region: +/- " + stepWidth / 2;
            assertTrue(message, (footsteps.get(0).getX() <= stepWidth / 2) && (footsteps.get(0).getX() >= -stepWidth / 2));
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout=300000)
   public void forwardBackwardNonStandardInitialConditionsAutoStanceSideSelectionTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double endY = 0.0;

      double distTest = Math.sqrt(stepLength * stepLength + stepWidth * stepWidth);

      ArrayList<Footstep> footsteps;
      SideDependentList<Footstep> startFeet;

      double[] feetDeltaXArray = { stepLength, stepLength / 2, -stepLength / 2, -stepLength };

      for (PathOrientation pathDirection : PathOrientation.forwardReverseValues())
      {
         double startYaw = pathDirection.getOrientationRadians();
         double endYaw = pathDirection.getOrientationRadians();

         for (int deltaXIndex = 0; deltaXIndex < feetDeltaXArray.length; deltaXIndex++)
         {
            initialDeltaFeetLocalX = feetDeltaXArray[deltaXIndex];
            startFeet = updatedStartFeet(startX, startY, startYaw);
            double endX = 4.0 * stepLength + initialDeltaFeetLocalX / 2; // Try to have max length divisible steps

            // vis = Visualization.VISUALIZE;

            RobotSide expectedFirstSwingSide = (initialDeltaFeetLocalX > 0) ? RobotSide.RIGHT : RobotSide.LEFT;
            if (pathDirection == PathOrientation.REVERSE)
               expectedFirstSwingSide = expectedFirstSwingSide.getOppositeSide();

            String testDescription = pathDirection.toString() + String.format(" feet inital delta x = %.2f", initialDeltaFeetLocalX);
            String testDescription1 = testDescription + " AutoStance (test far foot selection), turn staight";
            footsteps = testTurnStraightFootstepGenerator(testDescription1, startX, startY, startYaw, null, pathDirection, endX, endY, vis);
            assertStepSide(testDescription, footsteps.get(0), expectedFirstSwingSide);
            assertMaxDisplacementStanceToSwingIncludingInitialFeet(distTest, footsteps, startFeet, testDescription1);

            // vis = Visualization.NO_VISUALIZATION;

            testDescription1 = testDescription + " AutoStance (test far foot selection), turn staight turn";
            footsteps = testTurnStraightTurnFootstepGenerator(testDescription1, startX, startY, startYaw, null, pathDirection, endX, endY, endYaw, vis);
            assertStepSide(testDescription, footsteps.get(0), expectedFirstSwingSide);
            assertMaxDisplacementStanceToSwingIncludingInitialFeet(distTest, footsteps, startFeet, testDescription1);

            // vis = Visualization.VISUALIZE;

            String testDescription2 = testDescription + " NearStance (test for overstep control), turn staight";
            footsteps = testTurnStraightFootstepGenerator(testDescription2, startX, startY, startYaw, expectedFirstSwingSide, pathDirection, endX, endY, vis);
            assertMaxDisplacementStanceToSwingIncludingInitialFeet(distTest, footsteps, startFeet, testDescription2);

            // vis = Visualization.NO_VISUALIZATION;

            testDescription2 = testDescription + " NearStance (test for overstep control), turn staight turn";
            footsteps = testTurnStraightTurnFootstepGenerator(testDescription2, startX, startY, startYaw, expectedFirstSwingSide, pathDirection, endX, endY,
                  endYaw, vis);
            assertMaxDisplacementStanceToSwingIncludingInitialFeet(distTest, footsteps, startFeet, testDescription2);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void TranslationPathTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double endY = 3.0;
      double endX = 3.0;
      double startYaw = 0.0;

      String testDescription = "Translation";
      testTranslationFootstepGenerator(testDescription + " RStance", startX, startY, startYaw, RobotSide.RIGHT, endX, endY, vis);
      testTranslationFootstepGenerator(testDescription + " LStance", startX, startY, startYaw, RobotSide.LEFT, endX, endY, vis);
      testTranslationFootstepGenerator(testDescription + " LStance", startX, startY, startYaw, null, endX, endY, vis);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void TranslationPathEndPointTest()
   {
      // Visualization vis = Visualization.VISUALIZE;

      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double startYaw = 0.0;

      double r = 3;
      for (double theta = 0; theta < Math.PI * 2; theta += Math.PI / 6)
      {
         String testDescription = String.format("Translation path_theta = %.2f", theta);
         translationAtAngleTest(testDescription, vis, startX, startY, startYaw, r, theta, Double.POSITIVE_INFINITY);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void TranslationPathYawTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;

      double d = 3;
      double r = Math.sqrt(2 * d * d);
      double paththeta = Math.PI / 4;
      for (double theta = 0; theta < Math.PI * 2; theta += Math.PI / 6)
      {
         String testDescription = String.format("Translation path orientation = %.2f", theta);
         translationAtAngleTest(String.format(testDescription, Math.toDegrees(theta), r), vis, startX, startY, theta, r * 4, paththeta,
               Double.POSITIVE_INFINITY);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void TranslationPathMaxDispTest()
   {
      //    Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double startYaw = 0.0;

      String testDescription = "Translation dist test theta = %.2f, r = %.2f";

      initialDeltaFeetLocalY = translationalNominalStepWidth;

      double theta = 0;
      double r = translationalPathType.getForwardStepLength();
      translationAtAngleTest(String.format(testDescription, Math.toDegrees(theta), r), vis, startX, startY, startYaw, r * 4, theta, r * 2);

      theta = Math.PI;
      r = translationalPathType.getBackwardStepLength();
      translationAtAngleTest(String.format(testDescription, Math.toDegrees(theta), r), vis, startX, startY, startYaw, r * 4, theta, r * 2);

      initialDeltaFeetLocalY = translationalMinimumStepWidth;

      theta = Math.PI / 2;
      r = translationalPathType.getSidewardStepLength();
      translationAtAngleTest(String.format(testDescription, Math.toDegrees(theta), r), vis, startX, startY, startYaw, r * 4, theta, r);

      theta = -Math.PI / 2;
      r = translationalPathType.getSidewardStepLength();
      translationAtAngleTest(String.format(testDescription, Math.toDegrees(theta), r), vis, startX, startY, startYaw, r * 4, theta, r);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void TranslationPath_RandomTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      Random random = new Random(1984L);
      
      double startX = 2 * random.nextDouble() - 1;
      double startY = 2 * random.nextDouble() - 1;
      double startYaw = 2 * Math.PI * random.nextDouble();

      double r = 2.0 * random.nextDouble() - 1.0;
      double theta = 2 * Math.PI * random.nextDouble();

      String testDescription = String.format("Translation (%.2f,%.2f)->(r%.2f,theta%.2f) initialtheta = %.2f", startX, startY, r, theta, startYaw);
      translationAtAngleTest(String.format(testDescription, Math.toDegrees(theta), r), vis, startX, startY, startYaw, r, theta, Double.POSITIVE_INFINITY);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void TranslationPath_VaryLeftRightNonStandardInitialConditionsTest()
   {
      //    Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      double stepLength = translationalSidewardStepLength;
      double stepWidth = translationalMinimumStepWidth;

      double startY = 0.0;
      double endX = stepLength * 4; // Make this evenly divisible by stepLength to get max length steps (stepLength)
      double endY = 0.0;

      ArrayList<Footstep> footsteps;

      double[] feetDeltaYArray = { stepWidth, stepLength * 3 / 8 + stepWidth, stepLength * 5 / 8 + stepWidth, stepLength + stepWidth };
      RobotSide[] expectedRightOpeningStartFootArray = { RobotSide.RIGHT, RobotSide.RIGHT, RobotSide.LEFT, RobotSide.LEFT };
      double[] startXArray = { -stepLength, -stepLength * 13 / 16, -stepLength * 5 / 16, -stepLength / 2 }; // Make it so first footstep/stance centers around 0.0

      for (PathOrientation pathDirection : PathOrientation.leftRightValues())
      {
         double startYaw = pathDirection.getOrientationRadians();

         for (int deltaYIndex = 0; deltaYIndex < feetDeltaYArray.length; deltaYIndex++)
         {
            double startX = startXArray[deltaYIndex];
            initialDeltaFeetLocalY = feetDeltaYArray[deltaYIndex];
            SideDependentList<Footstep> startFeet = updatedStartFeet(startX, startY, startYaw);

            RobotSide expectedSide = expectedRightOpeningStartFootArray[deltaYIndex];
            if (pathDirection == PathOrientation.LEFT)
               expectedSide = expectedSide.getOppositeSide();

            String testDescription = pathDirection.toString() + String.format(" feet inital delta y = %.2f", initialDeltaFeetLocalY);
            footsteps = testTranslationFootstepGenerator(testDescription + " AutoStance, translation path", startX, startY, startYaw, null, endX, endY, vis);
            assertStepSide(testDescription, footsteps.get(0), expectedSide);
            assertMaxDisplacementStanceToSwingIncludingInitialFeet(stepLength + stepWidth, footsteps, startFeet, testDescription);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout=300000)
   public void TranslationPath_VaryforwardNonStandardInitialConditionsTest()
   {
      //    Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX;
      double startY = 0.0;
      double endY = 0.0;

      double stepLength = translationalForwardStepLength;
      double stepWidth = translationalNominalStepWidth;
      double distTest = Math.sqrt(stepLength * stepLength + stepWidth * stepWidth);

      ArrayList<Footstep> footsteps;
      SideDependentList<Footstep> startFeet;

      double[] feetDeltaXArray = { stepLength, stepLength / 2, -stepLength / 2, -stepLength };

      PathOrientation pathDirection = PathOrientation.FORWARD;

      double startYaw = pathDirection.getOrientationRadians();

      for (int deltaXIndex = 0; deltaXIndex < feetDeltaXArray.length; deltaXIndex++)
      {
         startX = -Math.abs(feetDeltaXArray[deltaXIndex]) / 2;
         initialDeltaFeetLocalX = feetDeltaXArray[deltaXIndex];
         startFeet = updatedStartFeet(startX, startY, startYaw);
         double endX = 4.0 * stepLength; // Try to have max length divisible steps

         RobotSide expectedFirstSwingSide = (initialDeltaFeetLocalX > 0) ? RobotSide.RIGHT : RobotSide.LEFT;
         if (pathDirection == PathOrientation.REVERSE)
            expectedFirstSwingSide = expectedFirstSwingSide.getOppositeSide();

         String testDescription = pathDirection.toString() + String.format(" feet inital delta x = %.2f", initialDeltaFeetLocalX);
         String testDescription1 = testDescription + " AutoStance (test far foot selection), translation path";
         footsteps = testTranslationFootstepGenerator(testDescription1, startX, startY, startYaw, null, endX, endY, vis);
         assertMaxDisplacementStanceToSwingIncludingInitialFeet(distTest, footsteps, startFeet, testDescription1);
         assertStepSide(testDescription, footsteps.get(0), expectedFirstSwingSide);

         String testDescription2 = testDescription + " NearStance (test for overstep control), translation path";
         footsteps = testTranslationFootstepGenerator(testDescription2, startX, startY, startYaw, expectedFirstSwingSide, endX, endY, vis);
         assertMaxDisplacementStanceToSwingIncludingInitialFeet(distTest, footsteps, startFeet, testDescription2);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void TranslationPath_VaryBackwardNonStandardInitialConditionsTest()
   {
      // Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double endY = 0.0;

      double stepLength = translationalBackwardStepLength;
      double stepWidth = translationalNominalStepWidth;
      double distTest = Math.sqrt(stepLength * stepLength + stepWidth * stepWidth);

      ArrayList<Footstep> footsteps;
      SideDependentList<Footstep> startFeet;

      double[] feetDeltaXArray = { stepLength, stepLength / 2, -stepLength / 2, -stepLength };

      PathOrientation pathDirection = PathOrientation.REVERSE;

      double startYaw = pathDirection.getOrientationRadians();

      for (int deltaXIndex = 0; deltaXIndex < feetDeltaXArray.length; deltaXIndex++)
      {
         initialDeltaFeetLocalX = feetDeltaXArray[deltaXIndex];
         startFeet = updatedStartFeet(startX, startY, startYaw);
         double endX = 4.0 * stepLength + initialDeltaFeetLocalX / 2; // Try to have max length divisible steps

         RobotSide expectedFirstSwingSide = (initialDeltaFeetLocalX > 0) ? RobotSide.RIGHT : RobotSide.LEFT;
         if (pathDirection == PathOrientation.REVERSE)
            expectedFirstSwingSide = expectedFirstSwingSide.getOppositeSide();

         String testDescription = pathDirection.toString() + String.format(" feet inital delta x = %.2f", initialDeltaFeetLocalX);
         String testDescription1 = testDescription + " AutoStance (test far foot selection), translation path";
         footsteps = testTranslationFootstepGenerator(testDescription1, startX, startY, startYaw, null, endX, endY, vis);
         assertMaxDisplacementStanceToSwingIncludingInitialFeet(distTest, footsteps, startFeet, testDescription1);
         assertStepSide(testDescription, footsteps.get(0), expectedFirstSwingSide);

         String testDescription2 = testDescription + " NearStance (test for overstep control), translation path";
         footsteps = testTranslationFootstepGenerator(testDescription2, startX, startY, startYaw, expectedFirstSwingSide, endX, endY, vis);
         assertMaxDisplacementStanceToSwingIncludingInitialFeet(distTest, footsteps, startFeet, testDescription2);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void RotateTranslateRotate_StepInPlaceTest()
   {
      //    Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      /////////////////////////////////////////////
      // Setup desired start and end configurations/poses
      double startX = 0.0;
      double startY = 0.0;
      double startYaw = 0.0;
      double pathYaw = 0.0;
      double endX = 0.0;
      double endY = 0.0;
      double endYaw = 0.0;

      ArrayList<Footstep> footsteps;

      //    SideDependentList<Footstep> startFeet;
      //
      //    startFeet = updatedStartFeet(startX, startY, startYaw);

      String testDescription = String.format("RTR step in place, (x,y,t)-t-(x,y,t) = (%.2f,%.2f,%.2f)-%.2f-(%.2f,%.2f,%.2f)", startX, startY, startYaw,
            pathYaw, endX, endY, endYaw);
      footsteps = testRotateTranslateRotateFootstepGenerator(testDescription, startX, startY, startYaw, pathYaw, endX, endY, endYaw, vis);
      assertEquals("Should have exactly 2 footsteps for step in place. " + testDescription, 2, footsteps.size());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.5)
	@Test(timeout=300000)
   public void RotateTranslateRotate_RandomTest()
   {
	   Random random = new Random(1789L);
	   
      //    Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      for (int i = 0; i < 100; i++)
      {
         double startX = RandomTools.generateRandomDouble(random, 1.0);
         double startY = RandomTools.generateRandomDouble(random, 1.0);
         double startYaw = RandomTools.generateRandomDouble(random, Math.PI);
         double pathYaw = RandomTools.generateRandomDouble(random, Math.PI);
         double endX = RandomTools.generateRandomDouble(random, 1.0);
         double endY = RandomTools.generateRandomDouble(random, 1.0);
         double endYaw = RandomTools.generateRandomDouble(random, Math.PI);

         String testDescription = String.format("RTR random, (x,y,t):t:(x,y,t) = (%.2f,%.2f,%.2f):%.2f:(%.2f,%.2f,%.2f)", startX, startY, startYaw, pathYaw,
               endX, endY, endYaw);
         testRotateTranslateRotateFootstepGenerator(testDescription, startX, startY, startYaw, pathYaw, endX, endY, endYaw, vis);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.2)
	@Test(timeout=300000)
   public void RotateTranslateRotate_RotateTranslateRotateTest()
   {
      //    Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      double r = 3.0;
      double[] startXs = { -1.0, 0.0 };
      double[] startYs = { -1.0, 0.0 };
      for (int i = 0; i < startXs.length; i++)
      {
         double startX = startXs[i];
         double startY = startYs[i];

         for (double theta = 0; theta < Math.PI * 2; theta += Math.PI / 2)
         {
            double endX = r * Math.cos(theta);
            double endY = r * Math.sin(theta);
            for (double startYaw = 0; startYaw < Math.PI * 2; startYaw += Math.PI / 2)
            {
               for (double pathYaw = 0; pathYaw < Math.PI * 2; pathYaw += Math.PI / 2)
               {
                  for (double endYaw = 0; endYaw < Math.PI * 2; endYaw += Math.PI / 2)
                  {

                     String testDescription = String.format("RTR, (x,y,t):t:([r,t],t) = (%.2f,%.2f,%.2f):%.2f:([%.2f,%.2f],%.2f)", startX, startY, startYaw,
                           pathYaw, r, theta, endYaw);
                     testRotateTranslateRotateFootstepGenerator(testDescription, startX, startY, startYaw, pathYaw, endX, endY, endYaw, vis);
                  }
               }
            }
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.3)
	@Test(timeout=300000)
   public void RotateTranslateRotate_RotateRotateTest()
   {
      // If no translation, treat as one rotate beginning to end.
      //    Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      double startX = -0.5;
      double startY = -1.0;
      double endX = startX;
      double endY = startY;

      for (double startYaw = 0; startYaw < Math.PI * 2; startYaw += Math.PI / 4)
      {
         for (double pathYaw = 0; pathYaw < Math.PI * 2; pathYaw += Math.PI / 4)
         {
            for (double endYaw = 0; endYaw < Math.PI * 2; endYaw += Math.PI / 4)
            {
               ArrayList<Footstep> footsteps;

               //             if(startYaw == 0.0 && endYaw == Math.PI / 4)
               //                vis = Visualization.VISUALIZE;
               //             else
               //                vis = Visualization.NO_VISUALIZATION;
               //             
               String testDescription = String.format("RTR noTranslation, t:t:t = %.2f:%.2f:%.2f", startYaw, pathYaw, endYaw);
               footsteps = testRotateTranslateRotateFootstepGenerator(testDescription, startX, startY, startYaw, pathYaw, endX, endY, endYaw, vis);

               for (int i = 0; i < footsteps.size(); i++)
               {
                  String testDescription2 = "Step " + i + " " + testDescription;
                  assertFootstepInAngleRange(testDescription2, footsteps.get(i), startYaw, endYaw);
               }
            }
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void RotateTranslateRotate_CourseRestepTest()
   {
      //    Visualization vis = Visualization.VISUALIZE;

      Visualization vis = Visualization.NO_VISUALIZATION;

      ArrayList<Footstep> footsteps;

      AbstractFootstepGenerator.setNoTranslationTolerance(.01); // This is part of the fix: a preference. The rest is propagating it properly.

      double startX = 0;
      double startY = 0;
      double endX = 1;
      double endY = 1;
      double startYaw = 0;

      // test that small angle displacement doesn't create initial square up steps
      double pathYaw = 2.2159003493005258E-5;
      double endYaw = pathYaw;

      double eps = Math.abs(translationalNominalStepWidth - translationalMinimumStepWidth) / 2;

      String testDescription = String.format("RTR, (x,y,t):t:([r,t],t) = (%.2f,%.2f,%.2f):%.2f:(%.2f,%.2f,%.2f)", startX, startY, startYaw, pathYaw, endX,
            endY, endYaw);
      footsteps = testRotateTranslateRotateFootstepGenerator(testDescription, startX, startY, startYaw, pathYaw, endX, endY, endYaw, vis);
      assertNoCourseRepeatedSquareUpSteps(testDescription, footsteps, startX, startY, startYaw, eps, 1);

      // test that angular displacement just larger than "no-displacement", but smaller than a step doesn't create square up steps
      endX = 1.5;
      pathYaw = -.02;
      endYaw = pathYaw;

      testDescription = String.format("RTR, (x,y,t):t:([r,t],t) = (%.2f,%.2f,%.2f):%.2f:(%.2f,%.2f,%.2f)", startX, startY, startYaw, pathYaw, endX, endY,
            endYaw);
      footsteps = testRotateTranslateRotateFootstepGenerator(testDescription, startX, startY, startYaw, pathYaw, endX, endY, endYaw, vis);
      assertNoCourseRepeatedSquareUpSteps(testDescription, footsteps, startX, startY, startYaw, eps, 1);

      // test that angular displacement just under first turn step doesn't create square up steps, but also turns to correct angle
      pathYaw = -maxOpenHipAngle / 2;
      endYaw = pathYaw;

      testDescription = String.format("RTR, (x,y,t):t:([r,t],t) = (%.2f,%.2f,%.2f):%.2f:(%.2f,%.2f,%.2f)", startX, startY, startYaw, pathYaw, endX, endY,
            endYaw);
      footsteps = testRotateTranslateRotateFootstepGenerator(testDescription, startX, startY, startYaw, pathYaw, endX, endY, endYaw, vis);
      assertNoCourseRepeatedSquareUpSteps(testDescription, footsteps, startX, startY, startYaw, eps, 1);
      String plusTest = testDescription + " check correct footstep orientation if no overstep";
      FrameOrientation2d expectedStepOrientation = new FrameOrientation2d(WORLD_FRAME, pathYaw);
      assertStepIsPointingCorrectly(plusTest, footsteps.get(0), expectedStepOrientation);

      // test that angular displacement in direction of translational displacement backwards doesn't create square up steps
      endX = -1.5;
      pathYaw = -.02;
      endYaw = pathYaw;

      testDescription = String.format("RTR, (x,y,t):t:([r,t],t) = (%.2f,%.2f,%.2f):%.2f:(%.2f,%.2f,%.2f)", startX, startY, startYaw, pathYaw, endX, endY,
            endYaw);
      footsteps = testRotateTranslateRotateFootstepGenerator(testDescription, startX, startY, startYaw, pathYaw, endX, endY, endYaw, vis);
      assertNoCourseRepeatedSquareUpSteps(testDescription, footsteps, startX, startY, startYaw, eps, 1);

      // test that small angular displacement in direction of translational displacement doesn't step in place
      endX = 1.5;
      pathYaw = .02;
      endYaw = pathYaw;

      testDescription = String.format("RTR, (x,y,t):t:([r,t],t) = (%.2f,%.2f,%.2f):%.2f:(%.2f,%.2f,%.2f)", startX, startY, startYaw, pathYaw, endX, endY,
            endYaw);
      footsteps = testRotateTranslateRotateFootstepGenerator(testDescription, startX, startY, startYaw, pathYaw, endX, endY, endYaw, vis);
      assertNoCourseRepeatedSquareUpSteps(testDescription, footsteps, startX, startY, startYaw, eps, 1);
      plusTest = testDescription + " check correct footstep orientation if no overstep";
      expectedStepOrientation = new FrameOrientation2d(WORLD_FRAME, pathYaw);
      assertStepIsPointingCorrectly(plusTest, footsteps.get(0), expectedStepOrientation);

      // test that angular displacement in opposite direction of translational displacement going backward doesn't cross toe across other foot or create step in place step
      endX = -1.5;
      pathYaw = .02;
      endYaw = pathYaw;

      testDescription = String.format("RTR, (x,y,t):t:([r,t],t) = (%.2f,%.2f,%.2f):%.2f:(%.2f,%.2f,%.2f)", startX, startY, startYaw, pathYaw, endX, endY,
            endYaw);
      footsteps = testRotateTranslateRotateFootstepGenerator(testDescription, startX, startY, startYaw, pathYaw, endX, endY, endYaw, vis);
      assertNoCourseRepeatedSquareUpSteps(testDescription, footsteps, startX, startY, startYaw, eps, 1);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void RotateTranslateRotate_CourseRestepCrossoverTest()
   {
      //    Visualization vis = Visualization.VISUALIZE;

      Visualization vis = Visualization.NO_VISUALIZATION;

      ArrayList<Footstep> footsteps;

      AbstractFootstepGenerator.setNoTranslationTolerance(.01);

      double startX = 0;
      double startY = 0;
      double endX = 1;
      double endY = 1;
      double startYaw = 0;

      double eps = Math.abs(translationalNominalStepWidth - translationalMinimumStepWidth) / 2;

      // test that angular displacement in direction of translational displacement doesn't cross feet
      endX = 1.5;
      double pathYaw = maxOpenHipAngle / 2;
      double endYaw = pathYaw;

      String testDescription = String.format("RTR, (x,y,t):t:([r,t],t) = (%.2f,%.2f,%.2f):%.2f:(%.2f,%.2f,%.2f)", startX, startY, startYaw, pathYaw, endX,
            endY, endYaw);
      footsteps = testRotateTranslateRotateFootstepGenerator(testDescription, startX, startY, startYaw, pathYaw, endX, endY, endYaw, vis);
      assertNoCourseRepeatedSquareUpSteps(testDescription, footsteps, startX, startY, startYaw, eps, 1);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout=300000)
   public void RotateTranslateRotate_CourseRestepAtEndTest()
   {
      //    Visualization vis = Visualization.VISUALIZE;
      Visualization vis = Visualization.NO_VISUALIZATION;

      ArrayList<Footstep> footsteps;

      AbstractFootstepGenerator.setNoTranslationTolerance(.01); // This is part of the fix: a preference. The rest is propagating it properly.

      double startX = 0;
      double startY = 0;
      double endX = 1.5;
      double endY = 1;
      double startYaw = 0;

      // test that small angle displacement doesn't create initial square up steps
      double pathYaw = 0;
      double endYaw = .02;

      double eps = Math.abs(translationalNominalStepWidth - translationalMinimumStepWidth) / 2 + .001;

      String testDescription = String.format("RTR, (x,y,t):t:([r,t],t) = (%.2f,%.2f,%.2f):%.2f:(%.2f,%.2f,%.2f)", startX, startY, startYaw, pathYaw, endX,
            endY, endYaw);
      footsteps = testRotateTranslateRotateFootstepGenerator(testDescription, startX, startY, startYaw, pathYaw, endX, endY, endYaw, vis);
      assertNoCourseRepeatedSquareUpSteps(testDescription, footsteps, startX, startY, startYaw, eps, 1);
   }

   private void assertNoCourseRepeatedSquareUpSteps(String message, ArrayList<Footstep> footstepsOriginal, double startX, double startY, double startYaw,
         double eps, int shortSteps)
   {
      SideDependentList<Footstep> startFeet = updatedStartFeet(startX, startY, startYaw);
      ArrayList<Footstep> footsteps = prependStanceToFootstepQueue(footstepsOriginal, startFeet);
      int numFootsteps = footsteps.size();

      for (int i = 0; i + 2 < numFootsteps; i++)
      {
         Footstep firstFootstep = footsteps.get(i);
         Footstep nextFootstep = footsteps.get(i + 2);
         FramePose firstFootstepPose = new FramePose();
         firstFootstep.getSolePose(firstFootstepPose);
         FramePose nextFootstepPose = new FramePose();
         nextFootstep.getSolePose(nextFootstepPose);
         FramePoint2d firstFootstepPosition2d = new FramePoint2d();
         FramePoint2d nextFootstepPosition2d = new FramePoint2d();
         firstFootstepPose.getPosition2dIncludingFrame(firstFootstepPosition2d);
         nextFootstepPose.getPosition2dIncludingFrame(nextFootstepPosition2d);
         double testeps = eps;

         String message2 = message + "course restep test " + i + "to" + (i + 2);
         boolean condition = (nextFootstepPosition2d.distance(firstFootstepPosition2d) > testeps)
               || (Math.abs(firstFootstepPose.getYaw() - nextFootstepPose.getYaw()) > testeps);
         Visualization visualize = (condition == true) ? Visualization.NO_VISUALIZATION : Visualization.VISUALIZE;
         showFootstepsIfVisualize(message2, footstepsOriginal, startFeet, visualize);
         assertTrue(message2, condition);
      }
   }

   private void assertFootstepInAngleRange(String testDescription, Footstep footstep, double startYaw, double endYaw)
   {
      FramePose footPose = new FramePose();
      footstep.getSolePose(footPose);

      FrameOrientation2d footOrientation = new FrameOrientation2d();
      footPose.getOrientation2dIncludingFrame(footOrientation);
      footOrientation.changeFrame(WORLD_FRAME);
      double footYaw = footOrientation.getYaw();

      double angleBetweenStartAndEnd = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(endYaw, startYaw));
      double angleBetweenStartAndFoot = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(footYaw, startYaw));
      double angleBetweenFootAndEnd = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(endYaw, footYaw));

      //if start->foot + foot-> end > start->end, then the foot angle is outside the range between the two

      boolean outsideFan = (angleBetweenStartAndFoot + angleBetweenFootAndEnd) > angleBetweenStartAndEnd + 1e-14;
   }

   private void assertMaxDisplacementStanceToSwing(String message, ArrayList<Footstep> footsteps, double distTest)
   {
      if (distTest == Double.POSITIVE_INFINITY)
         return;

      Footstep footstep1;
      Footstep footstep2;
      for (int i = 0; i < footsteps.size() - 1; i++)
      {
         footstep1 = footsteps.get(i);
         footstep2 = footsteps.get(i + 1);
         FramePoint position2 = new FramePoint();
         footstep2.getPositionIncludingFrame(position2);
         FramePoint position1 = new FramePoint();
         footstep1.getPositionIncludingFrame(position1);
         double dis = position2.distance(position1);
         assertTrue(String.format(message + " %.2f<=%.2f? step %d to %d", dis, distTest, i, i + 1), dis < distTest + 1e-14);
      }
   }

   private void assertMaxDisplacementSwingToSwing(String message, ArrayList<Footstep> footsteps, double distTest)
   {
      if (distTest == Double.POSITIVE_INFINITY)
         return;

      Footstep footstep1;
      Footstep footstep2;
      for (int i = 0; i < footsteps.size() - 2; i++)
      {
         footstep1 = footsteps.get(i);
         footstep2 = footsteps.get(i + 2);
         FramePoint position2 = new FramePoint();
         footstep2.getPositionIncludingFrame(position2);
         FramePoint position1 = new FramePoint();
         footstep1.getPositionIncludingFrame(position1);
         double dis = position2.distance(position1);

         //       assertTrue(true);
         assertTrue(String.format(message + " %.2f<=%.2f? step %d to %d", dis, distTest, i, i + 2), dis < distTest + 1e-14);
      }
   }

   private void assertMaxDisplacementStanceToSwingIncludingInitialFeet(double distTest, ArrayList<Footstep> footsteps, SideDependentList<Footstep> startFeet,
         String testDescription)
   {
      footsteps = prependStanceToFootstepQueue(footsteps, startFeet);
      assertMaxDisplacementStanceToSwing(testDescription, footsteps, distTest);
   }

   private ArrayList<Footstep> testTranslationFootstepGenerator(String testDescription, double startX, double startY, double startYaw,
         RobotSide startStanceSide, double endX, double endY, Visualization visualize)
   {
      Point2d startWorldPoint = new Point2d(startX, startY);
      double startWorldYaw = startYaw;
      Point2d endWorldPoint = new Point2d(endX, endY);

      FramePoint2d endPoint = new FramePoint2d(WORLD_FRAME, endWorldPoint);

      /////////////////////////////
      // find first foot to step with (hopefully refactored away to be unnecessary later

      // boolean moveRearFootFirst = true;//because above angle is 0.0...
      // HumanoidReferenceFrames referenceFrames = null;//new HumanoidReferenceFrames();

      SideDependentList<Footstep> startFeet = updatedStartFeet(startX, startY, startYaw);

      ////////////////////////////////////////////
      // Create generator and generate footsteps
      TranslationFootstepGenerator generator;
      if (startStanceSide == null)
         generator = new TranslationFootstepGenerator(feet, soleFrames, endPoint, translationalPathType);
      else
         generator = new TranslationFootstepGenerator(feet, soleFrames, endPoint, translationalPathType, startStanceSide);

      ArrayList<Footstep> footsteps = (ArrayList<Footstep>) generator.generateDesiredFootstepList();

      /////////////////////////////////////
      // Validate footsteps

      FrameOrientation2d expectedEndOrientation = new FrameOrientation2d(WORLD_FRAME, startWorldYaw);
      assertFootstepsValid(testDescription, visualize, startWorldPoint, startYaw, endWorldPoint, startFeet, footsteps);
      assertLastTwoStepsPointingCorrectly(testDescription, footsteps, expectedEndOrientation);

      for (Footstep footstep : footsteps)
      {
         assertStepIsPointingCorrectly(testDescription, footstep, expectedEndOrientation);
      }

      return footsteps;
   }

   private void translationAtAngleTest(String testDescription, Visualization vis, double startX, double startY, double startYaw, double r, double theta,
         double distTest)
   {
      double endX = r * Math.cos(theta);
      double endY = r * Math.sin(theta);

      SideDependentList<Footstep> startFeet = updatedStartFeet(startX, startY, startYaw);

      String testDescription1 = testDescription + " RStance";
      ArrayList<Footstep> footsteps = testTranslationFootstepGenerator(testDescription1, startX, startY, startYaw, RobotSide.RIGHT, endX, endY, vis);
      footsteps = prependStanceToFootstepQueue(footsteps, startFeet);
      assertMaxDisplacementSwingToSwing(testDescription1, footsteps, distTest);

      String testDescription2 = testDescription + " LStance";
      footsteps = testTranslationFootstepGenerator(testDescription2, startX, startY, startYaw, RobotSide.LEFT, endX, endY, vis);
      footsteps = prependStanceToFootstepQueue(footsteps, startFeet);
      assertMaxDisplacementSwingToSwing(testDescription2, footsteps, distTest);

      String testDescription3 = testDescription + " AutoStance";
      footsteps = testTranslationFootstepGenerator(testDescription3, startX, startY, startYaw, null, endX, endY, vis);
      footsteps = prependStanceToFootstepQueue(footsteps, startFeet);
      assertMaxDisplacementSwingToSwing(testDescription3, footsteps, distTest);
   }

   private ArrayList<Footstep> testRotateTranslateRotateFootstepGenerator(String testDescription, double startX, double startY, double startYaw,
         double pathYaw, double endX, double endY, double endYaw, Visualization visualize)
   {
      Point2d startWorldPoint = new Point2d(startX, startY);

      FrameOrientation2d pathOrientation = new FrameOrientation2d(WORLD_FRAME, pathYaw);

      Point2d endWorldPoint = new Point2d(endX, endY);
      FramePoint2d endPoint = new FramePoint2d(WORLD_FRAME, endWorldPoint);
      FrameOrientation2d endOrientation = new FrameOrientation2d(WORLD_FRAME, endYaw);
      FramePose2d endPose = new FramePose2d(endPoint, endOrientation);

      SideDependentList<Footstep> startFeet = updatedStartFeet(startX, startY, startYaw);

      ////////////////////////////////////////////
      // Create generator and generate footsteps
      TurnTranslateTurnFootstepGenerator generator = new TurnTranslateTurnFootstepGenerator(feet, soleFrames, pathOrientation, endPose, pathType,
            translationalPathType);

      ArrayList<Footstep> footsteps = (ArrayList<Footstep>) generator.generateDesiredFootstepList();

      /////////////////////////////////////
      // Validate footsteps

      FrameOrientation2d expectedEndOrientation = new FrameOrientation2d(WORLD_FRAME, endYaw);
      assertFootstepsValid(testDescription, visualize, startWorldPoint, startYaw, endWorldPoint, startFeet, footsteps);
      assertLastTwoStepsPointingCorrectly(testDescription, footsteps, expectedEndOrientation);

      return footsteps;
   }

   private ArrayList<Footstep> testTurnStraightFootstepGenerator(String testDescription, double startX, double startY, double startYaw,
         RobotSide startStanceSide, double endX, double endY, Visualization visualize)
   {
      PathOrientation pathDirection = PathOrientation.FORWARD;

      return testTurnStraightFootstepGenerator(testDescription, startX, startY, startYaw, startStanceSide, pathDirection, endX, endY, visualize);
   }

   private ArrayList<Footstep> testTurnStraightFootstepGenerator(String testDescription, double startX, double startY, double startYaw,
         RobotSide startStanceSide, PathOrientation pathDirection, double endX, double endY, Visualization visualize)
   {
      Point2d startWorldPoint = new Point2d(startX, startY);
      double startWorldYaw = startYaw;
      Point2d endWorldPoint = new Point2d(endX, endY);

      FramePoint2d endPoint = new FramePoint2d(WORLD_FRAME, endWorldPoint);

      double pathYaw = pathDirection.getOrientationRadians();
      pathType.setAngle(pathYaw);

      SideDependentList<Footstep> startFeet = updatedStartFeet(startX, startY, startYaw);

      ////////////////////////////////////////////
      // Create generator and generate footsteps

      TurningThenStraightFootstepGenerator generator;
      if (startStanceSide == null)
         generator = new TurningThenStraightFootstepGenerator(feet, soleFrames, endPoint, pathType);
      else
         generator = new TurningThenStraightFootstepGenerator(feet, soleFrames, endPoint, pathType, startStanceSide);

      ArrayList<Footstep> footsteps = (ArrayList<Footstep>) generator.generateDesiredFootstepList();

      /////////////////////////////////////
      // Validate footsteps

      assertFootstepsValid(testDescription, visualize, startWorldPoint, startWorldYaw, endWorldPoint, startFeet, footsteps);
      assertLastTwoStepsArePathAlignedWithOffset(testDescription, footsteps, startWorldPoint, endWorldPoint, startWorldYaw, pathYaw);

      return footsteps;
   }

   private ArrayList<Footstep> testTurnStraightTurnFootstepGenerator(String testDescription, double startX, double startY, double startYaw,
         RobotSide startStanceSide, double endX, double endY, double endYaw, Visualization visualize)
   {
      PathOrientation pathDirection = PathOrientation.FORWARD;

      return testTurnStraightTurnFootstepGenerator(testDescription, startX, startY, startYaw, startStanceSide, pathDirection, endX, endY, endYaw, visualize);
   }

   private ArrayList<Footstep> testTurnStraightTurnFootstepGenerator(String testDescription, double startX, double startY, double startYaw,
         RobotSide startStanceSide, PathOrientation pathDirection, double endX, double endY, double endYaw, Visualization visualize)
   {
      Point2d startWorldPoint = new Point2d(startX, startY);
      Point2d endWorldPoint = new Point2d(endX, endY);
      FramePoint2d endPoint = new FramePoint2d(WORLD_FRAME, endWorldPoint);
      FrameOrientation2d endOrientation = new FrameOrientation2d(WORLD_FRAME, endYaw);

      FramePose2d endPose = new FramePose2d(endPoint, endOrientation);

      double pathYaw = pathDirection.getOrientationRadians();
      pathType.setAngle(pathYaw);

      SideDependentList<Footstep> startFeet = updatedStartFeet(startX, startY, startYaw);

      ////////////////////////////////////////////
      // Create generator and generate footsteps

      TurnStraightTurnFootstepGenerator generator;
      if (startStanceSide == null)
         generator = new TurnStraightTurnFootstepGenerator(feet, soleFrames, endPose, pathType);
      else
         generator = new TurnStraightTurnFootstepGenerator(feet, soleFrames, endPose, pathType, startStanceSide);

      ArrayList<Footstep> footsteps = (ArrayList<Footstep>) generator.generateDesiredFootstepList();

      /////////////////////////////////////
      // Validate footsteps

      assertFootstepsValid(testDescription, visualize, startWorldPoint, startYaw, endWorldPoint, startFeet, footsteps);
      assertLastTwoStepsPointingCorrectly(testDescription, footsteps, endOrientation);

      return footsteps;
   }

   private ArrayList<Footstep> testTwoSegmentFootstepGenerator(String testDescription, double startX, double startY, double startYaw,
         RobotSide startStanceSide, PathOrientation pathDirection1, double midX, double midY, RobotSide midStanceStartSide, PathOrientation pathDirection2,
         double endX, double endY, Visualization visualize)
   {
      Point2d startWorldPoint = new Point2d(startX, startY);
      Point2d midWorldPoint = new Point2d(midX, midY);
      Point2d endWorldPoint = new Point2d(endX, endY);

      FramePoint2d startPoint = new FramePoint2d(WORLD_FRAME, startWorldPoint);
      FramePoint2d midPoint = new FramePoint2d(WORLD_FRAME, midWorldPoint);
      FramePoint2d endPoint = new FramePoint2d(WORLD_FRAME, endWorldPoint);

      FrameOrientation2d startOrientation = new FrameOrientation2d(WORLD_FRAME, startYaw);
      FramePose2d startPose = new FramePose2d(startPoint, startOrientation);
      double midYaw = AngleTools.calculateHeading(startPose, midPoint, 0.0, 1e-14);

      FrameOrientation2d midOrientation = new FrameOrientation2d(WORLD_FRAME, midYaw);
      FramePose2d midPose = new FramePose2d(midPoint, midOrientation);

      double pathYaw1 = pathDirection1.getOrientationRadians();
      pathType.setAngle(pathYaw1);

      double pathYaw2 = pathDirection2.getOrientationRadians();
      pathType2.setAngle(pathYaw2);

      SideDependentList<Footstep> startFeet = updatedStartFeet(startX, startY, startYaw);

      ////////////////////////////////////////////
      // Create generator and generate footsteps

      TwoSegmentFootstepGenerator generator;
      if (startStanceSide == null)
         generator = new TwoSegmentFootstepGenerator(feet, soleFrames, midPose, endPoint, pathType, pathType2);
      else if (midStanceStartSide == null)
         generator = new TwoSegmentFootstepGenerator(feet, soleFrames, midPose, endPoint, pathType, pathType2, startStanceSide);
      else
         generator = new TwoSegmentFootstepGenerator(feet, soleFrames, midPose, endPoint, pathType, pathType2, startStanceSide, midStanceStartSide);

      ArrayList<Footstep> footsteps = (ArrayList<Footstep>) generator.generateDesiredFootstepList();

      /////////////////////////////////////
      // Validate footsteps

      assertFootstepsValid(testDescription, visualize, startWorldPoint, startYaw, endWorldPoint, startFeet, footsteps);

      return footsteps;
   }

   private SideDependentList<Footstep> updatedStartFeet(double startX, double startY, double startYaw)
   {
      double leftXFactor = initialDeltaFeetLocalX / 2 * Math.cos(startYaw) - initialDeltaFeetLocalY / 2 * Math.sin(startYaw);
      double leftYFactor = initialDeltaFeetLocalX / 2 * Math.sin(startYaw) + initialDeltaFeetLocalY / 2 * Math.cos(startYaw);
      FootstepSnapper footstepSnapper = new SimpleFootstepSnapper();

      Point2d leftFootStartPoint = new Point2d(startX + leftXFactor, startY + leftYFactor);
      FramePose2d leftFootPose2d = new FramePose2d(WORLD_FRAME, leftFootStartPoint, startYaw + initialDeltaFeetYaw / 2);
      Point2d rightFootStartPoint = new Point2d(startX - leftXFactor, startY - leftYFactor);
      FramePose2d rightFootPose2d = new FramePose2d(WORLD_FRAME, rightFootStartPoint, startYaw - initialDeltaFeetYaw / 2);

      FramePoint leftPosition = new FramePoint(WORLD_FRAME, leftFootStartPoint.getX(), leftFootStartPoint.getY(), 0);
      FramePoint rightPosition = new FramePoint(WORLD_FRAME, rightFootStartPoint.getX(), rightFootStartPoint.getY(), 0);
      FrameOrientation leftOrientation = new FrameOrientation(WORLD_FRAME, leftFootPose2d.getYaw(), 0.0, 0.0);
      FrameOrientation rightOrientation = new FrameOrientation(WORLD_FRAME, rightFootPose2d.getYaw(), 0.0, 0.0);

      leftContactableFoot.setSoleFrame(leftPosition, leftOrientation);
      rightContactableFoot.setSoleFrame(rightPosition, rightOrientation);

      Footstep leftStart = footstepSnapper.generateFootstepWithoutHeightMap(leftFootPose2d, feet.get(RobotSide.LEFT), contactableFeet.get(RobotSide.LEFT).getSoleFrame(),
            RobotSide.LEFT, GROUND_HEIGHT, PLANE_NORMAL);
      Footstep rightStart = footstepSnapper.generateFootstepWithoutHeightMap(rightFootPose2d, feet.get(RobotSide.RIGHT), contactableFeet.get(RobotSide.RIGHT).getSoleFrame(),
            RobotSide.RIGHT, GROUND_HEIGHT, PLANE_NORMAL);
      SideDependentList<Footstep> startFeet = new SideDependentList<Footstep>(leftStart, rightStart);

      return startFeet;
   }

   private void assertFootstepsValid(String testDescription, Visualization visualize, Point2d startWorldPoint, double startWorldYaw, Point2d endWorldPoint,
         SideDependentList<Footstep> startFeet, ArrayList<Footstep> footsteps)
   {
      showFootstepsIfVisualize(testDescription, footsteps, startFeet, visualize);

      double semiCircleOffset = 0.0;
      double validSideOffset = 2 * footSide;
      double radianFootTwistLimit = 1.1 * Math.max(Math.abs(pathType.getTurningCloseStepAngle()), Math.abs(pathType.getTurningOpenStepAngle()));
      double footReachLimitMeters = 1.1 * (pathType.getStepLength() + Math.max(pathType.getStepWidth(), pathType.getTurningStepWidth()));
      FootstepValidityMetric footstepValidityMetric = new SemiCircularStepValidityMetric(leftContactableFoot.getRigidBody(), semiCircleOffset,
            radianFootTwistLimit, footReachLimitMeters, validSideOffset);

      assertAllStepsLevelAndZeroHeight(testDescription, footsteps, zToAnkle);
      assertAllStepsValid(testDescription, footsteps, startFeet, footstepValidityMetric);
      assertNoRepeatedSquareUpSteps(testDescription + " Shouldn't restep at end", footsteps);
      assertNoInitialRepeatedSteps(testDescription, footsteps, startFeet);
      assertLastTwoStepsCenterAroundEndPoint(testDescription, footsteps, endWorldPoint);
   }

   private void assertNoInitialRepeatedSteps(String testDescription, ArrayList<Footstep> footsteps, SideDependentList<Footstep> startFeet)
   {
      if (footsteps.size() <= 2)
         return;

      ArrayList<Footstep> testableFootstepQueue = prependStanceToFootstepQueue(footsteps, startFeet);
      assertNoRepeatedSquareUpSteps(testDescription + " Shouldn't restep at beginning", testableFootstepQueue.subList(0, 4));
   }

   private void showFootstepsIfVisualize(String description, final ArrayList<Footstep> footsteps, SideDependentList<Footstep> startFeet, Visualization visualize)
   {
      if (((visualize == Visualization.VISUALIZE) && (allowVisualization == Visualization.VISUALIZE)) || forceVisualizeAll)
      {         
         int maxNumberOfFootstepsPerSide = 100;
         int maxContactPointsPerFoot = 10;
         GroundProfile3D groundProfile = null;
         Graphics3DObject linkGraphics = null;
         
         FootstepVisualizer footstepVisualizer = new FootstepVisualizer(groundProfile, linkGraphics , maxNumberOfFootstepsPerSide, maxContactPointsPerFoot, description);
         footstepVisualizer.startVisualizer();

         footstepVisualizer.visualizeInitialFootsteps(contactableFeet, startFeet);
         footstepVisualizer.visualizeFootsteps(contactableFeet, footsteps);
         footstepVisualizer.waitForSCSToClose();
      }
   }

   private void assertAllStepsLevelAndZeroHeight(String message, ArrayList<Footstep> footSteps, double zToAnkle)
   {
      double eps = 1e-15;
      for (Footstep footstep : footSteps)
      {
         assertEquals(message + " foot should be level and zero height.", zToAnkle, footstep.getZ(), eps);
      }
   }

   private void assertAllStepsValid(String message, ArrayList<Footstep> footSteps, SideDependentList<Footstep> startFeet,
         FootstepValidityMetric footstepValidityMetric)
   {
      ArrayList<Footstep> testableFootstepQueue = prependStanceToFootstepQueue(footSteps, startFeet);
      for (int i = 0; i < testableFootstepQueue.size() - 2; i++)
      {
         Footstep swingStart = testableFootstepQueue.get(i);
         Footstep stance = testableFootstepQueue.get(i + 1);
         Footstep swingEnd = testableFootstepQueue.get(i + 2);
         footstepValidityMetric.assertValid(message + " Step " + i + " to " + (i + 2) + ".", swingStart, stance, swingEnd);
      }
   }

   private ArrayList<Footstep> prependStanceToFootstepQueue(ArrayList<Footstep> footSteps, SideDependentList<Footstep> currentFootLocations)
   {
      RobotSide firstStepSide = footSteps.get(0).getRobotSide();
      ArrayList<Footstep> footstepQueue = new ArrayList<Footstep>();
      footstepQueue.add(currentFootLocations.get(firstStepSide));
      footstepQueue.add(currentFootLocations.get(firstStepSide.getOppositeSide()));
      footstepQueue.addAll(footSteps);

      return footstepQueue;
   }

   private void assertLastTwoStepsArePathAlignedWithOffset(String testDescription, ArrayList<Footstep> footsteps, Point2d startWorldPoint,
         Point2d endWorldPoint, double startWorldYaw, double pathYaw)
   {
      assertTrue(testDescription + " should have at least two footsteps", footsteps.size() >= 2);

      Point2d pathVector = new Point2d(endWorldPoint);
      pathVector.sub(startWorldPoint);

      assertStepIsPathAlignedWithOffset(testDescription + " Second to last footstep.", footsteps.get(footsteps.size() - 2), pathVector, startWorldYaw, pathYaw);
      assertStepIsPathAlignedWithOffset(testDescription + " Last footstep.", footsteps.get(footsteps.size() - 1), pathVector, startWorldYaw, pathYaw);
   }

   private static void assertStepIsPathAlignedWithOffset(String message, Footstep footstep, Point2d destinationVector, double startWorldYaw,
         double pathRelativeYaw)
   {
      FrameOrientation2d endOrientation;
      if (destinationVector.distance(new Point2d(0, 0)) < 1e-14)
      {
         endOrientation = new FrameOrientation2d(WORLD_FRAME, startWorldYaw);
      }
      else
      {
         double destinationVectorYaw = Math.atan2(destinationVector.getY(), destinationVector.getX());
         endOrientation = new FrameOrientation2d(WORLD_FRAME, destinationVectorYaw + pathRelativeYaw);
      }

      assertStepIsPointingCorrectly(message, footstep, endOrientation);
   }

   private static void assertLastTwoStepsPointingCorrectly(String testDescription, ArrayList<Footstep> footsteps, FrameOrientation2d endOrientation)
   {
      assertTrue(testDescription + " should have at least two footsteps", footsteps.size() >= 2);
      assertStepIsPointingCorrectly(testDescription + " Second to last footstep.", footsteps.get(footsteps.size() - 2), endOrientation);
      assertStepIsPointingCorrectly(testDescription + " Last footstep.", footsteps.get(footsteps.size() - 1), endOrientation);
   }

   private static void assertMiddleStepsPointingCorrectly(String testDescription, ArrayList<Footstep> footsteps, double expectedFootYaw)
   {
      FrameOrientation2d midOrientation = new FrameOrientation2d(WORLD_FRAME, expectedFootYaw);

      int midStep1 = footsteps.size() / 2;
      int midStep2 = footsteps.size() / 2 + 1;

      assertStepIsPointingCorrectly(testDescription + " Mid footstep 1.", footsteps.get(midStep1), midOrientation);
      assertStepIsPointingCorrectly(testDescription + " Mid footstep 2.", footsteps.get(midStep2), midOrientation);
   }

   private void assertNoRepeatedSquareUpSteps(String message, List<Footstep> footsteps)
   {
      int numFootsteps = footsteps.size();
      if (numFootsteps < 4) // True step in place should only have two footsteps and that should be OK.
         return;

      // Assuming last steps are squared up: shouldn't match steps two before.
      for (int i = numFootsteps - 1; i > numFootsteps - 3; i--)
      {
         Footstep lastFootstep = footsteps.get(i);
         Footstep compareToLastFootstep = footsteps.get(i - 2);
         FramePose lastFootstepPose = new FramePose();
         lastFootstep.getSolePose(lastFootstepPose);
         FramePose compareToLastFootstepPose = new FramePose();
         compareToLastFootstep.getSolePose(compareToLastFootstepPose);
         FramePoint2d lastFootstepPosition2d = new FramePoint2d();
         FramePoint2d compareToLastFootstepPosition2d = new FramePoint2d();
         lastFootstepPose.getPosition2dIncludingFrame(lastFootstepPosition2d);
         compareToLastFootstepPose.getPosition2dIncludingFrame(compareToLastFootstepPosition2d);
         double eps = 1e-14;
         assertTrue(
               message + " step " + i + " and " + (i - 2),
               (compareToLastFootstepPosition2d.distance(lastFootstepPosition2d) > eps)
                     || (Math.abs(lastFootstepPose.getYaw() - compareToLastFootstepPose.getYaw()) > eps));
      }
   }

   private static void assertStepIsPointingCorrectly(String message, Footstep footstep, FrameOrientation2d endOrientation)
   {
      FramePose footPose = new FramePose();
      footstep.getSolePose(footPose);

      // System.out.println(Math.toDegrees(endOrientation.getYaw()) + "?=" + Math.toDegrees(footPose.getYaw()));
      // System.out.println(Math.toDegrees(footPose.getRoll()) + "?=" + Math.toDegrees(footPose.getPitch()) + "?=" + 0.0);
      assertEquals(message + " Foot roll should be 0.", 0.0, footPose.getRoll(), 1e-10);
      assertEquals(message + " Foot pitch should be 0.", 0.0, footPose.getPitch(), 1e-10);
      FrameOrientation2d footOrientation = new FrameOrientation2d();
      footPose.getOrientation2dIncludingFrame(footOrientation);
      double yawDiff = endOrientation.sub(footOrientation);
      assertEquals(message + " Foot yaw and desired yaw difference should be 0.", 0.0, yawDiff, 1e-10);
   }

   private void assertLastTwoStepsCenterAroundEndPoint(String testDescription, ArrayList<Footstep> footsteps, Point2d endWorldPoint)
   {
      int numSteps = footsteps.size();
      Footstep lastStep = footsteps.get(numSteps - 1);
      Footstep nextLastStep = footsteps.get(numSteps - 2);

      Vector3d lastStepPosition = new Vector3d();
      Vector3d nextLastStepPosition = new Vector3d();
      Vector3d positionInFrame = new Vector3d();


      lastStep.getSoleReferenceFrame().getTransformToWorldFrame().getTranslation(lastStepPosition);
      nextLastStep.getSoleReferenceFrame().getTransformToWorldFrame().getTranslation(nextLastStepPosition);
      positionInFrame.interpolate(nextLastStepPosition, lastStepPosition, 0.5);
      Point2d positionInFrame2d = new Point2d(positionInFrame.getX(), positionInFrame.getY());

      double endPositionOffset = endWorldPoint.distance(positionInFrame2d);
      assertEquals(testDescription + " footsteps must end near desired end", 0.0, endWorldPoint.distance(positionInFrame2d), endPositionTolerance);
   }

   private void assertStepSide(String message, Footstep footstep, RobotSide expectedSide)
   {
      if (expectedSide != null)
         assertTrue(message + " first step should be " + expectedSide, expectedSide == footstep.getRobotSide());
   }

}
