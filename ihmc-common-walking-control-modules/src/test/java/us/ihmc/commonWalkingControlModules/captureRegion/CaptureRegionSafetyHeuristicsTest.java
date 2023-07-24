package us.ihmc.commonWalkingControlModules.captureRegion;

import com.google.common.util.concurrent.AtomicDouble;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentReachabilityConstraint;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.ConvexPolygon2dCalculator;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameGeometry2dPlotter;
import us.ihmc.robotics.geometry.FrameGeometryTestFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class CaptureRegionSafetyHeuristicsTest
{
   private static final boolean PLOT_RESULTS = false;
   private static final boolean WAIT_FOR_BUTTON_PUSH = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame leftAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("leftAnkleZUp", 0.1);
   private final ReferenceFrame rightAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("rightAnkleZUp", -0.1);
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>(leftAnkleZUpFrame, rightAnkleZUpFrame);

   private final YoRegistry registry = new YoRegistry("CaptureRegionCalculatorTest");

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @SuppressWarnings("unused")

   @Test
   public void testConstructor()
   {
      double kineamaticStepRange = 3.0;

      CaptureRegionSafetyHeuristics captureRegionCalculator = new CaptureRegionSafetyHeuristics(() -> kineamaticStepRange, registry);
   }

   @Test
   public void testPointsToTheLeftOfRightStance()
   {
      double footWidth = 0.1;
      double footLength = 0.2;
      double kinematicStepRange = 1.0;

      RobotSide swingSide = RobotSide.LEFT;
      double swingTimeRemaining = 0.1;
      double omega0 = 3.0;

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kinematicStepRange,
            ankleZUpFrames, registry, null);
      CaptureRegionSafetyHeuristics heuristics = new CaptureRegionSafetyHeuristics(() -> kinematicStepRange, registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, footWidth / 2.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(ankleZUpFrames.get(swingSide.getOppositeSide()), Vertex2DSupplier.asVertex2DSupplier(listOfPoints));
      supportFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      FramePoint2D icp = new FramePoint2D(ankleZUpFrames.get(swingSide.getOppositeSide()), 0.0, 0.08);
      icp.changeFrameAndProjectToXYPlane(worldFrame);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();

      heuristics.computeCaptureRegionWithSafetyHeuristics(swingSide.getOppositeSide(),
                                                          icp, supportFootPolygon.getCentroid(), captureRegion);

      for (int i = 0; i < heuristics.getCaptureRegionWithSafetyMargin().getNumberOfVertices(); i++)
      {
         FramePoint2D vertex = new FramePoint2D(heuristics.getCaptureRegionWithSafetyMargin().getVertex(i));
         vertex.changeFrameAndProjectToXYPlane(captureRegion.getReferenceFrame());
         if (!captureRegion.isPointInside(vertex))
            assertEquals(vertex.distanceFromOrigin(), kinematicStepRange, 1e-2);
      }

      assertTheShrunkenRegionIsInTheUnshrunkenRegion(captureRegion, heuristics.getCaptureRegionWithSafetyMargin(), 1e-5);

      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-5, 5, -5, 5);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(supportFootPolygon, Color.black);
         plotter.addPolygon(captureRegion, Color.green);
         plotter.addFramePoint2d(icp, Color.blue);
         plotter.addPolygon(heuristics.getCaptureRegionWithSafetyMargin(), Color.red);

         for (int i = 0; i < captureRegion.getNumberOfVertices(); i++)
            plotter.addFramePoint2d(captureRegion.getVertex(i), Color.green);
         for (int i = 0; i < heuristics.getCaptureRegionWithSafetyMargin().getNumberOfVertices(); i++)
            plotter.addFramePoint2d(heuristics.getCaptureRegionWithSafetyMargin().getVertex(i), Color.red);

         waitForButtonOrPause(testFrame);
      }

   }

   @Test
   public void testPointsInsideCaptureRegion()
   {
      double footWidth = 0.1;
      double footLength = 0.2;
      double kinematicStepRange = 1.0;

      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.1;
      double omega0 = 3.0;

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kinematicStepRange,
                                                                                                  ankleZUpFrames, registry, null);
      CaptureRegionSafetyHeuristics heuristics = new CaptureRegionSafetyHeuristics(() -> kinematicStepRange, registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, footWidth / 2.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(ankleZUpFrames.get(swingSide.getOppositeSide()), Vertex2DSupplier.asVertex2DSupplier(listOfPoints));
      supportFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      FramePoint2D icp = new FramePoint2D(worldFrame, 0.3, -0.2);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();

      heuristics.computeCaptureRegionWithSafetyHeuristics(swingSide.getOppositeSide(),
                                                          icp, supportFootPolygon.getCentroid(), captureRegion);

      assertTheShrunkenRegionIsInTheUnshrunkenRegion(captureRegion, heuristics.getCaptureRegionWithSafetyMargin(), 5e-3);

      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-5, 5, -5, 5);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(supportFootPolygon, Color.black);
         plotter.addPolygon(captureRegion, Color.green);
         plotter.addFramePoint2d(icp, Color.blue);
         plotter.addPolygon(heuristics.getCaptureRegionWithSafetyMargin(), Color.red);

         for (int i = 0; i < captureRegion.getNumberOfVertices(); i++)
            plotter.addFramePoint2d(captureRegion.getVertex(i), Color.green);
         for (int i = 0; i < heuristics.getCaptureRegionWithSafetyMargin().getNumberOfVertices(); i++)
            plotter.addFramePoint2d(heuristics.getCaptureRegionWithSafetyMargin().getVertex(i), Color.red);

         waitForButtonOrPause(testFrame);
      }

   }

   @Test
   public void testPointNearFoot()
   {
      double footWidth = 0.03;
      double footLength = 0.13;
      double kinematicStepRange = 1.0;

      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.564;
      double omega0 = 3.0;

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kinematicStepRange,
                                                                                                  ankleZUpFrames, registry, null);
      CaptureRegionSafetyHeuristics heuristics = new CaptureRegionSafetyHeuristics(() -> kinematicStepRange, registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, footWidth / 2.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(ankleZUpFrames.get(swingSide.getOppositeSide()), Vertex2DSupplier.asVertex2DSupplier(listOfPoints));
      supportFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      FramePoint2D icp = new FramePoint2D(ankleZUpFrames.get(swingSide.getOppositeSide()), 0.05, -0.06);
      icp.changeFrameAndProjectToXYPlane(worldFrame);

      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();

      heuristics.computeCaptureRegionWithSafetyHeuristics(swingSide.getOppositeSide(),
                                                          icp, supportFootPolygon.getCentroid(), captureRegion);
      assertTheShrunkenRegionIsInTheUnshrunkenRegion(captureRegion, heuristics.getCaptureRegionWithSafetyMargin(), 5e-3);


      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-2, 2, -2, 2);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(supportFootPolygon, Color.black);
         plotter.addPolygon(captureRegion, Color.green);
         plotter.addFramePoint2d(icp, Color.blue);
         plotter.addPolygon(heuristics.getCaptureRegionWithSafetyMargin(), Color.red);
         plotter.addFrameLine2d(heuristics.getLineOfMinimalAction(), Color.BLUE);

         for (int i = 0; i < captureRegion.getNumberOfVertices(); i++)
            plotter.addFramePoint2d(captureRegion.getVertex(i), Color.green);
         for (int i = 0; i < heuristics.getCaptureRegionWithSafetyMargin().getNumberOfVertices(); i++)
            plotter.addFramePoint2d(heuristics.getCaptureRegionWithSafetyMargin().getVertex(i), Color.red);

         waitForButtonOrPause(testFrame);
      }

   }


   @Test
   public void testProjectedFootCorners()
   {
      // do not change parameters
      // expected results are pre-calculated
      double midFootAnkleXOffset = 0.1;
      double footWidth = 0.5;
      double footLength = 1.0;
      double kinematicsStepRange = 3.0;

      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.2;
      double omega0 = 3.0;

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kinematicsStepRange,
            ankleZUpFrames, registry, null);
      CaptureRegionSafetyHeuristics heuristics = new CaptureRegionSafetyHeuristics(() -> kinematicsStepRange, registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, footWidth / 2.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(ankleZUpFrames.get(swingSide.getOppositeSide()), Vertex2DSupplier.asVertex2DSupplier(listOfPoints));
      supportFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      FramePoint2D icp = new FramePoint2D(worldFrame, 0.6, -0.5);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();

      heuristics.computeCaptureRegionWithSafetyHeuristics(swingSide.getOppositeSide(), icp, supportFootPolygon.getCentroid(), captureRegion);
      FrameConvexPolygon2D shrunkenRegion = new FrameConvexPolygon2D(heuristics.getCaptureRegionWithSafetyMargin());


      ArrayList<FramePoint2D> expectedPointsOnBorder = new ArrayList<FramePoint2D>();
      ReferenceFrame supportAnkleFrame = ankleZUpFrames.get(swingSide.getOppositeSide());

      // Points that are expected to be vertexes on the capture region border
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 1.50433, -0.705530));
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 0.682212, -0.705530));
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 0.682212, -1.11659));

      for (int i = 0; i < expectedPointsOnBorder.size(); i++)
      {
         FramePoint2DBasics closestVertex = captureRegion.getClosestVertexCopy(expectedPointsOnBorder.get(i));
         shrunkenRegion.changeFrame(closestVertex.getReferenceFrame());

         closestVertex.checkReferenceFrameMatch(expectedPointsOnBorder.get(i));
         EuclidCoreTestTools.assertEquals(closestVertex, expectedPointsOnBorder.get(i), 1.0e-6);
         assertTrue(closestVertex.epsilonEquals(expectedPointsOnBorder.get(i), 10e-7));

         assertFalse(shrunkenRegion.pointIsOnPerimeter(expectedPointsOnBorder.get(i)));
      }

      assertTheShrunkenRegionIsInTheUnshrunkenRegion(captureRegion, heuristics.getCaptureRegionWithSafetyMargin(), 1e-2);

      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-5, 5, -5, 5);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(supportFootPolygon, Color.black);
         plotter.addPolygon(captureRegion, Color.green);
         plotter.addFramePoint2d(icp, Color.blue);

         for (int i = 0; i < expectedPointsOnBorder.size(); i++)
         {
            plotter.addFramePoint2d(expectedPointsOnBorder.get(i), Color.red);
         }


         waitForButtonOrPause(testFrame);
      }
   }

   @Test
   public void testCaptureRegionIsALine()
   {
      // In this test, the capture region is just a line, because the support polygon is just a line, and there's no swing time remaining. The front point of
      // this line should be the "extra distance" away when the heuristics are applied.

      // do not change parameters
      // expected results are pre-calculated
      double footWidth = 0.5;
      AtomicDouble kinematicsStepRange = new AtomicDouble(3.0);

      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.01;
      double omega0 = 3.0;

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth,
                                                                                                  kinematicsStepRange::get,
                                                                                                  ankleZUpFrames,
                                                                                                  true,
                                                                                                  "",
                                                                                                  registry,
                                                                                                  null);
      CaptureRegionSafetyHeuristics heuristics = new CaptureRegionSafetyHeuristics(kinematicsStepRange::get, registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D());
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(ankleZUpFrames.get(swingSide.getOppositeSide()), Vertex2DSupplier.asVertex2DSupplier(listOfPoints));
      supportFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      FramePoint2D icp = new FramePoint2D(worldFrame, 0.6, -0.5);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();

      heuristics.computeCaptureRegionWithSafetyHeuristics(swingSide.getOppositeSide(), icp, supportFootPolygon.getCentroid(), captureRegion);

      assertTrue(heuristics.getCaptureRegionWithSafetyMargin().signedDistance(icp) > heuristics.extraDistanceToStepFromStanceFoot.getValue());

      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-5, 5, -5, 5);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(supportFootPolygon, Color.black);
         plotter.addPolygon(heuristics.getCaptureRegionWithSafetyMargin(), Color.green);
         plotter.addFramePoint2d(icp, Color.blue);

         waitForButtonOrPause(testFrame);
      }

      // If the capture region line is less than the "extra distance", we can't achieve that full projection. Let's test if we're a little bit there, right at
      // there, and past there
      // make it so that the far point of the line is too far away.
      icp = new FramePoint2D(worldFrame, 0.6, -0.5);

      FramePoint2D nearEdgeOfCaptureRegion = new FramePoint2D(icp);
      nearEdgeOfCaptureRegion.scale(Math.exp(omega0 * swingTimeRemaining));
      double maxDistance = nearEdgeOfCaptureRegion.distanceFromOrigin() + heuristics.extraDistanceToStepFromStanceFoot.getValue() / 2.0;
      kinematicsStepRange.set(maxDistance);
      captureRegionCalculator.calculateReachableRegions();

      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      captureRegion = captureRegionCalculator.getCaptureRegion();

      heuristics.computeCaptureRegionWithSafetyHeuristics(swingSide.getOppositeSide(), icp, supportFootPolygon.getCentroid(), captureRegion);

      double expectedDistance = maxDistance - icp.distanceFromOrigin();
      assertEquals(expectedDistance, heuristics.getCaptureRegionWithSafetyMargin().signedDistance(icp), 1e-3);

      // do it right less than the max
      maxDistance = nearEdgeOfCaptureRegion.distanceFromOrigin() + 1e-4;
      kinematicsStepRange.set(maxDistance);
      captureRegionCalculator.calculateReachableRegions();

      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      captureRegion = captureRegionCalculator.getCaptureRegion();

      heuristics.computeCaptureRegionWithSafetyHeuristics(swingSide.getOppositeSide(), icp, supportFootPolygon.getCentroid(), captureRegion);

      expectedDistance = maxDistance - icp.distanceFromOrigin();
      assertEquals(expectedDistance, heuristics.getCaptureRegionWithSafetyMargin().signedDistance(icp), 1e-3);

      // do it right past the max
      maxDistance = nearEdgeOfCaptureRegion.distanceFromOrigin() - 1e-4;
      kinematicsStepRange.set(maxDistance);
      captureRegionCalculator.calculateReachableRegions();

      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      captureRegion = captureRegionCalculator.getCaptureRegion();

      heuristics.computeCaptureRegionWithSafetyHeuristics(swingSide.getOppositeSide(), icp, supportFootPolygon.getCentroid(), captureRegion);

      expectedDistance = maxDistance - icp.distanceFromOrigin();
      assertEquals(expectedDistance, heuristics.getCaptureRegionWithSafetyMargin().signedDistance(icp), 1e-3);

   }

   @Test
   public void testCaptureRegionIsATriangleFromData()
   {
      // This test is set up with data found on the robot. We want to make sure that it runs properly.
      double footWidth = 0.2;
      double kinematicsStepRange = 3.0;

      RobotSide swingSide = RobotSide.LEFT;
      double swingTimeRemaining = -0.02169507837222262;
      double omega0 = 3.2;
      double feedbackAlpha = 0.9608231064369759;
n
      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kinematicsStepRange,
                                                                                                  ankleZUpFrames, registry, null);
      CaptureRegionSafetyHeuristics heuristics = new CaptureRegionSafetyHeuristics(() -> kinematicsStepRange, registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(worldFrame);
      supportFootPolygon.addVertex(1.3949152009254906, -0.1201878543241284);
      supportFootPolygon.addVertex(1.4006190929921736, -0.025363505698424986);
      supportFootPolygon.addVertex(1.6141713669852293, -0.05575763281036978);
      supportFootPolygon.addVertex(1.6105689088378505, -0.11564669510028772);
      supportFootPolygon.update();

      FramePoint2D icp = new FramePoint2D(worldFrame, 1.903064526465177, 0.23821403187269613);
      FramePoint2D cmp = new FramePoint2D(worldFrame, 1.5584643076526459, -0.06764253854378881);

      supportFootPolygon.scale(cmp, 1.0 - feedbackAlpha);

      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();

      heuristics.computeCaptureRegionWithSafetyHeuristics(swingSide.getOppositeSide(), icp, supportFootPolygon.getCentroid(), captureRegion);

      // Check to make sure that the extra distance projection functioned properly. The ICP should be AT LEAST that distance from the capture region
      assertTrue(heuristics.getCaptureRegionWithSafetyMargin().signedDistance(icp) > heuristics.extraDistanceToStepFromStanceFoot.getValue());
   }

   @Test
   public void testCaptureRegionWithNoTimeLeft()
   {
      // do not change parameters
      // expected results are pre-calculated
      double footLength = 0.2;
      double footWidth = 0.1;
      double kinematicsStepRange = 3.0;

      RobotSide swingSide = RobotSide.LEFT;
      double swingTimeRemaining = 0.0;
      double omega0 = 3.0;

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kinematicsStepRange,
                                                                                                  ankleZUpFrames, registry, null);
      CaptureRegionSafetyHeuristics heuristics = new CaptureRegionSafetyHeuristics(() -> kinematicsStepRange, registry);

      new DefaultParameterReader().readParametersInRegistry(registry);


      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, footWidth / 2.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(ankleZUpFrames.get(swingSide.getOppositeSide()), Vertex2DSupplier.asVertex2DSupplier(listOfPoints));
      supportFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      FramePoint2D icp = new FramePoint2D(worldFrame, footLength / 2.0 - 0.02, 0.4);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();

      heuristics.computeCaptureRegionWithSafetyHeuristics(swingSide.getOppositeSide(), icp, supportFootPolygon.getCentroid(), captureRegion);

      // Check to make sure that the extra distance projection functioned properly. The ICP should be AT LEAST that distance from the capture region
      assertTrue(heuristics.getCaptureRegionWithSafetyMargin().signedDistance(icp) > heuristics.extraDistanceToStepFromStanceFoot.getValue());

      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-5, 5, -5, 5);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(supportFootPolygon, Color.black);
         plotter.addPolygon(captureRegion, Color.red);
         plotter.addPolygon(heuristics.getCaptureRegionWithSafetyMargin(), Color.green);
         plotter.addFramePoint2d(icp, Color.blue);

         waitForButtonOrPause(testFrame);
      }
   }

   @Test
   public void testMaxProjectionDistance()
   {
      double maxDistanceTotal = 1.0;
      double currentDistance = 0.5;

      // if the angle is zero, then it's just continuing along the current line. test this all around the edge
      double angle = 0.0;
      assertEquals(maxDistanceTotal - currentDistance, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = 0.99;
      assertEquals(maxDistanceTotal - currentDistance, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = maxDistanceTotal - 1e-5;
      assertEquals(maxDistanceTotal - currentDistance, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = maxDistanceTotal;
      assertEquals(0.0, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = maxDistanceTotal + 1e-5;
      assertEquals(0.0, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = 1.01;
      assertEquals(0.0, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = 1.5;
      assertEquals(0.0, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);

      // testing a simple right triangle
      maxDistanceTotal = 1.0;
      currentDistance = 0.5;

      angle = Math.PI / 2.0;
      assertEquals(Math.sqrt(MathTools.square(maxDistanceTotal) - MathTools.square(currentDistance)),
                   CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle),
                   1e-5);
      currentDistance = 0.99;
      assertEquals(Math.sqrt(MathTools.square(maxDistanceTotal) - MathTools.square(currentDistance)),
                   CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle),
                   1e-5);
      currentDistance = maxDistanceTotal - 1e-5;
      assertEquals(Math.sqrt(MathTools.square(maxDistanceTotal) - MathTools.square(currentDistance)),
                   CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle),
                   1e-5);
      currentDistance = maxDistanceTotal;
      assertEquals(0.0, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = maxDistanceTotal + 1e-5;
      assertEquals(0.0, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = 1.01;
      assertEquals(0.0, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = 1.5;
      assertEquals(0.0, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);

      // testing with a really shallow projection angle
      maxDistanceTotal = 1.0;
      currentDistance = 0.5;
      angle = 1e-4;
      assertEquals(maxDistanceTotal - currentDistance, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = 0.99;
      assertEquals(maxDistanceTotal - currentDistance, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = maxDistanceTotal - 1e-5;
      assertEquals(maxDistanceTotal - currentDistance, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = maxDistanceTotal;
      assertEquals(0.0, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = maxDistanceTotal + 1e-5;
      assertEquals(0.0, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = 1.01;
      assertEquals(0.0, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);
      currentDistance = 1.5;
      assertEquals(0.0, CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle), 1e-5);


      // test with a bunch of random things
      int iters = 100000;
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         maxDistanceTotal = RandomNumbers.nextDouble(random, 0.0, 1.5);
         currentDistance = RandomNumbers.nextDouble(random, 0.0, 1.5 * maxDistanceTotal);
         angle = RandomNumbers.nextDouble(random, -Math.PI / 2.0, Math.PI / 2.0);

         double maxDistance = CaptureRegionSafetyHeuristics.findMaximumProjectionDistance(currentDistance, maxDistanceTotal, angle);
         double a = Math.PI - Math.abs(angle);
         double b = Math.asin(currentDistance / maxDistanceTotal * Math.sin(a));
         double c = Math.PI - a - b;
         if (currentDistance > maxDistanceTotal)
            assertEquals(0.0, maxDistance, 1e-5);
         else
         {
            assertEquals(currentDistance * Math.sin(c), maxDistance * Math.sin(b), 1e-5);

            double expectedTotal = Math.cos(c) * currentDistance + Math.cos(b) * maxDistance;

            assertEquals(expectedTotal, maxDistanceTotal, 1e-5);
         }
      }
   }

   private static void assertTheShrunkenRegionIsInTheUnshrunkenRegion(FrameConvexPolygon2DReadOnly unshrunkenRegion, FrameConvexPolygon2DReadOnly shrunkenRegion, double epsilon)
   {
      FrameConvexPolygon2D regionToCheck = new FrameConvexPolygon2D(shrunkenRegion);
      regionToCheck.changeFrame(unshrunkenRegion.getReferenceFrame());

      for (int i = 0; i < shrunkenRegion.getNumberOfVertices(); i++)
         assertTrue(unshrunkenRegion.isPointInside(regionToCheck.getVertex(i), epsilon));
   }

   private static class SimpleAnkleZUpReferenceFrame extends ReferenceFrame
   {
      private final Vector3D offset = new Vector3D();

      public SimpleAnkleZUpReferenceFrame(String name, double yOffset)
      {
         super(name, ReferenceFrame.getWorldFrame());
      }

      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.setIdentity();
         transformToParent.getTranslation().set(offset);
      }
   }

   private void waitForButtonOrPause(FrameGeometryTestFrame testFrame)
   {
      if (WAIT_FOR_BUTTON_PUSH)
         testFrame.waitForButtonPush();
      else
         pauseOneSecond();
   }

   private void pauseOneSecond()
   {
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException ex)
      {
      }
   }

   private static void setupVisualizer()
   {
      Robot robot = new Robot("CaptureRegionViz");
      double footLength = 0.2;
      double footWidth = 0.1;
      double kinematicStepRange = 1.0;
      final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
      final SideDependentList<FrameConvexPolygon2D> footPolygons = new SideDependentList<>();
      final SideDependentList<YoFrameConvexPolygon2D> yoFootPolygons = new SideDependentList<>();
      YoRegistry registry = robot.getRobotsYoRegistry();
      final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      final SideDependentList<YoArtifactPolygon> footArtifacts = new SideDependentList<>();
      for (final RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame ankleZUpFrame = new ReferenceFrame(robotSide.getCamelCaseNameForStartOfExpression() + "AnkleZUpFrame", worldFrame)
         {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               transformToParent.getTranslation().set(new Vector3D(0.0, robotSide.negateIfRightSide(0.0), 0.0));
            }
         };
         ankleZUpFrame.update();
         ankleZUpFrames.put(robotSide, ankleZUpFrame);

         FrameConvexPolygon2D footConvexPolygon2d = new FrameConvexPolygon2D(ankleZUpFrame);
         footConvexPolygon2d.addVertex(ankleZUpFrame, footLength / 2.0, -footWidth / 2.0);
         footConvexPolygon2d.addVertex(ankleZUpFrame, footLength / 2.0, footWidth / 2.0);
         footConvexPolygon2d.addVertex(ankleZUpFrame, -footLength / 2.0, footWidth / 2.0);
         footConvexPolygon2d.addVertex(ankleZUpFrame, -footLength / 2.0, -footWidth / 2.0);
         footConvexPolygon2d.update();
         footPolygons.put(robotSide, footConvexPolygon2d);

         YoFrameConvexPolygon2D yoFootPolygon = new YoFrameConvexPolygon2D(robotSide.getCamelCaseNameForStartOfExpression() + "Foot", "", worldFrame, 4,
               registry);
         footConvexPolygon2d.changeFrame(worldFrame);
         yoFootPolygon.set(footConvexPolygon2d);
         footConvexPolygon2d.changeFrame(ankleZUpFrame);
         yoFootPolygons.put(robotSide, yoFootPolygon);
         Color footColor = YoAppearance.DarkGreen().getAwtColor();
         YoArtifactPolygon footArtifact = new YoArtifactPolygon(robotSide.getCamelCaseNameForStartOfExpression(), yoFootPolygon, footColor, true);
         yoGraphicsListRegistry.registerArtifact("Feet", footArtifact);
         footArtifacts.put(robotSide, footArtifact);
      }
      final OneStepCaptureRegionCalculator oneStepCaptureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth,
            kinematicStepRange, ankleZUpFrames, registry, null);
      final CaptureRegionSafetyHeuristics heuristics = new CaptureRegionSafetyHeuristics(() -> kinematicStepRange, registry);

      new DefaultParameterReader().readParametersInRegistry(registry);


      final YoFrameConvexPolygon2D yoCaptureRegion = new YoFrameConvexPolygon2D("captureRegion", "", worldFrame, 50, registry);
      final YoFrameConvexPolygon2D yoCaptureRegionHeuristics = new YoFrameConvexPolygon2D("captureRegionHeuristics", "", worldFrame, 50, registry);
      YoArtifactPolygon captureRegionArtifact = new YoArtifactPolygon("CaptureRegion", yoCaptureRegion, Color.green, false);
      YoArtifactPolygon captureRegionHeuristicsArtifact = new YoArtifactPolygon("CaptureRegionHeuristics", yoCaptureRegionHeuristics, Color.green, false);
      yoGraphicsListRegistry.registerArtifact("Capture", captureRegionArtifact);
      yoGraphicsListRegistry.registerArtifact("Capture", captureRegionHeuristicsArtifact);
      final YoEnum<RobotSide> yoSupportSide = new YoEnum<>("supportSide", registry, RobotSide.class);
      final YoDouble swingTimeRemaining = new YoDouble("swingTimeRemaining", registry);
      final YoFramePoint2D yoICP = new YoFramePoint2D("ICP", worldFrame, registry);
      yoGraphicsListRegistry.registerArtifact("ICP", new YoGraphicPosition("ICP", yoICP, 0.025, YoAppearance.Purple(), GraphicType.BALL_WITH_CROSS).createArtifact());
      final double omega0 = 3.0;

      swingTimeRemaining.set(0.1);
      yoICP.set(0.3, -0.2);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      final SimulationOverheadPlotter simulationOverheadPlotter = new SimulationOverheadPlotter();
      YoVariableChangedListener variableChangedListener = new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            FramePoint2D icp = new FramePoint2D(yoICP);
            RobotSide supportSide = yoSupportSide.getEnumValue();
            yoFootPolygons.get(supportSide.getOppositeSide()).clear();
            footPolygons.get(supportSide).changeFrame(worldFrame);
            yoFootPolygons.get(supportSide).set(footPolygons.get(supportSide));
            footPolygons.get(supportSide).changeFrame(ankleZUpFrames.get(supportSide));
            oneStepCaptureRegionCalculator.calculateCaptureRegion(supportSide.getOppositeSide(), swingTimeRemaining.getDoubleValue(), icp, omega0,
                  footPolygons.get(supportSide));

            FrameConvexPolygon2D frameConvexPolygon2d = new FrameConvexPolygon2D();
            frameConvexPolygon2d.setIncludingFrame(oneStepCaptureRegionCalculator.getCaptureRegion());
            frameConvexPolygon2d.changeFrame(worldFrame);
            yoCaptureRegion.set(frameConvexPolygon2d);

            FramePoint2D stancePosition = new FramePoint2D(footPolygons.get(supportSide).getCentroid());
            stancePosition.changeFrame(worldFrame);
            heuristics.computeCaptureRegionWithSafetyHeuristics(supportSide, yoICP, stancePosition, oneStepCaptureRegionCalculator.getCaptureRegion());
            yoCaptureRegionHeuristics.set(heuristics.getCaptureRegionWithSafetyMargin());

            simulationOverheadPlotter.update();
            scs.tickAndUpdate();
         }
      };
      swingTimeRemaining.addListener(variableChangedListener);
      yoICP.attachVariableChangedListener(variableChangedListener);
      yoSupportSide.addListener(variableChangedListener);



      variableChangedListener.changed(null);


      //      simulationOverheadPlotter.setDrawHistory(false);

      scs.attachPlaybackListener(simulationOverheadPlotter);
      JPanel simulationOverheadPlotterJPanel = simulationOverheadPlotter.getJPanel();
      String plotterName = "Plotter";
      scs.addExtraJpanel(simulationOverheadPlotterJPanel, plotterName, true);
      JPanel plotterKeyJPanel = simulationOverheadPlotter.getJPanelKey();

      JScrollPane scrollPane = new JScrollPane(plotterKeyJPanel);
      scs.addExtraJpanel(scrollPane, "Plotter Legend", false);

      yoGraphicsListRegistry.update();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, false);
      yoGraphicsListRegistry.addArtifactListsToPlotter(simulationOverheadPlotter.getPlotter());
      Thread myThread = new Thread(scs);
      myThread.start();
   }

   public static void main(String[] args)
   {
      setupVisualizer();
   }
}
