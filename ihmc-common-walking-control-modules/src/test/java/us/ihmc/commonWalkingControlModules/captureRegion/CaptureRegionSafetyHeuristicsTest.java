package us.ihmc.commonWalkingControlModules.captureRegion;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
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
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.geometry.ConvexPolygon2dCalculator;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameGeometry2dPlotter;
import us.ihmc.robotics.geometry.FrameGeometryTestFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;

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
         EuclidCoreTestTools.assertTuple2DEquals(closestVertex, expectedPointsOnBorder.get(i), 1.0e-6);
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
      double footLength = 0.255;
      double footBack = 0.09;
      double footForward = footLength - footBack;
      double midFootAnkleXOffset = footForward - footLength / 2.0;
      double footWidth = 0.095;
      double kinematicStepRange = 0.6;
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
               transformToParent.getTranslation().set(new Vector3D(0.0, robotSide.negateIfRightSide(0.15), 0.0));
            }
         };
         ankleZUpFrame.update();
         ankleZUpFrames.put(robotSide, ankleZUpFrame);

         FrameConvexPolygon2D footConvexPolygon2d = new FrameConvexPolygon2D(ankleZUpFrame);
         footConvexPolygon2d.addVertex(ankleZUpFrame, footForward, -footWidth / 2.0);
         footConvexPolygon2d.addVertex(ankleZUpFrame, footForward, footWidth / 2.0);
         footConvexPolygon2d.addVertex(ankleZUpFrame, -footBack, footWidth / 2.0);
         footConvexPolygon2d.addVertex(ankleZUpFrame, -footBack, -footWidth / 2.0);
         footConvexPolygon2d.update();
         footPolygons.put(robotSide, footConvexPolygon2d);

         YoFrameConvexPolygon2D yoFootPolygon = new YoFrameConvexPolygon2D(robotSide.getCamelCaseNameForStartOfExpression() + "Foot", "", worldFrame, 4,
               registry);
         footConvexPolygon2d.changeFrame(worldFrame);
         yoFootPolygon.set(footConvexPolygon2d);
         footConvexPolygon2d.changeFrame(ankleZUpFrame);
         yoFootPolygons.put(robotSide, yoFootPolygon);
         Color footColor;
         if (robotSide == RobotSide.LEFT)
            footColor = Color.pink;
         else
            footColor = Color.green;
         YoArtifactPolygon footArtifact = new YoArtifactPolygon(robotSide.getCamelCaseNameForStartOfExpression(), yoFootPolygon, footColor, false);
         yoGraphicsListRegistry.registerArtifact("Feet", footArtifact);
         footArtifacts.put(robotSide, footArtifact);
      }
      final OneStepCaptureRegionCalculator oneStepCaptureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth,
            kinematicStepRange, ankleZUpFrames, registry, null);

      final YoFrameConvexPolygon2D yoCaptureRegion = new YoFrameConvexPolygon2D("captureRegion", "", worldFrame, 50, registry);
      YoArtifactPolygon captureRegionArtifact = new YoArtifactPolygon("CaptureRegion", yoCaptureRegion, Color.BLACK, false);
      yoGraphicsListRegistry.registerArtifact("Capture", captureRegionArtifact);
      final YoEnum<RobotSide> yoSupportSide = new YoEnum<>("supportSide", registry, RobotSide.class);
      final YoDouble swingTimeRemaining = new YoDouble("swingTimeRemaining", registry);
      final YoFramePoint2D yoICP = new YoFramePoint2D("ICP", worldFrame, registry);
      yoGraphicsListRegistry.registerArtifact("ICP", new YoGraphicPosition("ICP", yoICP, 0.02, YoAppearance.Blue(), GraphicType.CROSS).createArtifact());
      final double omega0 = 3.4;

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

            simulationOverheadPlotter.update();
         }
      };
      swingTimeRemaining.addListener(variableChangedListener);
      yoICP.attachVariableChangedListener(variableChangedListener);
      yoSupportSide.addListener(variableChangedListener);

      swingTimeRemaining.set(0.3);
      yoICP.set(0.1, 0.2);
      variableChangedListener.changed(null);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

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
