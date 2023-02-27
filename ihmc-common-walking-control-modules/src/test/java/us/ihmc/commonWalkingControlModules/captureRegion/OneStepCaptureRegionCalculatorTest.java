package us.ihmc.commonWalkingControlModules.captureRegion;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.awt.Color;
import java.util.ArrayList;

import javax.swing.JPanel;
import javax.swing.JScrollPane;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
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
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class OneStepCaptureRegionCalculatorTest
{
   private static final boolean PLOT_RESULTS = false;
   private static final boolean WAIT_FOR_BUTTON_PUSH = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame leftAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("leftAnkleZUp");
   private final ReferenceFrame rightAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("rightAnkleZUp");
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<ReferenceFrame>(leftAnkleZUpFrame, rightAnkleZUpFrame);

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
      double footWidth = 0.5;
      double kineamaticStepRange = 3.0;

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kineamaticStepRange,
            ankleZUpFrames, registry, null);
   }

   @Test
   public void testPointsInsideCaptureRegion()
   {
      // do not change parameters
      // expected results are pre-calculated
      double footWidth = 0.5;
      double footLength = 1.0;
      double kineamaticStepRange = 3.0;

      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.2;
      double omega0 = 3.0;

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kineamaticStepRange,
            ankleZUpFrames, registry, null);

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, footWidth / 2.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(worldFrame, Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      FramePoint2D icp = new FramePoint2D(worldFrame, 0.6, -0.5);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();

      ArrayList<FramePoint2D> expectedPointsOnBorder = new ArrayList<FramePoint2D>();
      ReferenceFrame supportAnkleFrame = ankleZUpFrames.get(swingSide.getOppositeSide());

      // Points that are used to construct a approximation of the expected capture region
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 1.504330, -0.705530)); // norm: 1.66156
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 0.682212, -0.705530)); // norm: 0.98142
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 0.682212, -1.116590)); // norm: 1.30851
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 2.716116, -1.273857));
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 2.085382, -2.156660));
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 1.564096, -2.559988));

      double movePointsFactor = 0.02;
      FrameConvexPolygon2D expectedPolygonInside = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(expectedPointsOnBorder));

      ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
      FrameConvexPolygon2D shrunkenPolygon = new FrameConvexPolygon2D();

      shrinker.scaleConvexPolygon(expectedPolygonInside, movePointsFactor, shrunkenPolygon);

      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(shrunkenPolygon, captureRegion));

      ArrayList<FramePoint2D> expectedPointsOutside = new ArrayList<FramePoint2D>();
      movePointsFactor = 1.03;
      expectedPointsOutside.add(new FramePoint2D(supportAnkleFrame, movePointsFactor * 2.716116, movePointsFactor * -1.273857));
      expectedPointsOutside.add(new FramePoint2D(supportAnkleFrame, movePointsFactor * 2.085382, movePointsFactor * -2.156660));
      expectedPointsOutside.add(new FramePoint2D(supportAnkleFrame, movePointsFactor * 1.564096, movePointsFactor * -2.559988));
      movePointsFactor = 0.97;
      expectedPointsOutside.add(new FramePoint2D(supportAnkleFrame, movePointsFactor * 1.504330, movePointsFactor * -0.705530));
      expectedPointsOutside.add(new FramePoint2D(supportAnkleFrame, movePointsFactor * 0.682212, movePointsFactor * -0.705530));
      expectedPointsOutside.add(new FramePoint2D(supportAnkleFrame, movePointsFactor * 0.682212, movePointsFactor * -1.116590));

      for (int i = 0; i < expectedPointsOutside.size(); i++)
      {
         assertFalse(captureRegion.isPointInside(expectedPointsOutside.get(i)));
      }

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
         plotter.addPolygon(shrunkenPolygon, Color.red);
         for (int i = 0; i < expectedPointsOutside.size(); i++)
         {
            plotter.addFramePoint2d(expectedPointsOutside.get(i), Color.cyan);
         }
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
      double kineamaticStepRange = 3.0;

      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.2;
      double omega0 = 3.0;

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kineamaticStepRange,
            ankleZUpFrames, registry, null);

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, footWidth / 2.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(worldFrame, Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      FramePoint2D icp = new FramePoint2D(worldFrame, 0.6, -0.5);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();

      ArrayList<FramePoint2D> expectedPointsOnBorder = new ArrayList<FramePoint2D>();
      ArrayList<FramePoint2D> expectedPointsInside = new ArrayList<FramePoint2D>();
      ReferenceFrame supportAnkleFrame = ankleZUpFrames.get(swingSide.getOppositeSide());

      // Points that are expected to be vertexes on the capture region border
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 1.50433, -0.705530));
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 0.682212, -0.705530));
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 0.682212, -1.11659));

      for (int i = 0; i < expectedPointsOnBorder.size(); i++)
      {
         FramePoint2DBasics closestVertex = captureRegion.getClosestVertexCopy(expectedPointsOnBorder.get(i));
         closestVertex.checkReferenceFrameMatch(expectedPointsOnBorder.get(i));
         EuclidCoreTestTools.assertEquals(closestVertex, expectedPointsOnBorder.get(i), 1.0e-6);
         assertTrue(closestVertex.epsilonEquals(expectedPointsOnBorder.get(i), 10e-7));
      }

      // Points that are expected to be inside the capture region
      expectedPointsInside.add(new FramePoint2D(supportAnkleFrame, 1.50433, -1.11659));

      for (int i = 0; i < expectedPointsInside.size(); i++)
      {
         assertTrue(captureRegion.isPointInside(expectedPointsInside.get(i)));
      }

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
         for (int i = 0; i < expectedPointsInside.size(); i++)
         {
            plotter.addFramePoint2d(expectedPointsInside.get(i), Color.magenta);
         }

         waitForButtonOrPause(testFrame);
      }
   }

   @Test
   public void testCalculationWithICPInFootPolygon()
   {
      double footWidth = 0.5;
      double footLength = 1.0;
      double kineamaticStepRange = 3.0;

      RobotSide swingSide = RobotSide.LEFT;
      double swingTimeRemaining = 0.2;
      double omega0 = 3.0;

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kineamaticStepRange,
            ankleZUpFrames, registry, null);
      // set the cutoff angle such that the reachable region should be a half circle
      captureRegionCalculator.setReachableRegionCutoffAngle(1.0);

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, footWidth / 2.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(worldFrame, Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      // set the icp to be inside the foot polygon
      FramePoint2D icp = new FramePoint2D(worldFrame, 0.0, 0.0);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();

      // check that capture region is reachable region
      ArrayList<FramePoint2D> testPointsInside = new ArrayList<FramePoint2D>();
      ArrayList<FramePoint2D> testPointsOutside = new ArrayList<FramePoint2D>();
      ReferenceFrame supportAnkleFrame = ankleZUpFrames.get(swingSide.getOppositeSide());
      double sign = swingSide == RobotSide.LEFT ? 1.0 : -1.0;

      testPointsInside.add(new FramePoint2D(supportAnkleFrame, kineamaticStepRange * 0.95, sign * (0.1 + footWidth / 2.0)));
      testPointsInside.add(new FramePoint2D(supportAnkleFrame, -kineamaticStepRange * 0.95, sign * (0.1 + footWidth / 2.0)));
      testPointsInside.add(new FramePoint2D(supportAnkleFrame, 0, sign * kineamaticStepRange * 0.95));

      testPointsOutside.add(new FramePoint2D(supportAnkleFrame, kineamaticStepRange * 1.05, sign * (0.1 + footWidth / 2.0)));
      testPointsOutside.add(new FramePoint2D(supportAnkleFrame, -kineamaticStepRange * 1.05, sign * (0.1 + footWidth / 2.0)));
      testPointsOutside.add(new FramePoint2D(supportAnkleFrame, 0, -sign * (0.1 - footWidth / 2.0)));
      testPointsOutside.add(new FramePoint2D(supportAnkleFrame, 0, sign * kineamaticStepRange * 1.05));

      for (int i = 0; i < testPointsInside.size(); i++)
      {
         assertTrue(captureRegion.isPointInside(testPointsInside.get(i)));
      }

      for (int i = 0; i < testPointsOutside.size(); i++)
      {
         assertFalse(captureRegion.isPointInside(testPointsOutside.get(i)));
      }

      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-5, 5, -5, 5);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(supportFootPolygon, Color.black);
         plotter.addPolygon(captureRegion, Color.green);
         for (int i = 0; i < testPointsInside.size(); i++)
         {
            plotter.addFramePoint2d(testPointsInside.get(i), Color.green);
         }
         for (int i = 0; i < testPointsOutside.size(); i++)
         {
            plotter.addFramePoint2d(testPointsOutside.get(i), Color.red);
         }
         waitForButtonOrPause(testFrame);
      }
   }

   @Test
   public void testCalculationWithHighSwingTime()
   {
      // do not change parameters
      // expected results are pre-calculated
      double footWidth = 0.5;
      double footLength = 1.0;
      double kineamaticStepRange = 3.0;

      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.6;
      double omega0 = 3.0;

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kineamaticStepRange,
            ankleZUpFrames, registry, null);

      ArrayList<FramePoint2D> listOfPoints = new ArrayList<FramePoint2D>();
      listOfPoints.add(new FramePoint2D(worldFrame, -footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new FramePoint2D(worldFrame, -footLength / 2.0, footWidth / 2.0));
      listOfPoints.add(new FramePoint2D(worldFrame, footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new FramePoint2D(worldFrame, footLength / 2.0, footWidth / 2.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(listOfPoints));

      FramePoint2D icp = new FramePoint2D(worldFrame, 0.3, -0.5);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();

      ArrayList<FramePoint2D> predictedICPList = new ArrayList<FramePoint2D>();
      for (FramePoint2D cop : listOfPoints)
      {
         FramePoint2D predictedICP = new FramePoint2D();
         CapturePointTools.computeDesiredCapturePointPosition(omega0, swingTimeRemaining, icp, cop, predictedICP);
         predictedICPList.add(predictedICP);
      }

      ReferenceFrame supportAnkleFrame = ankleZUpFrames.get(swingSide.getOppositeSide());
      ArrayList<FramePoint2D> expectedPointsOutside = new ArrayList<FramePoint2D>();
      expectedPointsOutside.add(new FramePoint2D(supportAnkleFrame, 2.0, -1.5));

      for (int i = 0; i < expectedPointsOutside.size(); i++)
      {
         assertFalse(captureRegion.isPointInside(expectedPointsOutside.get(i)));
      }

      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-2, 5, -5, 2);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(supportFootPolygon, Color.black);
         plotter.addPolygon(captureRegion, Color.green);
         for (int i = 0; i < predictedICPList.size(); i++)
         {
            plotter.addFramePoint2d(predictedICPList.get(i), Color.blue);
         }
         for (int i = 0; i < expectedPointsOutside.size(); i++)
         {
            plotter.addFramePoint2d(expectedPointsOutside.get(i), Color.red);
         }

         waitForButtonOrPause(testFrame);
      }
   }

   @Test
   public void testReallyCloseToFoot()
   {
      // do not change parameters
      // expected results are pre-calculated
      double footWidth = 0.5;
      double footLength = 1.0;
      double kineamaticStepRange = 3.0;

      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.2;
      double omega0 = 3.0;

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kineamaticStepRange,
                                                                                                  ankleZUpFrames, registry, null);

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, footWidth / 2.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(worldFrame, Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      FramePoint2D icp = new FramePoint2D(worldFrame, 0.0, -0.5 * footWidth - 0.001);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();

      ArrayList<FramePoint2D> expectedPointsOnBorder = new ArrayList<FramePoint2D>();
      ArrayList<FramePoint2D> expectedPointsOnInside = new ArrayList<FramePoint2D>();

      ReferenceFrame supportAnkleFrame = ankleZUpFrames.get(swingSide.getOppositeSide());
      expectedPointsOnInside.add(new FramePoint2D(supportAnkleFrame, 0.0, -kineamaticStepRange + 0.05));

      // Points that are used to construct a approximation of the expected capture region
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 1.504330, -0.705530)); // norm: 1.66156
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 0.682212, -0.705530)); // norm: 0.98142
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 0.682212, -1.116590)); // norm: 1.30851
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 2.716116, -1.273857));
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 2.085382, -2.156660));
      expectedPointsOnBorder.add(new FramePoint2D(supportAnkleFrame, 1.564096, -2.559988));

      double movePointsFactor = 0.02;
      FrameConvexPolygon2D expectedPolygonInside = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(expectedPointsOnBorder));

      ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
      FrameConvexPolygon2D shrunkenPolygon = new FrameConvexPolygon2D();

      shrinker.scaleConvexPolygon(expectedPolygonInside, movePointsFactor, shrunkenPolygon);

//      assertTrue(ConvexPolygon2dCalculator.isPolygonInside(shrunkenPolygon, captureRegion));

      ArrayList<FramePoint2D> expectedPointsOutside = new ArrayList<FramePoint2D>();
      movePointsFactor = 1.03;
      expectedPointsOutside.add(new FramePoint2D(supportAnkleFrame, movePointsFactor * 2.716116, movePointsFactor * -1.273857));
      expectedPointsOutside.add(new FramePoint2D(supportAnkleFrame, movePointsFactor * 2.085382, movePointsFactor * -2.156660));
      expectedPointsOutside.add(new FramePoint2D(supportAnkleFrame, movePointsFactor * 1.564096, movePointsFactor * -2.559988));
      movePointsFactor = 0.97;
      expectedPointsOutside.add(new FramePoint2D(supportAnkleFrame, movePointsFactor * 1.504330, movePointsFactor * -0.705530));
      expectedPointsOutside.add(new FramePoint2D(supportAnkleFrame, movePointsFactor * 0.682212, movePointsFactor * -0.705530));
      expectedPointsOutside.add(new FramePoint2D(supportAnkleFrame, movePointsFactor * 0.682212, movePointsFactor * -1.116590));

      for (int i = 0; i < expectedPointsOnInside.size(); i++)
         assertTrue(captureRegion.isPointInside(expectedPointsOnInside.get(i)));

//      for (int i = 0; i < expectedPointsOutside.size(); i++)
//      {
//         assertFalse(captureRegion.isPointInside(expectedPointsOutside.get(i)));
//      }

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
         plotter.addPolygon(shrunkenPolygon, Color.red);
         for (int i = 0; i < expectedPointsOutside.size(); i++)
         {
            plotter.addFramePoint2d(expectedPointsOutside.get(i), Color.cyan);
         }
         waitForButtonOrPause(testFrame);
      }
   }

   private class SimpleAnkleZUpReferenceFrame extends ReferenceFrame
   {
      private final Vector3D offset = new Vector3D();

      public SimpleAnkleZUpReferenceFrame(String name)
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
