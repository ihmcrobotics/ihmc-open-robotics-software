package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Beige;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Blue;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.DarkRed;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.LawnGreen;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.util.ArrayList;
import java.util.Random;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;

import org.apache.commons.lang3.exception.ExceptionUtils;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.SmartCMPProjector.ProjectionMethod;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.PlotterShowHideMenu;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.MutationTestingTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SmartCMPProjectorTest
{
   private static boolean showPlotter = false;
   private static boolean showPlotterOnDirectionChange = false;
   private static boolean showPlotterOnFail = false;

   private static final double epsilon = 1.0e-4;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Random random = new Random(727434726273L);

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testNoProjection1()
   {
      Point2D capturePoint = new Point2D(0.0, 0.0);
      Point2D desiredCMP = new Point2D(0.08, 0.02);
      Point2D expectedProjection = new Point2D(0.08, 0.02);
      doTest(makeFootPolygon(), capturePoint, desiredCMP, expectedProjection, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testNoProjection2()
   {
      Point2D capturePoint = new Point2D(0.0, 0.0);
      Point2D desiredCMP = new Point2D(-0.08, -0.02);
      Point2D expectedProjection = new Point2D(-0.08, -0.02);
      doTest(makeFootPolygon(), capturePoint, desiredCMP, expectedProjection, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSimpleProjection1()
   {
      Point2D capturePoint = new Point2D(-0.05, 0.0);
      Point2D desiredCMP = new Point2D(0.05, -0.1);
      Point2D expectedProjection = new Point2D(0.0, -0.05);
      doTest(makeFootPolygon(), capturePoint, desiredCMP, expectedProjection, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSimpleProjection2()
   {
      Point2D capturePoint = new Point2D(0.05, 0.0);
      Point2D desiredCMP = new Point2D(0.15, 0.0);
      Point2D expectedProjection = new Point2D(0.1, 0.0);
      doTest(makeFootPolygon(), capturePoint, desiredCMP, expectedProjection, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testDiffcultProjection1()
   {
      Point2D capturePoint = new Point2D(-0.06, 0.1);
      Point2D desiredCMP = new Point2D(0.02, -0.1);
      Point2D expectedProjection = new Point2D(0.0, -0.05);
      doTest(makeFootPolygon(), capturePoint, desiredCMP, expectedProjection, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testDiffcultProjection2()
   {
      Point2D capturePoint = new Point2D(-0.03, 0.15);
      Point2D desiredCMP = new Point2D(0.0, 0.1);
      Point2D expectedProjection = new Point2D(0.03, 0.05);
      doTest(makeFootPolygon(), capturePoint, desiredCMP, expectedProjection, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testDiffcultProjection3()
   {
      Point2D capturePoint = new Point2D(-0.03, 0.15);
      Point2D desiredCMP = new Point2D(0.0, 0.1);
      Point2D expectedProjection = new Point2D(0.03, 0.05);
      doTest(makeFootPolygon(), capturePoint, desiredCMP, expectedProjection, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase1()
   {
      Point2D capturePoint = new Point2D(0.0, -0.05);
      Point2D desiredCMP = new Point2D(0.15, -0.15);
      Point2D expectedProjection = new Point2D(0.1, -0.05);
      doTest(makeFootPolygon(), capturePoint, desiredCMP, expectedProjection, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase1WithFinalDesired1()
   {
      Point2D capturePoint = new Point2D(0.0, -0.05);
      Point2D desiredCMP = new Point2D(0.15, -0.15);
      Point2D finalICP = new Point2D(-0.05, 0.0);
      Point2D expectedProjection = new Point2D(0.1, -0.05);
      doTest(makeFootPolygon(), capturePoint, desiredCMP, expectedProjection, finalICP, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase1WithFinalDesired2()
   {
      Point2D capturePoint = new Point2D(0.0, -0.05);
      Point2D desiredCMP = new Point2D(0.15, -0.15);
      Point2D finalICP = new Point2D(-0.05, -0.15);

      ConvexPolygon2d expectedArea = new ConvexPolygon2d();
      expectedArea.addVertex(new Point2D(0.0, -0.05));
      expectedArea.addVertex(new Point2D(0.1, -0.05));
      expectedArea.addVertex(new Point2D(0.05, 0.05));
      expectedArea.update();

      doTest(makeFootPolygon(), capturePoint, desiredCMP, null, finalICP, expectedArea);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase2()
   {
      Point2D capturePoint = new Point2D(0.11, 0.06);
      Point2D desiredCMP = new Point2D(0.08, 0.06);

      ConvexPolygon2d expectedArea = new ConvexPolygon2d();
      expectedArea.addVertex(new Point2D(-0.1, 0.05));
      expectedArea.addVertex(new Point2D(0.1, 0.05));
      expectedArea.update();

      doTest(makeFootPolygon(), capturePoint, desiredCMP, null, null, expectedArea);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase2WithFinalDesired1()
   {
      Point2D capturePoint = new Point2D(0.11, 0.06);
      Point2D desiredCMP = new Point2D(0.08, 0.06);
      Point2D finalICP = new Point2D(0.11, 0.1);

      ConvexPolygon2d expectedArea = new ConvexPolygon2d();
      expectedArea.addVertex(new Point2D(0.08, 0.05));
      expectedArea.addVertex(new Point2D(0.1, 0.05));
      expectedArea.addVertex(new Point2D(0.1, -0.05));
      expectedArea.update();

      doTest(makeFootPolygon(), capturePoint, desiredCMP, null, finalICP, expectedArea);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase2WithFinalDesired2()
   {
      Point2D capturePoint = new Point2D(0.11, 0.06);
      Point2D desiredCMP = new Point2D(0.08, 0.06);
      Point2D finalICP = new Point2D(0.15, 0.06);

      ConvexPolygon2d expectedArea = new ConvexPolygon2d();
      expectedArea.addVertex(new Point2D(-0.1, 0.05));
      expectedArea.addVertex(new Point2D(0.1, 0.05));
      expectedArea.update();

      doTest(makeFootPolygon(), capturePoint, desiredCMP, null, finalICP, expectedArea);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase2WithFinalDesired3()
   {
      Point2D capturePoint = new Point2D(0.11, 0.06);
      Point2D desiredCMP = new Point2D(0.08, 0.06);
      Point2D finalICP = new Point2D(0.12, -0.07);

      ConvexPolygon2d expectedArea = new ConvexPolygon2d();
      expectedArea.addVertex(new Point2D(-0.1, 0.05));
      expectedArea.addVertex(new Point2D(0.1, 0.05));
      expectedArea.update();

      doTest(makeFootPolygon(), capturePoint, desiredCMP, null, finalICP, expectedArea);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase3()
   {
      Point2D capturePoint = new Point2D(0.0, 0.1);
      Point2D desiredCMP = new Point2D(-0.03, 0.15);

      ConvexPolygon2d expectedArea = new ConvexPolygon2d();
      expectedArea.addVertex(new Point2D(-0.1, 0.05));
      expectedArea.addVertex(new Point2D(0.0, 0.05));
      expectedArea.update();

      doTest(makeFootPolygon(), capturePoint, desiredCMP, null, null, expectedArea);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase3WithFinalDesired1()
   {
      Point2D capturePoint = new Point2D(0.0, 0.1);
      Point2D desiredCMP = new Point2D(-0.03, 0.15);
      Point2D finalICP = new Point2D(0.0, 0.15);

      ConvexPolygon2d expectedArea = new ConvexPolygon2d();
      expectedArea.addVertex(new Point2D(-0.1, 0.05));
      expectedArea.addVertex(new Point2D(0.0, 0.05));
      expectedArea.update();

      doTest(makeFootPolygon(), capturePoint, desiredCMP, null, finalICP, expectedArea);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase3WithFinalDesired2()
   {
      Point2D capturePoint = new Point2D(0.0, 0.1);
      Point2D desiredCMP = new Point2D(-0.03, 0.15);
      Point2D finalICP = new Point2D(0.075, 0.0);

      ConvexPolygon2d expectedArea = new ConvexPolygon2d();
      expectedArea.addVertex(new Point2D(-0.1, 0.05));
      expectedArea.addVertex(new Point2D(0.0, 0.05));
      expectedArea.update();

      doTest(makeFootPolygon(), capturePoint, desiredCMP, null, finalICP, expectedArea);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase3WithFinalDesired3()
   {
      Point2D capturePoint = new Point2D(0.0, 0.1);
      Point2D desiredCMP = new Point2D(-0.03, 0.15);
      Point2D finalICP = new Point2D(-0.1, 0.1);

      ConvexPolygon2d expectedArea = new ConvexPolygon2d();
      expectedArea.addVertex(new Point2D(-0.1, 0.05));
      expectedArea.addVertex(new Point2D(0.1, 0.05));
      expectedArea.update();

      doTest(makeFootPolygon(), capturePoint, desiredCMP, null, finalICP, expectedArea);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase4()
   {
      Point2D capturePoint = new Point2D(0.03, 0.06);
      Point2D desiredCMP = new Point2D(0.0, 0.06);

      ConvexPolygon2d expectedArea = new ConvexPolygon2d();
      expectedArea.addVertex(new Point2D(-0.1, 0.05));
      expectedArea.addVertex(new Point2D(0.03, 0.05));
      expectedArea.update();

      doTest(makeFootPolygon(), capturePoint, desiredCMP, null, null, expectedArea);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase4WithFinalDesired1()
   {
      Point2D capturePoint = new Point2D(0.03, 0.06);
      Point2D desiredCMP = new Point2D(0.0, 0.06);
      Point2D finalICP = new Point2D(0.12, -0.07);

      ConvexPolygon2d expectedArea = new ConvexPolygon2d();
      expectedArea.addVertex(new Point2D(-0.1, 0.05));
      expectedArea.addVertex(new Point2D(0.03, 0.05));
      expectedArea.update();

      doTest(makeFootPolygon(), capturePoint, desiredCMP, null, finalICP, expectedArea);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testQuestionableCase5()
   {
      Point2D capturePoint = new Point2D(0.0, -0.06);
      Point2D desiredCMP = new Point2D(0.07, -0.08);

      ConvexPolygon2d expectedArea = new ConvexPolygon2d();
      expectedArea.addVertex(new Point2D(0.0, -0.05));
      expectedArea.addVertex(new Point2D(0.1, -0.05));
      expectedArea.update();

      doTest(makeFootPolygon(), capturePoint, desiredCMP, null, null, expectedArea);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSmallPolygon1()
   {
      Point2D capturePoint = new Point2D(0.0, -0.06);
      Point2D desiredCMP = new Point2D(0.07, -0.08);

      Point2D expectedCMP = new Point2D(0.0, 0.0);
      ConvexPolygon2d footPolygon = makeFootPolygon();
      footPolygon.scale(0.03);

      doTest(footPolygon, capturePoint, desiredCMP, expectedCMP, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSmallPolygon2()
   {
      Point2D capturePoint = new Point2D(0.0, -0.06);
      Point2D desiredCMP = new Point2D(0.07, -0.08);

      Point2D expectedCMP = new Point2D();
      ConvexPolygon2d projectionArea = makeRandomPolygon(0.01);
      projectionArea.getCentroid(expectedCMP);

      doTest(projectionArea, capturePoint, desiredCMP, expectedCMP, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testNoProjectionGenerated()
   {
      if (showPlotter)
      {
         System.out.println("Skipping random tests since showPlotter is requested.");
         return;
      }

      for (int i = 0; i < 100; i++)
      {
         ConvexPolygon2d projectionArea = makeRandomPolygon(0.3);
         Point2D desiredCMP = new Point2D(random.nextDouble() - 0.5, random.nextDouble() - 0.5);
         if (!projectionArea.isPointInside(desiredCMP))
            projectionArea.orthogonalProjection(desiredCMP);
         Point2D capturePoint = new Point2D(0.0, 0.0);
         Point2D expectedProjection = new Point2D(desiredCMP);
         doTest(projectionArea, capturePoint, desiredCMP, expectedProjection, null, null);
      }
   }

   // tests for manual debugging - will always pass but plots can be generated
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRandoms()
   {
      if (showPlotter)
      {
         System.out.println("Skipping random tests since showPlotter is requested.");
         return;
      }

      for (int i = 0; i < 100; i++)
      {
         ConvexPolygon2d projectionArea = makeRandomPolygon(0.3);

         double xCMP = 0.3 * (random.nextDouble() - 0.5);
         double yCMP = 0.3 * (random.nextDouble() - 0.5);
         Point2D desiredCMP = new Point2D(xCMP, yCMP);

         double xICP = 0.3 * (random.nextDouble() - 0.5);
         double yICP = 0.3 * (random.nextDouble() - 0.5);
         Point2D capturePoint = new Point2D(xICP, yICP);

         double xICPFinal = 0.3 * (random.nextDouble() - 0.5);
         double yICPFinal = 0.3 * (random.nextDouble() - 0.5);
         Point2D finalCapturePoint = new Point2D(xICPFinal, yICPFinal);

         doTest(projectionArea, capturePoint, desiredCMP, null, finalCapturePoint, projectionArea);
      }
   }

   // old tests from previous SmartCMPProjector implementation: these serve as regression tests
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRegression1()
   {
      double[][] pointList = new double[][] {{-4.979747892521815, 0.5541117019274466}, {-0.42026607108138236, 1.9379654867165463},
            {6.119471235760535, -3.9598753931171444}, {2.0773903942158043, -6.074548734056259}};
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);
      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -1.0533200632454793, 1.6302464329466666);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, -0.06917411248032934, 7.395654149367431);

      FramePoint2d expectedCMP = new FramePoint2d(worldFrame, -1.032512722097653, 1.7521417728793347);
      doTest(supportPolygon, capturePoint, desiredCMP, expectedCMP, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRegression2()
   {
      double[][] pointList = new double[][] {{-8.200433598264752, 4.736778327900604}, {-7.473324755152609, 8.139207651739621},
            {4.267220732947754, 9.671770258736647}, {4.341938271519323, -4.367767456386648}, {2.389406061561967, -7.47468823044664},
            {-3.2041110411643086, -5.730732619977008}};
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);
      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -7.601990600688118, -4.41238077781491);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, -5.030652009330159, -2.246171870476954);

      FramePoint2d expectedCMP = new FramePoint2d(worldFrame, -4.914184691115587, -2.148054670751448);
      doTest(supportPolygon, capturePoint, desiredCMP, expectedCMP, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRegression3()
   {
      double[][] pointList = new double[][] {{-9.866922926359909, 3.620889108752019}, {-6.106666508735787, 9.488161702372114},
            {7.281277900378459, 2.4430532928556126}, {8.761409532045942, -1.1373581188191206}, {7.705064492576788, -5.27367712948074},
            {6.242601307942824, -7.091872404831583}, {5.513393774314316, -7.496454748096967}, {-1.6921535955371763, -2.4479235906163606}};
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);
      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -5.310400543247988, 0.24850573514182805);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, -6.217613490956406, 0.7306813725318086);

      FramePoint2d expectedCMP = new FramePoint2d(worldFrame, -5.359254928113373, 0.27447140750272714);
      doTest(supportPolygon, capturePoint, desiredCMP, expectedCMP, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRegression4()
   {
      double[][] pointList = new double[][] {{-6.6364442778312505, 5.081037775538617}, {-5.809348495016624, 5.2896472244805715},
            {2.795403317979172, 5.62762983725845}, {-0.47916782691148363, 3.346181735352964}, {-6.593198911501661, 0.0730878970354496}};
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);
      FramePoint2d capturePoint = new FramePoint2d(worldFrame, 6.203278757526469, -4.079058120870805);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, 2.573986759898151, 5.590979195575493);

      FramePoint2d expectedCMP = new FramePoint2d(worldFrame, 2.573986759898151, 5.590979195575493);
      doTest(supportPolygon, capturePoint, desiredCMP, expectedCMP, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRegression5()
   {
      double[][] pointList = new double[][] {{-0.8598263783206956, 8.812003858355197}, {2.9111689851050997, -2.8275592421317626},
            {-7.99259023564723, -6.327559047941249}, {-4.898907389671077, 3.467436032944315}};
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);
      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -1.9885683919677177, -5.49168161377456);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, 9.549202437801455, -5.225841656143075);

      FramePoint2d expectedCMP = new FramePoint2d(worldFrame, 2.911168985099841, -2.827559242128753);
      doTest(supportPolygon, capturePoint, desiredCMP, expectedCMP, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRegression6()
   {
      double[][] pointList = new double[][] {{-7.2707688418724015, -3.012174173134758}, {-6.841964402209621, -1.954448299069142},
            {-0.558517910913384, 8.620706182930885}, {6.195787784134712, 9.748496783539697}, {5.968542879191432, 0.7219539002862714},
            {5.104345092057521, -6.686117070028681}, {-5.2229690037210785, -8.55066113785048}, {-6.611854281359739, -8.7659485302082}};
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);
      FramePoint2d capturePoint = new FramePoint2d(worldFrame, 5.817440017946563, 0.3354567884041124);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, 5.873889085928724, 0.19674124230587786);

      FramePoint2d expectedCMP = new FramePoint2d(worldFrame, 5.873889085928724, 0.19674124230587786);
      doTest(supportPolygon, capturePoint, desiredCMP, expectedCMP, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRegression7()
   {
      double[][] pointList = new double[][] {{-7.803067400320895, 5.2833847867746755}, {1.8456007994090697, 7.299950918772748},
            {5.749971206125327, -0.4548297661618612}, {-5.840882755811931, -1.9631480755498387}};
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);
      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -5.295144331776324, 5.807540241461124);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, -7.791683042751529, 8.691901050356392);

      FramePoint2d expectedCMP = new FramePoint2d(worldFrame, -7.109596433690787, 5.428319821936232);
      doTest(supportPolygon, capturePoint, desiredCMP, expectedCMP, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRegression8()
   {
      double[][] pointList = new double[][] {{-9.515882456803075, -0.7165753982559391}, {-0.36081135780595197, -1.8177034363630717},
            {3.1279637332797563, -2.4061970865397093}, {1.8287460206545418, -4.725313113918093}, {-8.38403587244779, -4.561486274023161}};
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);
      FramePoint2d capturePoint = new FramePoint2d(worldFrame, -1.617696297772003, -4.670027921361948);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, -9.521814853350515, -4.6289424304336);

      FramePoint2d expectedCMP = new FramePoint2d(worldFrame, -8.456495592368146, -4.315338811064966);
      doTest(supportPolygon, capturePoint, desiredCMP, expectedCMP, null, null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testRegression9()
   {
      double[][] pointList = new double[][] {{-8.135927687065115, -2.235069333986268}, {4.626495043779892, -9.594908447084016}};
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame, pointList);
      FramePoint2d capturePoint = new FramePoint2d(worldFrame, 4.779252571336825, -2.7041636789385564);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame, -5.046299707229876, -4.103110230736704);

      FramePoint2d expectedCMP = new FramePoint2d(worldFrame, -4.926263656271011, -4.086019688157568);
      doTest(supportPolygon, capturePoint, desiredCMP, expectedCMP, null, null);
   }

   private boolean checkDirectionPreserved(FramePoint2d capturePoint, FramePoint2d desiredCMP, FramePoint2d projectedCMP)
   {
      FrameVector2d desiredCMPToICP = new FrameVector2d(desiredCMP, capturePoint);
      FrameVector2d projectedCMPToICP = new FrameVector2d(projectedCMP, capturePoint);
      double angleDifference = desiredCMPToICP.angle(projectedCMPToICP);
      return Math.abs(angleDifference) < 1.0e-7;
   }

   private ConvexPolygon2d makeFootPolygon()
   {
      ConvexPolygon2d ret = new ConvexPolygon2d();
      ret.addVertex(new Point2D(0.1, 0.05));
      ret.addVertex(new Point2D(-0.1, 0.05));
      ret.addVertex(new Point2D(0.1, -0.05));
      ret.addVertex(new Point2D(-0.1, -0.05));
      ret.update();
      return ret;
   }

   private ConvexPolygon2d makeRandomPolygon(double size)
   {
      ConvexPolygon2d ret = new ConvexPolygon2d();
      for (int i = 0; i < random.nextInt(10) + 1; i++)
      {
         double x = size * (random.nextDouble() - 0.5);
         double y = size * (random.nextDouble() - 0.5);
         Point2D vertex = new Point2D(x, y);
         ret.addVertex(vertex);
      }
      ret.update();
      return ret;
   }

   private void doTest(ConvexPolygon2d projectionArea, Point2D capturePoint, Point2D desiredCMP, Point2D expectedProjection, Point2D finalCapturePoint,
         ConvexPolygon2d expectedArea)
   {
      FrameConvexPolygon2d projectionAreaWithFrame = new FrameConvexPolygon2d(worldFrame, projectionArea);
      FramePoint2d capturePointWithFrame = new FramePoint2d(worldFrame, capturePoint);
      FramePoint2d desiredCMPWithFrame = new FramePoint2d(worldFrame, desiredCMP);
      FramePoint2d expectedProjectionWithFrame, finalCapturePointWithFrame;
      if (expectedProjection != null)
         expectedProjectionWithFrame = new FramePoint2d(worldFrame, expectedProjection);
      else
         expectedProjectionWithFrame = null;
      if (finalCapturePoint != null)
         finalCapturePointWithFrame = new FramePoint2d(worldFrame, finalCapturePoint);
      else
         finalCapturePointWithFrame = null;
      FrameConvexPolygon2d expectedAreaWithFrame;
      if (expectedArea != null)
         expectedAreaWithFrame = new FrameConvexPolygon2d(worldFrame, expectedArea);
      else
         expectedAreaWithFrame = null;

      doTest(projectionAreaWithFrame, capturePointWithFrame, desiredCMPWithFrame, expectedProjectionWithFrame, finalCapturePointWithFrame,
            expectedAreaWithFrame);
   }

   private void doTest(FrameConvexPolygon2d projectionArea, FramePoint2d capturePoint, FramePoint2d desiredCMP, FramePoint2d expectedProjection,
         FramePoint2d finalCapturePoint, FrameConvexPolygon2d expectedArea)
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      // create and test the projector
      FramePoint2d projectedCMP = new FramePoint2d(desiredCMP);
      SmartCMPProjector projector = new SmartCMPProjector(null, registry);
      projector.projectCMP(projectedCMP, projectionArea, capturePoint, finalCapturePoint);
      ProjectionMethod projectionMethod = projector.getProjectionMethod();

      // do checks on the result
      if (expectedArea != null && expectedProjection != null)
         System.out.println("Got both a expetced area and an expected position. Ignoring the area.");
      boolean directionPreserved = checkDirectionPreserved(capturePoint, desiredCMP, projectedCMP);
      boolean success = true;
      if (expectedProjection != null)
         success = projectedCMP.epsilonEquals(expectedProjection, epsilon);
      else if (expectedArea != null)
         success = expectedArea.isPointInside(projectedCMP, epsilon);

      boolean correctInfo = true;
      boolean cmpMoved = !projectedCMP.epsilonEquals(desiredCMP, epsilon);
      if (!projector.getWasCMPProjected() && cmpMoved)
         correctInfo = false;

      // show overhead results if required
      if ((!success && showPlotterOnFail) || (!correctInfo && showPlotterOnFail) || (!directionPreserved && showPlotterOnDirectionChange) || showPlotter)
      {
         YoFramePoint2d yoCapturePoint = new YoFramePoint2d("CapturePoint", worldFrame, registry);
         YoArtifactPosition capturePointViz = new YoArtifactPosition("Capture Point", yoCapturePoint, GraphicType.BALL_WITH_ROTATED_CROSS, Blue().getAwtColor(),
               0.01);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), capturePointViz);
         yoCapturePoint.set(capturePoint);

         YoFramePoint2d yoDesiredCMP = new YoFramePoint2d("DesiredCMP", worldFrame, registry);
         YoArtifactPosition desiredCMPViz = new YoArtifactPosition("Desired CMP Position", yoDesiredCMP, GraphicType.SOLID_BALL, DarkRed().getAwtColor(),
               0.008);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), desiredCMPViz);
         yoDesiredCMP.set(desiredCMP);

         YoFramePoint2d yoProjectedCMP = new YoFramePoint2d("ProjectedCMP", worldFrame, registry);
         YoArtifactPosition projectedCMPViz = new YoArtifactPosition("Projected CMP Position", yoProjectedCMP, GraphicType.BALL_WITH_ROTATED_CROSS,
               DarkRed().getAwtColor(), 0.01);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), projectedCMPViz);
         yoProjectedCMP.set(projectedCMP);

         YoFrameConvexPolygon2d yoProjectionArea = new YoFrameConvexPolygon2d("CMPProjectionArea", worldFrame, 10, registry);
         YoArtifactPolygon projectionAreaViz = new YoArtifactPolygon("CMP Projection Area", yoProjectionArea, Blue().getAwtColor(), false);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), projectionAreaViz);
         yoProjectionArea.setFrameConvexPolygon2d(projectionArea);

         if (expectedProjection != null)
         {
            YoFramePoint2d yoExpectedCMP = new YoFramePoint2d("ExpectedProjection", worldFrame, registry);
            YoArtifactPosition expectedCMPViz = new YoArtifactPosition("Expected Projection", yoExpectedCMP, GraphicType.BALL, LawnGreen().getAwtColor(),
                  0.01);
            graphicsListRegistry.registerArtifact(getClass().getSimpleName(), expectedCMPViz);
            yoExpectedCMP.set(expectedProjection);
         }

         if (expectedArea != null)
         {
            YoFrameConvexPolygon2d yoExpectedArea = new YoFrameConvexPolygon2d("ExpectedArea", worldFrame, 10, registry);
            YoArtifactPolygon expectedAreaViz = new YoArtifactPolygon("Expected CMP Area", yoExpectedArea, LawnGreen().getAwtColor(), false, 4);
            graphicsListRegistry.registerArtifact(getClass().getSimpleName(), expectedAreaViz);
            yoExpectedArea.setFrameConvexPolygon2d(expectedArea);
         }

         if (finalCapturePoint != null)
         {
            YoFramePoint2d yoFinalCapturePoint = new YoFramePoint2d("FinalCapturePoint", worldFrame, registry);
            YoArtifactPosition finalCapturePointViz = new YoArtifactPosition("Final Capture Point", yoFinalCapturePoint, GraphicType.BALL_WITH_ROTATED_CROSS,
                  Beige().getAwtColor(), 0.01);
            graphicsListRegistry.registerArtifact(getClass().getSimpleName(), finalCapturePointViz);
            yoFinalCapturePoint.set(finalCapturePoint);
         }

         ConvexPolygon2d plotHelperPolygon = new ConvexPolygon2d(projectionArea.getConvexPolygon2d());
         plotHelperPolygon.addVertex(capturePoint.getPoint());
         plotHelperPolygon.addVertex(projectedCMP.getPoint());
         plotHelperPolygon.addVertex(desiredCMP.getPoint());
         if (expectedProjection != null)
            plotHelperPolygon.addVertex(expectedProjection.getPoint());
         if (finalCapturePoint != null)
            plotHelperPolygon.addVertex(finalCapturePoint.getPoint());
         plotHelperPolygon.update();

         Point2DReadOnly focus = plotHelperPolygon.getCentroid();
         double range = 1.2 * Math.max(plotHelperPolygon.getBoundingBoxRangeX(), plotHelperPolygon.getBoundingBoxRangeY());

         String projectionName = projectionMethod.toString();
         String message = "Used method: " + projectionName;
         showPlotter(graphicsListRegistry, focus, range, message);
      }

      // make test fail if projection was not successful
      if (expectedArea != null && expectedProjection != null)
         assertTrue("Expected conditions were not compatibe.", expectedArea.isPointInside(expectedProjection, epsilon));
      assertFalse("This test does not have a fail condition.", expectedArea == null && expectedProjection == null);
      assertTrue("CMP was changed but projector claimed it wasn't projected.", correctInfo);
      assertTrue("Projection of CMP did not equal expected.", success);
   }

   private void showPlotter(YoGraphicsListRegistry graphicsListRegistry, Point2DReadOnly focus, double range, String message)
   {
      String stackTrace = ExceptionUtils.getStackTrace(new RuntimeException());
      Pattern pattern = Pattern.compile(getClass().getSimpleName() + ".test.+?(?=\\()");
      Matcher matcher = pattern.matcher(stackTrace);
      matcher.find();
      String name = matcher.group();

      Plotter plotter = new Plotter();
      plotter.setViewRange(range);
      plotter.setFocusPointX(focus.getX());
      plotter.setFocusPointY(focus.getY());

      ArrayList<ArtifactList> artifactLists = new ArrayList<>();
      graphicsListRegistry.getRegisteredArtifactLists(artifactLists);
      for (ArtifactList artifactList : artifactLists)
         artifactList.setVisible(true);

      JFrame frame = new JFrame(name);
      Dimension preferredSize = new Dimension(1000, 650);
      frame.setPreferredSize(preferredSize);

      JCheckBox doneBox = new JCheckBox("Done");
      PlotterShowHideMenu plotterShowHideMenu = new PlotterShowHideMenu(plotter);
      plotter.addArtifactsChangedListener(plotterShowHideMenu);

      JPanel menuFrame = new JPanel();
      menuFrame.add(plotterShowHideMenu, BorderLayout.LINE_START);
      JScrollPane scrollPane = new JScrollPane(menuFrame);

      if (message != null)
      {
         JLabel messageLabel = new JLabel(message);
         frame.add(messageLabel, BorderLayout.NORTH);
      }

      frame.add(scrollPane, BorderLayout.EAST);
      frame.add(doneBox, BorderLayout.SOUTH);
      frame.add(plotter.getJPanel(), BorderLayout.CENTER);

      frame.setSize(preferredSize);
      frame.setVisible(true);

      graphicsListRegistry.addArtifactListsToPlotter(plotter);

      while (!doneBox.isSelected())
      {
         try
         {
            Thread.sleep(100);
         }
         catch (InterruptedException ex)
         {
         }
      }

      frame.setVisible(false);
      frame.dispose();
   }

   public static void main(String[] args)
   {
      String targetTests = "us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.SmartCMPProjectorTest";
      String targetClasses = "us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.SmartCMPProjector";
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClasses);
   }
}
