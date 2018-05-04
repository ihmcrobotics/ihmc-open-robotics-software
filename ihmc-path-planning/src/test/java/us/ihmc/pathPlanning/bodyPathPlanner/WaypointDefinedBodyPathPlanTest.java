package us.ihmc.pathPlanning.bodyPathPlanner;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JCheckBox;
import javax.swing.JFrame;

import org.junit.Assert;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestName;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.PlotterShowHideMenu;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class WaypointDefinedBodyPathPlanTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean showPlotter = false;
   private static final double epsilon = 1.0e-15;

   @Rule
   public TestName name = new TestName();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleBodyPath()
   {
      WaypointDefinedBodyPathPlan plan = new WaypointDefinedBodyPathPlan();
      List<Point2D> waypoints = new ArrayList<Point2D>();
      waypoints.add(new Point2D(0.0, 0.0));
      waypoints.add(new Point2D(0.5, 0.0));
      waypoints.add(new Point2D(1.0, 1.0));
      plan.setWaypoints(waypoints);
      plan.compute(null, null);

      // test path length method
      double segmentLength1 = 0.5;
      double segmentLength2 = Math.sqrt(0.5 * 0.5 + 1.0 * 1.0);
      double toalLength = segmentLength1 + segmentLength2;
      Assert.assertEquals(toalLength, plan.computePathLength(0.0), epsilon);
      Assert.assertEquals(segmentLength1 / 2.0 + segmentLength2, plan.computePathLength(0.5 * segmentLength1 / toalLength), epsilon);
      Assert.assertEquals(segmentLength2, plan.computePathLength(segmentLength1 / toalLength), epsilon);
      Assert.assertEquals(segmentLength2 / 2.0, plan.computePathLength(1.0 - 0.5 * segmentLength2 / toalLength), epsilon);
      Assert.assertEquals(0.0, plan.computePathLength(1.0), epsilon);

      // test point along path method
      Pose2D testPose = new Pose2D();
      plan.getPointAlongPath(0.0, testPose);
      EuclidCoreTestTools.assertTuple2DEquals(waypoints.get(0), testPose.getPosition(), epsilon);
      plan.getPointAlongPath(segmentLength1 / toalLength, testPose);
      EuclidCoreTestTools.assertTuple2DEquals(waypoints.get(1), testPose.getPosition(), epsilon);
      plan.getPointAlongPath(1.0, testPose);
      EuclidCoreTestTools.assertTuple2DEquals(waypoints.get(2), testPose.getPosition(), epsilon);

      // test get closest point method
      double d1 = plan.getClosestPoint(new Point2D(-1.0, 0.0), testPose);
      EuclidCoreTestTools.assertTuple2DEquals(waypoints.get(0), testPose.getPosition(), epsilon);
      Assert.assertEquals(0.0, d1, epsilon);
      double d2 = plan.getClosestPoint(new Point2D(10.0, 0.0), testPose);
      EuclidCoreTestTools.assertTuple2DEquals(waypoints.get(2), testPose.getPosition(), epsilon);
      Assert.assertEquals(1.0, d2, epsilon);
      double d3 = plan.getClosestPoint(new Point2D(10.0, -10.0), testPose);
      EuclidCoreTestTools.assertTuple2DEquals(waypoints.get(1), testPose.getPosition(), epsilon);
      Assert.assertEquals(segmentLength1 / toalLength, d3, epsilon);
      double d4 = plan.getClosestPoint(new Point2D(0.25, 0.1), testPose);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.25, 0.0), testPose.getPosition(), epsilon);
      Assert.assertEquals(0.5 * segmentLength1 / toalLength, d4, epsilon);
      double d5 = plan.getClosestPoint(new Point2D(0.75 + 1.0, 0.5 - 0.5), testPose);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.75, 0.5), testPose.getPosition(), epsilon);
      Assert.assertEquals(1.0 - 0.5 * segmentLength2 / toalLength, d5, epsilon);

      if (showPlotter)
      {
         showPlotter(plan, name.getMethodName());
      }
   }

   public static void showPlotter(WaypointDefinedBodyPathPlan plan, String testName)
   {
      int markers = 5;
      YoVariableRegistry registry = new YoVariableRegistry(testName);
      YoGraphicsListRegistry graphicsList = new YoGraphicsListRegistry();
      for (int i = 0; i < markers; i++)
      {
         double alpha = (double) i / (double) (markers - 1);
         Pose2D pose = new Pose2D();
         plan.getPointAlongPath(alpha, pose);
         YoFramePoint3D yoStartPoint = new YoFramePoint3D("PointStart" + i, worldFrame, registry);
         yoStartPoint.set(pose.getX(), pose.getY(), 0.0);

         double length = 0.1;
         YoFrameVector3D direction = new YoFrameVector3D("Direction" + i, worldFrame, registry);
         direction.set(length * Math.cos(pose.getYaw()), length * Math.sin(pose.getYaw()), 0.0);
         YoFramePoint3D yoEndPoint = new YoFramePoint3D("PointEnd" + i, worldFrame, registry);
         yoEndPoint.set(yoStartPoint);
         yoEndPoint.add(direction);

         YoGraphicPosition poseStartVis = new YoGraphicPosition("PointStart" + i, yoStartPoint, 0.01, YoAppearance.Blue());
         YoGraphicPosition poseEndVis = new YoGraphicPosition("PointEnd" + i, yoEndPoint, 0.01, YoAppearance.Red());
         graphicsList.registerArtifact(testName, poseStartVis.createArtifact());
         graphicsList.registerArtifact(testName, poseEndVis.createArtifact());
      }

      showPlotter(graphicsList, testName);
   }

   public static void showPlotter(YoGraphicsListRegistry yoGraphicsListRegistry, String windowName)
   {
      Plotter plotter = new Plotter();
      plotter.setViewRange(2.0);

      ArrayList<ArtifactList> artifactLists = new ArrayList<>();
      yoGraphicsListRegistry.getRegisteredArtifactLists(artifactLists);
      for (ArtifactList artifactList : artifactLists)
      {
         artifactList.setVisible(true);
      }

      JFrame frame = new JFrame(windowName);
      Dimension preferredSize = new Dimension(600, 600);
      frame.setPreferredSize(preferredSize);

      JCheckBox doneBox = new JCheckBox("Done");
      PlotterShowHideMenu plotterShowHideMenu = new PlotterShowHideMenu(plotter);
      plotter.addArtifactsChangedListener(plotterShowHideMenu);

      frame.add(doneBox, BorderLayout.SOUTH);
      frame.add(plotter.getJPanel(), BorderLayout.CENTER);

      frame.setSize(preferredSize);
      frame.setVisible(true);

      yoGraphicsListRegistry.addArtifactListsToPlotter(plotter);

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
      MutationTestFacilitator.facilitateMutationTestForClass(WaypointDefinedBodyPathPlan.class, WaypointDefinedBodyPathPlanTest.class);
   }
}
