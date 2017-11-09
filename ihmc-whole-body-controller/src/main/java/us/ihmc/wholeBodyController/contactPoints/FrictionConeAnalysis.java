package us.ihmc.wholeBodyController.contactPoints;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;

import org.apache.commons.math3.stat.descriptive.SummaryStatistics;
import org.apache.commons.math3.util.Precision;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.plotting.artifact.CircleArtifact;
import us.ihmc.graphicsDescription.plotting.artifact.LineArtifact;
import us.ihmc.graphicsDescription.plotting.artifact.PolygonArtifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.PlotterShowHideMenu;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;

public class FrictionConeAnalysis
{
   private static final double friction = 0.8;
   private static final int vectors = 4;

   private static final boolean showPlotter = true;
   private static final int pointsOnCircle = 50;
   private static final double copGridResolution = 0.05;
   private static final boolean edgeOnly = false;
   private static final double viewRange = 0.5;

   /**
    * The cone calculator is taking care of rotating the friction cones. Examples are:</br>
    * {@link NoOffsetFrictionConeCalculator}</br>
    * {@link InteriorAngleFrictionConeCalculator}</br>
    * {@link CenteredFrictionConeCalculator}
    */
   private static final FrictionConeRotationCalculator coneCalculator = new InteriorAngleFrictionConeCalculator();

   private final SummaryStatistics statistics = new SummaryStatistics();

   public FrictionConeAnalysis()
   {
      List<Point2D> contactPoints = createFootShape();
      List<Point2D> copSamples = createCopGrid(contactPoints, copGridResolution, edgeOnly);

      FrictionConeContactForceSolver solver = new FrictionConeContactForceSolver(contactPoints, coneCalculator);

      Map<Point2D, List<Vector2D>> resultMap = new HashMap<>();
      Map<Point2D, List<Vector2D>> referenceMap = new HashMap<>();
      double percent = computePercentAcieved(solver, copSamples, resultMap, referenceMap);

      PrintTools.info("Mean: " + Precision.round(100.0 * percent, 2) + "%");
      if (showPlotter)
      {
         visualizeAndSleep(contactPoints, copSamples, referenceMap, resultMap, coneCalculator);
      }
   }

   private double computePercentAcieved(FrictionConeContactForceSolver solver, List<Point2D> copSamples, Map<Point2D, List<Vector2D>> resultMap,
                                        Map<Point2D, List<Vector2D>> referenceMap)
   {
      statistics.clear();

      for (Point2D cop : copSamples)
      {
         double fullArea = Math.PI * friction * friction;
         double achievedArea = 0.0;

         List<Vector2D> reference = new ArrayList<>();
         List<Vector2D> achieved = new ArrayList<>();

         for (int i = 0; i < pointsOnCircle; i++)
         {
            double angle = 2.0 * Math.PI * i / pointsOnCircle;
            double fx = Math.cos(angle) * friction;
            double fy = Math.sin(angle) * friction;

            Vector2D force = new Vector2D(fx, fy);
            Vector2D achievedForce = new Vector2D();
            if (solver.solveForFixedCoPAndDirection(vectors, friction, cop, force, achievedForce))
            {
               double achievedLength = achievedForce.length();
               achievedArea += Math.PI * achievedLength * achievedLength / pointsOnCircle;
               achieved.add(achievedForce);
            }
            reference.add(force);

            if (referenceMap != null)
            {
               referenceMap.put(cop, reference);
            }
            if (resultMap != null)
            {
               resultMap.put(cop, achieved);
            }
         }

         double percent = achievedArea / fullArea;
         statistics.addValue(percent);
      }

      return statistics.getMean();
   }

   public static void visualizeAndSleep(List<Point2D> contactPoints, List<Point2D> copSamples, Map<Point2D, List<Vector2D>> referenceMap,
                                        Map<Point2D, List<Vector2D>> resultMap, FrictionConeRotationCalculator coneCalculator)
   {
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      for (int contactIdx = 0; contactIdx < contactPoints.size(); contactIdx++)
      {
         Point2D contactPoint = contactPoints.get(contactIdx);

         for (int vectorIdx = 0; vectorIdx < vectors; vectorIdx++)
         {
            Vector3D vectorToPack = new Vector3D();
            coneCalculator.packVector(contactPoints, contactIdx, vectors, vectorIdx, friction, vectorToPack);

            Vector2D direction = new Vector2D(vectorToPack);
            direction.scale(2.0 * copGridResolution / friction);
            Point2D endPoint = new Point2D(contactPoint);
            endPoint.add(direction);
            LineArtifact contactVector = new LineArtifact("Vector" + vectorIdx + "At" + contactIdx, contactPoint, endPoint);
            contactVector.setColor(Color.BLUE);
            contactVector.setLevel(4);
            graphicsListRegistry.registerArtifact("Contact State", contactVector);
         }
      }

      ConvexPolygon2D supportPolygon = new ConvexPolygon2D(contactPoints);
      PolygonArtifact polygonViz = new PolygonArtifact("SupportPolygon", true, Color.LIGHT_GRAY, supportPolygon);
      polygonViz.setLevel(1);
      graphicsListRegistry.registerArtifact("Contact State", polygonViz);

      int copCount = 0;
      for (Point2D cop : copSamples)
      {
         CircleArtifact copViz = new CircleArtifact("CoP" + copCount, cop.getX(), cop.getY(), 0.01, true, Color.BLUE);
         copViz.setLevel(4);
         graphicsListRegistry.registerArtifact("CoPs", copViz);

         int vectorCount = 0;
         for (Vector2D desired : referenceMap.get(cop))
         {
            desired.scale(0.5 * copGridResolution / friction);
            Point2D endPoint = new Point2D(cop);
            endPoint.add(desired);
            LineArtifact desiredForce = new LineArtifact("DesiredForce" + vectorCount + "At" + copCount, cop, endPoint);
            desiredForce.setColor(Color.RED);
            desiredForce.setLevel(2);
            graphicsListRegistry.registerArtifact("Forces", desiredForce);
            vectorCount++;
         }

         vectorCount = 0;
         for (Vector2D achieved : resultMap.get(cop))
         {
            achieved.scale(0.5 * copGridResolution / friction);
            Point2D endPoint = new Point2D(cop);
            endPoint.add(achieved);
            LineArtifact achievedForce = new LineArtifact("AchievedForce" + vectorCount + "At" + copCount, cop, endPoint);
            achievedForce.setColor(Color.GREEN);
            achievedForce.setLevel(3);
            graphicsListRegistry.registerArtifact("Forces", achievedForce);
            vectorCount++;
         }

         copCount++;
      }

      showPlotter(graphicsListRegistry, FrictionConeAnalysis.class.getSimpleName());
   }

   private static void showPlotter(YoGraphicsListRegistry yoGraphicsListRegistry, String windowName)
   {
      Plotter plotter = new Plotter();
      plotter.setViewRange(viewRange);

      ArrayList<ArtifactList> artifactLists = new ArrayList<>();
      yoGraphicsListRegistry.getRegisteredArtifactLists(artifactLists);
      for (ArtifactList artifactList : artifactLists)
      {
         artifactList.setVisible(true);
      }

      JFrame frame = new JFrame(windowName);
      Dimension preferredSize = new Dimension(1000, 650);
      frame.setPreferredSize(preferredSize);

      JCheckBox doneBox = new JCheckBox("Done");
      PlotterShowHideMenu plotterShowHideMenu = new PlotterShowHideMenu(plotter);
      plotter.addArtifactsChangedListener(plotterShowHideMenu);

      JPanel menuFrame = new JPanel();
      menuFrame.add(plotterShowHideMenu, BorderLayout.LINE_START);
      JScrollPane scrollPane = new JScrollPane(menuFrame);

      frame.add(scrollPane, BorderLayout.EAST);
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

   public static List<Point2D> createCopGrid(List<Point2D> contactPoints, double gridResolution, boolean edgeOnly)
   {
      ConvexPolygon2D supportPolygon = new ConvexPolygon2D(contactPoints);
      ConvexPolygon2D shrunkPolygon = new ConvexPolygon2D(supportPolygon);
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();
      scaler.scaleConvexPolygon(supportPolygon, 0.9 * gridResolution, shrunkPolygon);

      List<Point2D> copGrid = new ArrayList<>();
      int startX = (int) (supportPolygon.getMinX() / gridResolution) - 1;
      int endX = (int) (supportPolygon.getMaxX() / gridResolution) + 1;
      int startY = (int) (supportPolygon.getMinY() / gridResolution) - 1;
      int endY = (int) (supportPolygon.getMaxY() / gridResolution) + 1;
      for (int xIdx = startX; xIdx <= endX; xIdx++)
      {
         for (int yIdx = startY; yIdx <= endY; yIdx++)
         {
            double x = gridResolution * xIdx;
            double y = gridResolution * yIdx;
            Point2D cop = new Point2D(x, y);
            boolean insidePoint = shrunkPolygon.isPointInside(cop, -1.0e-5) && edgeOnly;
            if (supportPolygon.isPointInside(cop, 1.0e-5) && !insidePoint)
            {
               copGrid.add(cop);
            }
         }
      }
      return copGrid;
   }

   public static List<Point2D> createFootShape()
   {
      List<Point2D> contactPoints = new ArrayList<>();

      contactPoints.add(new Point2D(0.2, 0.1));
      contactPoints.add(new Point2D(0.2, -0.1));
      contactPoints.add(new Point2D(-0.2, -0.1));
      contactPoints.add(new Point2D(-0.2, 0.1));

      moveCentroidToOrigin(contactPoints);
      return contactPoints;
   }

   private static final void moveCentroidToOrigin(List<Point2D> points)
   {
      ConvexPolygon2D footPolygon = new ConvexPolygon2D(points);
      Point2DReadOnly centroid = footPolygon.getCentroid();
      for (Point2D point : points)
      {
         point.sub(centroid);
      }
   }

   public static void main(String[] args)
   {
      new FrictionConeAnalysis();
   }
}
