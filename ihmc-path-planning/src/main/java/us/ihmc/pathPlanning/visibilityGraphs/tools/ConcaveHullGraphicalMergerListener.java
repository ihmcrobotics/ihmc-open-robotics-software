package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.awt.Color;
import java.util.ArrayList;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.FrameGeometry2dPlotter;
import us.ihmc.robotics.geometry.FrameGeometryTestFrame;

public class ConcaveHullGraphicalMergerListener implements ConcaveHullMergerListener
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FrameGeometry2dPlotter plotter;
   private final Color colorOne = Color.red;
   private final Color colorTwo = Color.green;
   private final Color loopedVerticesColor = Color.blue;

   private final Color startingVertexColorOne = Color.pink;
   private final Color startingVertexColorTwo = Color.cyan;
   private final Color intersectionVertexColorOne = Color.black;
   private final Color intersectionVertexColorTwo = Color.orange;

   private double xMin = -1.0;
   private double xMax = 1.0;
   private double yMin = -1.0;
   private double yMax = 1.0;

   public ConcaveHullGraphicalMergerListener()
   {
      FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);
      plotter = testFrame.getFrameGeometry2dPlotter();
      plotter.setPointPixels(16);
   }

   @Override
   public void originalHulls(ArrayList<Point2D> hullOne, ArrayList<Point2D> hullTwo)
   {
      drawAFrameWithTheHulls("Original Hulls", hullOne, hullTwo);

      drawThePoints(plotter, hullOne, colorOne);
      drawThePoints(plotter, hullTwo, colorTwo);
   }

   @Override
   public void preprocessedHull(ArrayList<Point2D> hullOne, ArrayList<Point2D> hullTwo)
   {
      drawAFrameWithTheHulls("Hulls after preprocessing", hullOne, hullTwo);

      drawThePoints(plotter, hullOne, colorOne);
      drawThePoints(plotter, hullTwo, colorTwo);
   }


   private void drawAFrameWithTheHulls(String name, ArrayList<Point2D> hullOne, ArrayList<Point2D> hullTwo)
   {
      FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(name, xMin, xMax, yMin, yMax);
      FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
      plotter.setPointPixels(16);
      
      drawThePoints(plotter, hullOne, colorOne);
      drawTheEdges(plotter, hullOne, colorOne);

      drawThePoints(plotter, hullTwo, colorTwo);
      drawTheEdges(plotter, hullTwo, colorTwo);

   }
   
   private void drawTheEdges(FrameGeometry2dPlotter plotter, ArrayList<Point2D> hull, Color color)
   {
      Point2D previousPoint = hull.get(hull.size() - 1);
      
      for (int i=0; i<hull.size(); i++)
      {
         Point2D nextPoint = hull.get(i);
         
         LineSegment2D lineSegment = new LineSegment2D(previousPoint, nextPoint);
         plotter.addFrameLineSegment2d(new FrameLineSegment2D(worldFrame, lineSegment), color);

         previousPoint = nextPoint;
      }
      
   }

   private static void drawThePoints(FrameGeometry2dPlotter plotter, ArrayList<Point2D> hull, Color color)
   {
      for (Point2D point : hull)
      {
         plotter.addFramePoint2d(new FramePoint2D(ReferenceFrame.getWorldFrame(), point), color);
      }
   }


   @Override
   public void foundStartingVertexAndWorkingHull(Point2D startingVertex, ArrayList<Point2D> workingHull, boolean workingHullIsOne)
   {
      if (workingHullIsOne)
      {
         plotter.addFramePoint2d(new FramePoint2D(worldFrame, startingVertex), startingVertexColorOne);
      }
      else
      {
         plotter.addFramePoint2d(new FramePoint2D(worldFrame, startingVertex), startingVertexColorTwo);
      }

   }

   @Override
   public void consideringWorkingEdge(LineSegment2D workingEdge, boolean workingHullIsOne)
   {
      plotter.addFrameLineSegment2d(new FrameLineSegment2D(worldFrame, workingEdge), getColor(workingHullIsOne));
   }

   private Color getColor(boolean workingHullIsOne)
   {
      if (workingHullIsOne)
         return colorOne;
      return colorTwo;
   }

   @Override
   public void foundIntersectionPoint(Point2D intersectionPoint, boolean workingHullIsOne)
   {
      if (workingHullIsOne)
      {
         plotter.addFramePoint2d(new FramePoint2D(worldFrame, intersectionPoint), intersectionVertexColorOne);
      }
      else
      {
         plotter.addFramePoint2d(new FramePoint2D(worldFrame, intersectionPoint), intersectionVertexColorTwo);
      }
   }

   @Override
   public void hullGotLooped(ArrayList<Point2D> hullOne, ArrayList<Point2D> hullTwo, ArrayList<Point2D> mergedVertices)
   {
      FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame("Caught Loop!", xMin, xMax, yMin, yMax);
      FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
      plotter.setPointPixels(16);

      for (Point2D point : hullOne)
      {
         plotter.addFramePoint2d(new FramePoint2D(worldFrame, point), colorOne);
      }

      for (Point2D point : hullTwo)
      {
         plotter.addFramePoint2d(new FramePoint2D(worldFrame, point), colorTwo);
      }

      plotter.setPointPixels(8);

      for (Point2D point : mergedVertices)
      {
         plotter.addFramePoint2d(new FramePoint2D(worldFrame, point), loopedVerticesColor);
      }

      System.out.println("\n\n");
      ConcaveHullMerger.printHull("hullOne", hullOne);
      System.out.println("\n\n");
      ConcaveHullMerger.printHull("hullTwo", hullTwo);
   }

   @Override
   public void hullsAreInvalid(ArrayList<Point2D>... invalidHulls)
   {
      LogTools.error("Got some invalid hulls.");
   }

}
