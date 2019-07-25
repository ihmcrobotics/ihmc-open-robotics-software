package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.awt.Color;
import java.util.ArrayList;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.FrameGeometry2dPlotter;
import us.ihmc.robotics.geometry.FrameGeometryTestFrame;

public class ConcaveHullMergerListener
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

   public ConcaveHullMergerListener()
   {
      FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);
      plotter = testFrame.getFrameGeometry2dPlotter();
      plotter.setPointPixels(16);
   }

   public void preprocessedHull(Point2D[] hullOne, Point2D[] hullTwo)
   {
      for (Point2D point : hullOne)
      {
         plotter.addFramePoint2d(new FramePoint2D(worldFrame, point), colorOne);
      }

      for (Point2D point : hullTwo)
      {
         plotter.addFramePoint2d(new FramePoint2D(worldFrame, point), colorTwo);
      }
   }

   public void foundStartingVertexAndWorkingHull(Point2D startingVertex, Point2D[] workingHull, boolean workingHullIsOne)
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

   public void hullGotLooped(Point2D[] hullOne, Point2D[] hullTwo, ArrayList<Point2D> mergedVertices)
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
   
}
