package us.ihmc.imageProcessing.driving;

import us.ihmc.imageProcessing.utilities.PostProcessor;
import us.ihmc.utilities.math.geometry.BoundingBox2d;
import us.ihmc.utilities.math.geometry.Line2d;

import javax.vecmath.Point2d;
import java.awt.*;
import java.util.ArrayList;

/**
 * User: Matt
 * Date: 3/5/13
 */
public class VanishingPointDetector implements PostProcessor
{
   private ArrayList<Line2d> lines = new ArrayList<Line2d>();
   private int markerDiameter = 5;
   private Color color = Color.cyan;
   private Dimension screenDimension = new Dimension(640, 480);

   public VanishingPointDetector(Color color, int markerDiameter)
   {
      this.color = color;
      this.markerDiameter = markerDiameter;
   }

   public void setLines(ArrayList<Line2d> lines)
   {
      this.lines = lines;
   }

   public void clearLines()
   {
      lines.clear();
   }

   public void setScreenDimension(Dimension screenDimension)
   {
      this.screenDimension = screenDimension;
   }

   public void paint(Graphics graphics)
   {
      Color originalGraphicsColor = graphics.getColor();
      graphics.setColor(color);

      Point2d point = getBestIntersection();
      if (point != null)
      {
         graphics.fillOval((int) point.getX(), (int) point.getY(), markerDiameter, markerDiameter);
      }

      graphics.setColor(originalGraphicsColor);
   }

   private Point2d getBestIntersection()
   {
      ArrayList<Point2d> intersections = getAllIntersections();
      ArrayList<Point2d> intersectionsOnScreen = filterOutOffscreenIntersections(intersections);
      Point2d bestIntersection = getMinDistancePoint(intersectionsOnScreen);


      return bestIntersection;
   }

   private Point2d getMinDistancePoint(ArrayList<Point2d> intersectionsOnScreen)
   {
      Point2d bestPoint = null;
      int maxCounter = 0;
      for (Point2d point2d : intersectionsOnScreen)
      {
         int count = 0;
         System.out.println("for " + point2d);
         for (Point2d point2d1 : intersectionsOnScreen)
         {
            double distance = point2d.distance(point2d1);
            System.out.println("distance = " + distance);
            if(distance < markerDiameter * 2)
            {
                count++;
            }
         }
         System.out.println("count = " + count);

         if (bestPoint == null)
         {
            bestPoint = point2d;
            maxCounter = count;
         }
         else
         {
            if (count < maxCounter)
            {
               bestPoint = point2d;
               maxCounter = count;
            }
         }
      }

      return bestPoint;
   }

   private ArrayList<Point2d> filterOutOffscreenIntersections(ArrayList<Point2d> intersections)
   {
      ArrayList<Point2d> intersectionsOnScreen = new ArrayList<Point2d>();
      BoundingBox2d screen = new BoundingBox2d(new Point2d(), new Point2d(screenDimension.getWidth(), screenDimension.getHeight()));
      for (Point2d intersection : intersections)
      {
         if (screen.isInside(intersection))
         {
            intersectionsOnScreen.add(intersection);
         }
      }
      return intersectionsOnScreen;
   }

   private ArrayList<Point2d> getAllIntersections()
   {
      ArrayList<Point2d> intersections = new ArrayList<Point2d>();

      for (Line2d line : lines)
      {
         for (Line2d line2d : lines)
         {
            Point2d intersection = line.intersectionWith(line2d);
            if (intersection != null)
            {
               intersections.add(intersection);
            }
         }
      }

      return intersections;
   }
}
