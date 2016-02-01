package us.ihmc.darpaRoboticsChallenge.driving.imageProcessing;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.util.ArrayList;

import javax.vecmath.Point2d;

import us.ihmc.imageProcessing.utilities.PostProcessor;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.Line2d;

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
      Graphics2D g2d = (Graphics2D) graphics;
      Stroke originalStroke = g2d.getStroke();

      // current aim point
//      g2d.setStroke(new BasicStroke(2));
//      graphics.setColor(Color.red);
//      int width = 20 + (int) screenDimension.getWidth() / 2;
//      int height = (int) screenDimension.getHeight() / 2;
//      graphics.drawLine(width - (markerDiameter / 2), height, width + (markerDiameter / 2), height);
//      graphics.drawLine(width, height - (markerDiameter / 2), width, height + (markerDiameter / 2));

      // vanishing point (desired aim point)
      graphics.setColor(color);
      Point2d point = getBestIntersection();
      if (point != null)
      {
         graphics.drawLine((int) point.getX() - markerDiameter / 2, (int) point.getY(), (int) point.getX() + markerDiameter / 2, (int) point.getY());
         graphics.drawLine((int) point.getX(), (int) point.getY() - markerDiameter / 2, (int) point.getX(), (int) point.getY() + markerDiameter / 2);
      }

//      // steering wheel
//      g2d.setStroke(new BasicStroke(4));
//      graphics.setColor(Color.red);
//      graphics.drawOval(195, (int) (screenDimension.getHeight() / 2) + 130, 220, 220);
//
//      // steering input
//      if (point != null)
//      {
//         graphics.setColor(Color.cyan);
//         double angle = -Math.atan2((point.getX() - width), height);
//         graphics.drawArc(195, (int) (screenDimension.getHeight() / 2) + 130, 220, 220, 90,  (int) Math.toDegrees(angle));
//      }

      graphics.setColor(originalGraphicsColor);
      g2d.setStroke(originalStroke);
   }

   private Point2d getBestIntersection()
   {
      ArrayList<Point2d> intersections = getAllIntersections();
      ArrayList<Point2d> intersectionsOnScreen = filterOutOffscreenIntersections(intersections);
      return GeometryTools.averagePoint2ds(intersectionsOnScreen);
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
      ArrayList<Line2d> linesCopy = new ArrayList<Line2d>(lines);

      for (Line2d line : lines)
      {
         int count = 0;
         linesCopy.remove(line);
         for (Line2d line2d : linesCopy)
         {
            count++;
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
