package us.ihmc.darpaRoboticsChallenge.driving.imageProcessing;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.imageProcessing.utilities.PostProcessor;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.geometry.Line2d;

/**
 * User: Matt
 * Date: 3/8/13
 */
public class ObstaclePositionEstimator implements PostProcessor
{
   private ArrayList<Line2d> lines = new ArrayList<Line2d>();
   private LanePositionIndicatorPanel lanePositionIndicatorPanel;
   private Dimension imageSize = new Dimension(640, 480);
   private Line2d axis = new Line2d(new Point2d(0, 320), new Vector2d(1.0, 0.0));
   private List<BoundingBox2d> boundingBoxes = new ArrayList<BoundingBox2d>();
   private List<ColoredLine> coloredLines = new ArrayList<ColoredLine>();
   private double roadWidthInMeters = 7.34;
   private double carWidthInMeters = 1.5;
   private double carPercentageOfRoad = carWidthInMeters / roadWidthInMeters;

   public ObstaclePositionEstimator(LanePositionIndicatorPanel lanePositionIndicatorPanel)
   {
      this.lanePositionIndicatorPanel = lanePositionIndicatorPanel;
   }

   public void setLines(ArrayList<Line2d> lines)
   {
      this.lines = lines;
   }

   public void setBoundingBoxes(List<BoundingBox2d> boundingBoxes)
   {
      this.boundingBoxes = boundingBoxes;
      updateLanePositionEstimate();
   }

   public void updateLanePositionEstimate()
   {
      Line2d leftEdgeOfRoad = null;
      Line2d rightEdgeOfRoad = null;
      for (Line2d line : lines)
      {
         if ((leftEdgeOfRoad == null) && (rightEdgeOfRoad == null))
         {
            leftEdgeOfRoad = line;
            rightEdgeOfRoad = line;
         }
         else
         {
            if (line.getSlope() < leftEdgeOfRoad.getSlope())
            {
               leftEdgeOfRoad = line;
            }

            if (line.getSlope() > rightEdgeOfRoad.getSlope())
            {
               rightEdgeOfRoad = line;
            }
         }
      }

      if (leftEdgeOfRoad == null)
      {
         return;
      }

      ArrayList<BoundingBox2d> obstacles = updateObstacles(leftEdgeOfRoad, rightEdgeOfRoad);

      lanePositionIndicatorPanel.setObstacles(obstacles);
   }

   private ArrayList<BoundingBox2d> updateObstacles(Line2d leftEdgeOfRoad, Line2d rightEdgeOfRoad)
   {
      coloredLines.clear();

      Collections.reverse(boundingBoxes);
      ArrayList<BoundingBox2d> obstacles = new ArrayList<BoundingBox2d>();

      // System.out.println("updateObstacles");
      // System.out.println("updateObstacles");
      ArrayList<int[]> coveredRanges = new ArrayList<int[]>();
      for (BoundingBox2d values : boundingBoxes)
      {
         int minX = new Double(values.getMinPoint().x).intValue();
         int minY = new Double(values.getMinPoint().y).intValue();
         int maxX = new Double(values.getMaxPoint().x).intValue();
         int maxY = new Double(values.getMaxPoint().y).intValue();

         if (!isCovered(minX, coveredRanges))
         {
            axis = new Line2d(new Point2d(0, minY), new Vector2d(1.0, 0.0));

            double leftXInPixels = leftEdgeOfRoad.intersectionWith(axis).getX();
            double rightXInPixels = rightEdgeOfRoad.intersectionWith(axis).getX();

            if ((minX > leftXInPixels) && (maxX < rightXInPixels))
            {
               if (rightXInPixels > leftXInPixels)
               {
                  double rangeInPixels = rightXInPixels - leftXInPixels;
                  double midPointInPixels = leftXInPixels + (rangeInPixels / 2.0);
                  double offsetPercentageOfRoad = (minX - midPointInPixels) / (rangeInPixels / 2.0);
                  double boxWidthPercentageOfRoad = (maxX - minX) / (rangeInPixels / 2.0);

                  // System.out.println(leftXInPixels + ": " + midPointInPixels + ": " + rightXInPixels + " - range " + rangeInPixels + " - offset = " + offsetPercentageOfRoad + " - width = " + boxWidthPercentageOfRoad);
                  // System.out.println("offset = " + offsetPercentageOfRoad + " - width = " + boxWidthPercentageOfRoad);

                  BoundingBox2d boundingBox2d = new BoundingBox2d(new Point2d(offsetPercentageOfRoad, 0.0),
                                                   new Point2d(offsetPercentageOfRoad + boxWidthPercentageOfRoad, 1.0));
                  obstacles.add(boundingBox2d);

                  coveredRanges.add(new int[] {minX, maxX});

                  double roadWidthInPixels = rightXInPixels - leftXInPixels;
                  double carWidthInPixels = roadWidthInPixels * carPercentageOfRoad;
                  double leftOpeningInPixels = (minX - leftXInPixels);
                  double rightOpeningInPixels = (rightXInPixels - maxX);
                  Color color = Color.green;

                  if (leftOpeningInPixels > carWidthInPixels)
                  {
                     if (leftOpeningInPixels > rightOpeningInPixels)
                     {
                        color = Color.green;
                     }
                     else
                     {
                        color = Color.yellow;
                     }
                  }
                  else
                  {
                     color = Color.red;
                  }

                  ColoredLine coloredLine = new ColoredLine(color, (int) leftXInPixels, minY, minX, minY);
                  coloredLines.add(coloredLine);

                  if (rightOpeningInPixels > carWidthInPixels)
                  {
                     if (rightOpeningInPixels >= leftOpeningInPixels)
                     {
                        color = Color.green;
                     }
                     else
                     {
                        color = Color.yellow;
                     }
                  }
                  else
                  {
                     color = Color.red;
                  }

                  coloredLine = new ColoredLine(color, maxX, minY, (int) rightXInPixels, minY);
                  coloredLines.add(coloredLine);
               }
            }
         }
         else
         {
            // System.out.println("covered");
         }
      }

      return obstacles;
   }

   public void paint(Graphics graphics)
   {
      Color originalColor = graphics.getColor();
      Graphics2D g2d = (Graphics2D) graphics;
      Stroke originalStroke = g2d.getStroke();
      g2d.setStroke(new BasicStroke(3));

      for (ColoredLine coloredLine : coloredLines)
      {
         graphics.setColor(coloredLine.color);
         graphics.drawLine(coloredLine.x1, coloredLine.y1, coloredLine.x2, coloredLine.y2);
      }

      graphics.setColor(originalColor);
      g2d.setStroke(originalStroke);
   }

   private boolean isCovered(int minX, ArrayList<int[]> coveredRanges)
   {
      for (int[] range : coveredRanges)
      {
         if ((range[0] <= minX) && (range[1] >= minX))
         {
            return true;
         }
      }

      return false;
   }

   private Point2d getClosestObstacle()
   {
      Point2d closestObstacle = new Point2d(0.0, 0.0);
      System.out.println("boundingBox.size() = " + boundingBoxes.size());

      for (BoundingBox2d values : boundingBoxes)
      {
         int minX = new Double(values.getMinPoint().x).intValue();
         int minY = new Double(values.getMinPoint().y).intValue();

         if (minY > closestObstacle.getY())
         {
            closestObstacle = new Point2d(minX, minY);
         }
      }

      System.out.println("closestObstacle = " + closestObstacle);

      return closestObstacle;
   }

   public void setScreenDimension(Dimension screenDimension)
   {
      imageSize = screenDimension;
      axis = new Line2d(new Point2d(0, screenDimension.getHeight()), new Vector2d(1.0, 0.0));
   }

   private class ColoredLine
   {
      int x1;
      int y1;
      int x2;
      int y2;
      Color color;

      private ColoredLine(Color color, int x1, int y1, int x2, int y2)
      {
         this.color = color;
         this.x1 = x1;
         this.x2 = x2;
         this.y1 = y1;
         this.y2 = y2;
      }
   }

}
