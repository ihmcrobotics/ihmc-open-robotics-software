package us.ihmc.imageProcessing.driving;

import us.ihmc.utilities.math.geometry.BoundingBox2d;
import us.ihmc.utilities.math.geometry.Line2d;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import java.awt.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

/**
 * User: Matt
 * Date: 3/8/13
 */
public class ObstaclePositionEstimator
{
   private ArrayList<Line2d> lines = new ArrayList<Line2d>();
   private LanePositionIndicatorPanel lanePositionIndicatorPanel;
   private Dimension imageSize = new Dimension(640, 480);
   private Line2d axis = new Line2d(new Point2d(0, 320), new Vector2d(1.0, 0.0));
   private HashMap<Integer, int[]> boundingBoxes = new HashMap<Integer, int[]>();

   public ObstaclePositionEstimator(LanePositionIndicatorPanel lanePositionIndicatorPanel)
   {
      this.lanePositionIndicatorPanel = lanePositionIndicatorPanel;
   }

   public void setLines(ArrayList<Line2d> lines)
   {
      this.lines = lines;
   }

   public void setBoundingBoxes(HashMap<Integer, int[]> boundingBoxes)
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
         if (leftEdgeOfRoad == null && rightEdgeOfRoad == null)
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
      ArrayList<int[]> boundingBoxValues = new ArrayList<int[]>(boundingBoxes.values());
      Collections.reverse(boundingBoxValues);
      ArrayList<BoundingBox2d> obstacles = new ArrayList<BoundingBox2d>();
//      System.out.println("updateObstacles");
      ArrayList<int[]> coveredRanges = new ArrayList<int[]>();
      for (int[] values : boundingBoxValues)
      {
         int minX = values[0];
         int minY = values[1];
         int maxX = values[2];
         int maxY = values[3];

         if (!isCovered(minX, coveredRanges))
         {
            axis = new Line2d(new Point2d(0, minY), new Vector2d(1.0, 0.0));

            double leftXInPixels = leftEdgeOfRoad.intersectionWith(axis).getX();
            double rightXInPixels = rightEdgeOfRoad.intersectionWith(axis).getX();
            double rangeInPixels = rightXInPixels - leftXInPixels;
            double midPointInPixels = leftXInPixels + (rangeInPixels / 2.0);
            double offsetPercentageOfRoad = (minX - midPointInPixels) / (rangeInPixels / 2.0);
            double boxWidthPercentageOfRoad = (maxX - minX) / (rangeInPixels / 2.0);

            //         System.out.println(leftXInPixels + ": " + midPointInPixels + ": " + rightXInPixels + " - range " + rangeInPixels + " - offset = " + offsetPercentageOfRoad + " - width = " + boxWidthPercentageOfRoad);
//            System.out.println("offset = " + offsetPercentageOfRoad + " - width = " + boxWidthPercentageOfRoad);

            BoundingBox2d boundingBox2d = new BoundingBox2d(new Point2d(offsetPercentageOfRoad, 0.0), new Point2d(offsetPercentageOfRoad + boxWidthPercentageOfRoad, 1.0));
            obstacles.add(boundingBox2d);

            coveredRanges.add(new int[]{minX,  maxX});
         }
         else
         {
//            System.out.println("covered");
         }
      }

      return obstacles;
   }

   private boolean isCovered(int minX, ArrayList<int[]> coveredRanges)
   {
      for (int[] range : coveredRanges)
      {
         if (range[0] <= minX && range[1] >= minX)
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
      for (int[] values : boundingBoxes.values())
      {
         int minX = values[0];
         int minY = values[0];
         int maxX = values[0];
         int maxY = values[0];
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
}
