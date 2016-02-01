package us.ihmc.darpaRoboticsChallenge.driving.imageProcessing;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.JPanel;

import us.ihmc.robotics.geometry.BoundingBox2d;

/**
 * User: Matt
 * Date: 3/7/13
 */
public class LanePositionIndicatorPanel extends JPanel
{
   private BufferedImage carImage;
   private double offset = 0.0;

   private double roadWidthInMeters = 7.34;
   private double carWidthInMeters = 1.5;
   private double steeringWheelOffsetInMeters = 0.3;
   private SteeringInputEstimator steeringInputEstimator;
   private double maxAngle = 90.0;

   private double carWidthInPixels;
   private double pixelsPerMeter;

   private ArrayList<BoundingBox2d> obstacles = new ArrayList<BoundingBox2d>();

   public LanePositionIndicatorPanel(String fileName, SteeringInputEstimator steeringInputEstimator)
   {
      this.setBackground(Color.gray);
      this.steeringInputEstimator = steeringInputEstimator;
      try
      {
         carImage = ImageIO.read(new File(fileName));
         carWidthInPixels = carImage.getWidth();
         pixelsPerMeter = carWidthInPixels / carWidthInMeters;
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void setOffset(double offset)
   {
      this.offset = offset;
      repaint();
   }

   public void setObstacles(ArrayList<BoundingBox2d> obstacles)
   {
      this.obstacles = obstacles;
   }

   public void paint(Graphics g)
   {
      super.paint(g);

      // save original settings for resetting
      Color originalColor = g.getColor();
      Graphics2D g2d = (Graphics2D) g;
      Stroke originalStroke = g2d.getStroke();

      // determine relative sizes
      double carWidthInPixels = carImage.getWidth();
      double panelWidthInPixels = this.getWidth();
      double roadWidthInPixels = convertToPixels(roadWidthInMeters);
      double remainderWidthInPixels = panelWidthInPixels - roadWidthInPixels;
      double midPointOfPanelInPixels = panelWidthInPixels / 2.0;
      double rangeOfMotionInPixels = panelWidthInPixels - carWidthInPixels - remainderWidthInPixels;

      // draw grass edges to get proper scaling
      g.setColor(new Color(20, 125, 57));
      g.fillRect(0, 0, (int) (remainderWidthInPixels / 2.0), this.getHeight());
      g.fillRect((int) (panelWidthInPixels - (remainderWidthInPixels / 2.0)), 0, (int) (remainderWidthInPixels / 2.0), this.getHeight());

      // draw center line
      g.setColor(Color.yellow);
      g2d.setStroke(new BasicStroke(2));
      g.drawLine((int) (panelWidthInPixels / 2.0), 0, (int) (panelWidthInPixels / 2.0), getHeight());

      // draw obstacles
//      System.out.println("drawing " + obstacles.size());
      Color orange = new Color(255, 125, 20);
      Color color = orange;
      for (BoundingBox2d obstacle : obstacles)
      {
         double xPercentageOfRoad = obstacle.getMinPoint().getX();
         double boxWidthPercentageOfRoad = obstacle.getMaxPoint().getX() - xPercentageOfRoad;
         int insetPixels = 8;
         double obstaclePositionInPixels = midPointOfPanelInPixels + (xPercentageOfRoad * rangeOfMotionInPixels / 2.0);
         double obstacleWidthInPixels = boxWidthPercentageOfRoad * (rangeOfMotionInPixels / 2.0);

//         System.out.println("min = " + minX + ": " + maxX + ": " + width + ": " + obstaclePositionInPixels);
         g.setColor(color);
         g.fillRect((int) obstaclePositionInPixels, insetPixels, (int) obstacleWidthInPixels, this.getHeight() - (insetPixels*2));
         color = color.darker();
      }

      // draw vehicle at proper location
      double steeringWheelOffsetInPixels = convertToPixels(steeringWheelOffsetInMeters);
      double carPositionInPixels = midPointOfPanelInPixels - (carWidthInPixels / 2.0) + steeringWheelOffsetInPixels + (offset * rangeOfMotionInPixels / 2.0);
      g.drawImage(carImage, (int) carPositionInPixels, 0, null);

      // steering wheel input
      double steeringAngle = determineSteeringAngle();
      steeringInputEstimator.setAngleInDegrees(maxAngle * steeringAngle);

      g.setColor(originalColor);
      ((Graphics2D) g).setStroke(originalStroke);
   }

   private double determineSteeringAngle()
   {
      double steeringAngle = offset;

//      if(obstacles.size()> 0)
//      {
//         BoundingBox2d boundingBox2d = obstacles.get(0);
//      }

      return steeringAngle;
   }

   private double convertToPixels(double valueInMeters)
   {
      return valueInMeters * pixelsPerMeter;
   }

}
