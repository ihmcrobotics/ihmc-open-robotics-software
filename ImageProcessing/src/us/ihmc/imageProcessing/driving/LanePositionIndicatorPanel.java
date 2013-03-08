package us.ihmc.imageProcessing.driving;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

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

   private double carWidthInPixels;
   private double pixelsPerMeter;

   private int steeringWheelXInImagePixels = 195;

   public LanePositionIndicatorPanel(String fileName)
   {
      this.setBackground(Color.gray);
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

      // draw grass edges to get proper scaling
      g.setColor(new Color(20, 125, 57));
      g.fillRect(0, 0, (int) (remainderWidthInPixels / 2.0), this.getHeight());
      g.fillRect((int) (panelWidthInPixels - (remainderWidthInPixels / 2.0)), 0, (int) (remainderWidthInPixels / 2.0), this.getHeight());

      // draw center line
      g.setColor(Color.yellow);
      g2d.setStroke(new BasicStroke(2));
      g.drawLine((int) (panelWidthInPixels / 2.0), 0, (int) (panelWidthInPixels / 2.0), getHeight());

      // draw vehicle at proper location
      double midPointOfPanelInPixels = panelWidthInPixels / 2.0;
      double rangeOfMotionInPixels = panelWidthInPixels - carWidthInPixels - remainderWidthInPixels;
      double steeringWheelOffsetInPixels = convertToPixels(steeringWheelOffsetInMeters);
      double carPositionInPixels = midPointOfPanelInPixels - (carWidthInPixels / 2.0) + steeringWheelOffsetInPixels + (offset * rangeOfMotionInPixels / 2.0);
      g.drawImage(carImage, (int) carPositionInPixels, 0, null);

      g.setColor(originalColor);
      ((Graphics2D) g).setStroke(originalStroke);
   }

   private double convertToPixels(double valueInMeters)
   {
      return valueInMeters * pixelsPerMeter;
   }
}
