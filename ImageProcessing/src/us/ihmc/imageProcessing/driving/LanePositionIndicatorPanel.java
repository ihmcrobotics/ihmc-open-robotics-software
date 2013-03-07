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
   private BufferedImage vehicleImage;
   private double offset = 0.0;

   private double roadWidth = 7.34;
   private double carWidth = 1.5;

   public LanePositionIndicatorPanel(String fileName)
   {
      this.setBackground(Color.gray);
      try
      {
         vehicleImage = ImageIO.read(new File(fileName));
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

      double vehicleWidth = vehicleImage.getWidth();
      double panelWidth = this.getWidth();
      double midPoint = panelWidth / 2.0;

      double scaledWidth = vehicleWidth / (carWidth / roadWidth);
      double remainderWidth = panelWidth - scaledWidth;
      Color originalColor = g.getColor();
      g.setColor(new Color(20,125,57));
      g.fillRect(0, 0, (int) (remainderWidth / 2.0), this.getHeight());
      g.fillRect((int) (panelWidth - (remainderWidth / 2.0)), 0, (int) (remainderWidth / 2.0), this.getHeight());
      g.setColor(originalColor);

      double rangeOfMotion = panelWidth - vehicleWidth - remainderWidth;
      double vehiclePosition = midPoint - (vehicleWidth / 2.0) + (offset * rangeOfMotion / 2.0);
      g.drawImage(vehicleImage, (int) vehiclePosition, 0, null);
   }
}
