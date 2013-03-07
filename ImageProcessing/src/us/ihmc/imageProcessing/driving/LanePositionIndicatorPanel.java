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
      double rangeOfMotion = panelWidth - vehicleWidth;
      double midPoint = panelWidth / 2.0;

      double vehiclePosition = midPoint - (vehicleWidth / 2.0) + (offset * rangeOfMotion / 2.0);
      g.drawImage(vehicleImage, (int) vehiclePosition, 0, null);
   }
}
