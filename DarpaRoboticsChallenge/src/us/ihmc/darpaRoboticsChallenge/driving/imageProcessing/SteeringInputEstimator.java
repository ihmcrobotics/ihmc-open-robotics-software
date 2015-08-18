package us.ihmc.darpaRoboticsChallenge.driving.imageProcessing;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;

import us.ihmc.imageProcessing.utilities.PostProcessor;

/**
 * User: Matt
 * Date: 3/7/13
 */
public class SteeringInputEstimator implements PostProcessor
{
   private Dimension screenDimension = new Dimension(640, 480);
   private double angle = 0.0;

   public void setScreenDimension(Dimension screenDimension)
   {
      this.screenDimension = screenDimension;
   }

   public void setAngleInDegrees(double angle)
   {
      this.angle = angle;
   }

   public void paint(Graphics graphics)
   {
      Color originalGraphicsColor = graphics.getColor();
      Graphics2D g2d = (Graphics2D) graphics;
      Stroke originalStroke = g2d.getStroke();

      // steering wheel
      g2d.setStroke(new BasicStroke(4));
      graphics.setColor(Color.red);
      graphics.drawOval(195, (int) (screenDimension.getHeight() / 2) + 130, 220, 220);

      // steering input
      graphics.setColor(Color.cyan);
      graphics.drawArc(195, (int) (screenDimension.getHeight() / 2) + 130, 220, 220, 90, (int) angle);

      graphics.setColor(originalGraphicsColor);
      g2d.setStroke(originalStroke);
   }
}
