package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;

import us.ihmc.plotting.Graphics2DAdapter;
import us.ihmc.plotting.PlotterGraphics;
import us.ihmc.robotics.math.frames.YoFrameLine2d;

public class YoArtifactLine2d extends YoArtifact
{
   private static final BasicStroke STROKE = new BasicStroke(2);
   
   private final YoFrameLine2d yoFrameLine2d;
   private final Color color;
   
   private final PlotterGraphics plotterGraphics = new PlotterGraphics();

   public YoArtifactLine2d(String name, YoFrameLine2d yoFrameLine2d, Color color)
   {
      super(name, new double[0], color,
            yoFrameLine2d.getYoX0(), yoFrameLine2d.getYoY0(), yoFrameLine2d.getYoVx(), yoFrameLine2d.getYoVy());
      this.yoFrameLine2d = yoFrameLine2d;
      this.color = color;
   }

   @Override
   public void draw(Graphics2DAdapter graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (isVisible)
      {
         graphics.setColor(color);
         graphics.setStroke(STROKE);

         plotterGraphics.setCenter(Xcenter, Ycenter);
         plotterGraphics.setScale(scaleFactor);
         plotterGraphics.drawLineGivenStartAndVector(graphics, yoFrameLine2d.getX0(), yoFrameLine2d.getY0(), yoFrameLine2d.getVx(), yoFrameLine2d.getVy());
      }
   }

   @Override
   public void drawLegend(Graphics2DAdapter graphics, int centerX, int centerY, double scaleFactor)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);

      graphics.drawLineSegment(-20 + centerX, -5 + centerY, 20 + centerX, 5 + centerY);
   }

   @Override
   public void drawHistory(Graphics2DAdapter graphics2d, int centerX, int centerY, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return null;
   }
}
