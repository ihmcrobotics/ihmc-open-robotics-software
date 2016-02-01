package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.PlotterGraphics;
import us.ihmc.robotics.math.frames.YoFrameLine2d;

public class YoArtifactLine2d extends Artifact
{
   private static final long serialVersionUID = 5741633846461834438L;
   private final YoFrameLine2d yoFrameLine2d;
   private final PlotterGraphics plotterGraphics = new PlotterGraphics();
   private final Color color;

   private static final int pixels = 2;
   private static final BasicStroke stroke = new BasicStroke(pixels);

   public YoArtifactLine2d(String name, YoFrameLine2d yoFrameLine2d, Color color)
   {
      super(name);
      this.yoFrameLine2d = yoFrameLine2d;
      this.color = color;
   }

   public void draw(Graphics graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (isVisible)
      {
         graphics.setColor(color);
         if (stroke != null)
            ((Graphics2D) graphics).setStroke(stroke);

         double x0 = yoFrameLine2d.getX0();
         double y0 = yoFrameLine2d.getY0();
         double vx = yoFrameLine2d.getVx();
         double vy = yoFrameLine2d.getVy();

         plotterGraphics.setCenter(Xcenter, Ycenter);
         plotterGraphics.setScale(scaleFactor);

         plotterGraphics.drawLineGivenStartAndVector(graphics, x0, y0, vx, vy);
      }
   }

   public void drawLegend(Graphics graphics, int Xcenter, int Ycenter, double scaleFactor)
   {
      graphics.setColor(color);

      //    int pixels = 2;
      if (stroke != null)
         ((Graphics2D) graphics).setStroke(stroke);

      plotterGraphics.setCenter(Xcenter, Ycenter);
      plotterGraphics.setScale(scaleFactor);
      plotterGraphics.drawLineSegment(graphics, 0.0, 0.0, 0.1, 0.1);
   }

   public void drawHistory(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }

   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }

}
