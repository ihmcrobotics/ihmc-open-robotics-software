package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.PlotterGraphics;
import us.ihmc.robotics.math.frames.YoFramePoint;

public class YoArtifactLine extends Artifact
{
   private static final long serialVersionUID = 5741633846461834438L;
   private final YoFramePoint p1;
   private final YoFramePoint p2;
   private final PlotterGraphics plotterGraphics = new PlotterGraphics();
   private final Color color;

   private static final int pixels = 2;
   private static final BasicStroke stroke = new BasicStroke(pixels);

   public YoArtifactLine(String name, YoFramePoint p1, YoFramePoint p2, Color color)
   {
      super(name);
      this.p1 = p1;
      this.p2 = p2;
      this.color = color;
   }

   public void draw(Graphics graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (isVisible)
      {
         graphics.setColor(color);
         if (stroke != null)
            ((Graphics2D) graphics).setStroke(stroke);

         double x1 = p1.getX();
         double y1 = p1.getY();
         double x2 = p2.getX();
         double y2 = p2.getY();

         plotterGraphics.setCenter(Xcenter, Ycenter);
         plotterGraphics.setScale(scaleFactor);

         plotterGraphics.drawLineSegment(graphics, x1, y1, x2, y2);
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
