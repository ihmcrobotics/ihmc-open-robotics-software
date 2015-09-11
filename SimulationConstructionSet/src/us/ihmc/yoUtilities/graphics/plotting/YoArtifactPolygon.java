package us.ihmc.yoUtilities.graphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.PlotterGraphics;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.graphics.RemoteYoGraphic;

public class YoArtifactPolygon extends Artifact implements RemoteYoGraphic
{
   private static final long serialVersionUID = 1595929952165656135L;

   private final YoFrameConvexPolygon2d yoConvexPolygon2d;
   private final ConvexPolygon2d convexPolygon2d = new ConvexPolygon2d();

   private final PlotterGraphics plotterGraphics = new PlotterGraphics();

   // private final Color color;
   private final boolean fill;

   private static final int pixels = 2;
   private static final BasicStroke stroke = new BasicStroke(pixels);

   public YoArtifactPolygon(String name, YoFrameConvexPolygon2d yoConvexPolygon2d, Color color, boolean fill)
   {
      super(name);
      this.yoConvexPolygon2d = yoConvexPolygon2d;
      this.color = color;
      this.fill = fill;
   }

   public void drawLegend(Graphics graphics, int Xcenter, int Ycenter, double scaleFactor)
   {
      graphics.setColor(color);
      graphics.drawString("Polygon", Xcenter, Ycenter);
   }

   public void draw(Graphics graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (yoConvexPolygon2d.getNumberOfVertices() < 3)
         return;

      graphics.setColor(color);
      if (stroke != null)
         ((Graphics2D) graphics).setStroke(stroke);

      plotterGraphics.setCenter(Xcenter, Ycenter);
      plotterGraphics.setScale(scaleFactor);

      try
      {
         FrameConvexPolygon2d frameConvexPolygon2d = yoConvexPolygon2d.getFrameConvexPolygon2d();
         ConvexPolygon2d convexPolygon2dFromYoConvexPolygon = frameConvexPolygon2d.getConvexPolygon2d();
         convexPolygon2d.setAndUpdate(convexPolygon2dFromYoConvexPolygon);
      }
      catch (Exception e)
      {
         System.err.println("In YoArtifactPolygon.java: " + e.getClass().getSimpleName() + " while calling draw().");
         return;
      }
      
      if (convexPolygon2d.isEmpty())
            return;

      if (fill)
      {
         plotterGraphics.fillPolygon(graphics, convexPolygon2d);
      }
      else
      {
         plotterGraphics.drawPolygon(graphics, convexPolygon2d);
      }
   }

   public void drawHistory(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }

   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }

   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.POLYGON_ARTIFACT;
   }

   public YoVariable<?>[] getVariables()
   {
      YoVariable<?>[] vars = new YoVariable[1 + 2 * yoConvexPolygon2d.getMaxNumberOfVertices()];
      int i = 0;
      vars[i++] = yoConvexPolygon2d.getYoNumberVertices();

      for (YoFramePoint2d p : yoConvexPolygon2d.getYoFramePoints())
      {
         vars[i++] = p.getYoX();
         vars[i++] = p.getYoY();
      }

      return vars;
   }

   public double[] getConstants()
   {
      return new double[] { fill ? 1 : 0 };
   }

   public AppearanceDefinition getAppearance()
   {
      return new YoAppearanceRGBColor(color, 0.0);
   }

   public String getName()
   {
      return getID();
   }
}
