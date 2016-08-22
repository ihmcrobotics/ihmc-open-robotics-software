package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Stroke;
import java.awt.geom.Rectangle2D;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.plotting.Graphics2DAdapter;
import us.ihmc.plotting.PlotterGraphics;
import us.ihmc.plotting.artifact.Artifact;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.RemoteYoGraphic;

public class YoArtifactPolygon extends Artifact implements RemoteYoGraphic
{
   private static final long serialVersionUID = 1595929952165656135L;

   private final YoFrameConvexPolygon2d yoConvexPolygon2d;
   private final ConvexPolygon2d convexPolygon2d = new ConvexPolygon2d();

   private final PlotterGraphics plotterGraphics = new PlotterGraphics();

   // private final Color color;
   private final boolean fill;

   private final int pixels;
   private final BasicStroke stroke;

   public YoArtifactPolygon(String name, YoFrameConvexPolygon2d yoConvexPolygon2d, Color color, boolean fill)
   {
      super(name);
      this.yoConvexPolygon2d = yoConvexPolygon2d;
      this.color = color;
      this.fill = fill;
      this.pixels = 2;
      stroke = createStroke();
   }

   public YoArtifactPolygon(String name, YoFrameConvexPolygon2d yoConvexPolygon2d, Color color, boolean fill, int lineWidth)
   {
      super(name);
      this.yoConvexPolygon2d = yoConvexPolygon2d;
      this.color = color;
      this.fill = fill;
      this.pixels = lineWidth;
      stroke = createStroke();
   }

   public BasicStroke createStroke()
   {
      return new BasicStroke(pixels);
   }

   public void drawLegend(Graphics2DAdapter graphics, int Xcenter, int Ycenter, double scaleFactor)
   {
      graphics.setColor(color);
      String name = "Polygon";
      Rectangle2D textDimensions = graphics.getFontMetrics().getStringBounds(name, graphics.getGraphicsContext());
      int x = Xcenter - (int) (textDimensions.getWidth()/2);
      int y = Ycenter + (int) (textDimensions.getHeight()/2);
      graphics.drawString(name, x, y);
   }

   public void draw(Graphics2DAdapter graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (isVisible)
      {
         graphics.setColor(color);

         Stroke previousStroke = graphics.getStroke();
         graphics.setStroke(stroke);

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
            e.printStackTrace();
            return;
         }

         if (convexPolygon2d.isEmpty())
               return;

         if (fill && convexPolygon2d.getNumberOfVertices() > 2)
         {
            plotterGraphics.fillPolygon(graphics, convexPolygon2d);
         }
         else
         {
            plotterGraphics.drawPolygon(graphics, convexPolygon2d);
         }

         graphics.setStroke(previousStroke);
      }
   }

   public void drawHistory(Graphics2DAdapter graphics2d, int Xcenter, int Ycenter, double scaleFactor)
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
