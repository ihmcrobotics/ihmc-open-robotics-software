package us.ihmc.graphicsDescription.yoGraphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.geom.Rectangle2D;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;

public class YoArtifactPolygon extends YoArtifact
{
   private final YoFrameConvexPolygon2d convexPolygon;
   
   private final ConvexPolygon2D tempConvexPolygon = new ConvexPolygon2D();

   private final boolean fill;
   private final BasicStroke stroke;
   
   private final Point2D legendStringPosition = new Point2D();
   
   public YoArtifactPolygon(String name, YoFrameConvexPolygon2d yoConvexPolygon2d, Color color, boolean fill)
   {
      this(name, yoConvexPolygon2d, color, fill, 2, false);
   }

   public YoArtifactPolygon(String name, YoFrameConvexPolygon2d yoConvexPolygon2d, Color color, boolean fill, int lineWidth)
   {
      this(name, yoConvexPolygon2d, color, fill, lineWidth, false);
   }

   public YoArtifactPolygon(String name, YoFrameConvexPolygon2d yoConvexPolygon2d, Color color, boolean fill, boolean dashedLine)
   {
      this(name, yoConvexPolygon2d, color, fill, 2, dashedLine);
   }

   public YoArtifactPolygon(String name, YoFrameConvexPolygon2d yoConvexPolygon2d, Color color, boolean fill, int lineWidth, boolean dashedLine)
   {
      super(name,  new double[] {fill ? 1.0 : 0.0}, color);
      this.convexPolygon = yoConvexPolygon2d;
      this.fill = fill;
      if (!dashedLine)
         this.stroke = new BasicStroke(lineWidth);
      else
      {
         float[] dashArray = new float[]{10.0f};
         this.stroke = new BasicStroke(lineWidth, BasicStroke.CAP_SQUARE, BasicStroke.JOIN_MITER, 10.0f, dashArray, 0.0F);
      }
   }

   @Override
   public void drawLegend(Plotter2DAdapter graphics, Point2D origin)
   {
      graphics.setColor(color);
      String name = "Polygon";
      Rectangle2D textDimensions = graphics.getFontMetrics().getStringBounds(name, graphics.getGraphicsContext());
      legendStringPosition.set(origin.getX() - textDimensions.getWidth() / 2.0, origin.getY() + textDimensions.getHeight() / 2.0);
      graphics.drawString(graphics.getScreenFrame(), name, legendStringPosition);
   }

   @Override
   public void draw(Graphics2DAdapter graphics)
   {
      graphics.setColor(color);
      graphics.setStroke(stroke);

      convexPolygon.getFrameConvexPolygon2d().get(tempConvexPolygon);

      if (fill)
      {
         graphics.drawPolygonFilled(tempConvexPolygon);
      }
      else
      {
         graphics.drawPolygon(tempConvexPolygon);
      }
   }

   @Override
   public void drawHistoryEntry(Graphics2DAdapter graphics, double[] entry)
   {
      // not implemented
   }

   @Override
   public YoVariable<?>[] getVariables()
   {
      YoVariable<?>[] vars = new YoVariable[1 + 2 * convexPolygon.getMaxNumberOfVertices()];
      int i = 0;
      vars[i++] = convexPolygon.getYoNumberVertices();

      for (YoFramePoint2d p : convexPolygon.getYoFramePoints())
      {
         vars[i++] = p.getYoX();
         vars[i++] = p.getYoY();
      }

      return vars;
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.POLYGON_ARTIFACT;
   }
}
