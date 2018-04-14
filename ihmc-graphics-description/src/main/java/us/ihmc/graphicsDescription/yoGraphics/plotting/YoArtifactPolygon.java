package us.ihmc.graphicsDescription.yoGraphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.geom.Rectangle2D;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoArtifactPolygon extends YoArtifact
{
   private final YoFrameConvexPolygon2D convexPolygon;

   private final ConvexPolygon2D tempConvexPolygon = new ConvexPolygon2D();

   private final boolean fill;
   private final BasicStroke stroke;

   private final Point2D legendStringPosition = new Point2D();

   private final int lineWidth;
   private final boolean dashedLine;

   public YoArtifactPolygon(String name, YoFrameConvexPolygon2D yoConvexPolygon2d, Color color, boolean fill)
   {
      this(name, yoConvexPolygon2d, color, fill, 2, false);
   }

   public YoArtifactPolygon(String name, YoFrameConvexPolygon2D yoConvexPolygon2d, Color color, boolean fill, int lineWidth)
   {
      this(name, yoConvexPolygon2d, color, fill, lineWidth, false);
   }

   public YoArtifactPolygon(String name, YoFrameConvexPolygon2D yoConvexPolygon2d, Color color, boolean fill, boolean dashedLine)
   {
      this(name, yoConvexPolygon2d, color, fill, 2, dashedLine);
   }

   public YoArtifactPolygon(String name, YoFrameConvexPolygon2D yoConvexPolygon2d, Color color, boolean fill, int lineWidth, boolean dashedLine)
   {
      super(name, new double[] {fill ? 1.0 : 0.0}, color);
      convexPolygon = yoConvexPolygon2d;
      this.fill = fill;
      this.lineWidth = lineWidth;
      this.dashedLine = dashedLine;
      if (!dashedLine)
         stroke = new BasicStroke(lineWidth);
      else
      {
         float[] dashArray = new float[] {10.0f};
         stroke = new BasicStroke(lineWidth, BasicStroke.CAP_SQUARE, BasicStroke.JOIN_MITER, 10.0f, dashArray, 0.0F);
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

      tempConvexPolygon.clear();
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      { // Accessing directly the vertex buffer to avoid internal checks. Due asynchronous read/write, these check can throw exceptions.
         tempConvexPolygon.addVertex(convexPolygon.getVertexBuffer().get(i));
      }
      tempConvexPolygon.update();

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
      vars[i++] = convexPolygon.getYoNumberOfVertices();

      for (YoFramePoint2D p : convexPolygon.getVertexBuffer())
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

   @Override
   public YoArtifact duplicate(YoVariableRegistry newRegistry)
   {
      return new YoArtifactPolygon(getName(), convexPolygon.duplicate(newRegistry), color, fill, lineWidth, dashedLine);
   }
}
