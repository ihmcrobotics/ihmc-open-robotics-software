package us.ihmc.plotting;

import java.awt.Color;
import java.awt.Composite;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GraphicsConfiguration;
import java.awt.Image;
import java.awt.Paint;
import java.awt.Polygon;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.RenderingHints.Key;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.font.FontRenderContext;
import java.awt.font.GlyphVector;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.awt.image.BufferedImageOp;
import java.awt.image.ImageObserver;
import java.awt.image.RenderedImage;
import java.awt.image.renderable.RenderableImage;
import java.text.AttributedCharacterIterator;
import java.util.Map;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.plotting.frames.MetersReferenceFrame;
import us.ihmc.plotting.frames.PixelsReferenceFrame;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.linearDynamicSystems.BodeUnitsConverter;

/**
 * Everything not deprecated is in meters.
 */
public class Graphics2DAdapter
{
   private final MetersReferenceFrame metersFrame;
   private final PixelsReferenceFrame screenFrame;
   
   private final PlotterPoint2d[] pointBin = new PlotterPoint2d[10];
   private final PlotterVector2d[] vectorBin = new PlotterVector2d[pointBin.length];
   private final int[][] tempPoints = new int[2][pointBin.length];
   
   private Graphics2D graphics2d;
   
   public Graphics2DAdapter(MetersReferenceFrame metersFrame, PixelsReferenceFrame screenFrame)
   {
      this.metersFrame = metersFrame;
      this.screenFrame = screenFrame;
      
      for (int i = 0; i < pointBin.length; i++)
      {
         pointBin[i] = new PlotterPoint2d(metersFrame);
         vectorBin[i] = new PlotterVector2d(metersFrame);
      }
   }
   
   public void setGraphics2d(Graphics2D graphics2d)
   {
      this.graphics2d = graphics2d;
   }
   
   public Graphics getGraphicsContext()
   {
      return graphics2d;
   }

   private int pixelate(double continuous)
   {
      return (int) Math.round(continuous);
   }
   
   @Deprecated
   public void drawEmptyCircle(int x, int y, int radius, Color color)
   {
      setColor(color);
      drawOval((x - (radius / 2)), (y - (radius / 2)), radius, radius);
   }
   
   public void drawEmptyCircle(Point2d center, Vector2d radii)
   {
      drawOval(center, radii);
   }

   @Deprecated
   public void drawFilledCircle(int x, int y, int radius, Color color)
   {
      setColor(color);
      drawOvalFilled((x - (radius / 2)), (y - (radius / 2)), radius, radius);
   }
   
   public void drawFilledCircle(Point2d center, Vector2d radii)
   {
      drawOvalFilled(center, radii);
   }
   
   @Deprecated
   public void drawCross(int x, int y, int radius, Color color)
   {
      setColor(color);
      drawLineSegment((x - (radius / 2)), y, (x + (radius / 2)), y);
      drawLineSegment(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }
   
   public void drawCross(Point2d center, Vector2d radii)
   {
      drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(center.getX(), center.getY() - radii.getY(), center.getX(), center.getY() + radii.getY());
   }

   @Deprecated
   public void drawCircleWithCross(int x, int y, int radius, Color color)
   {
      setColor(color);
      drawOval((x - (radius / 2)), (y - (radius / 2)), radius, radius);
      drawLineSegment((x - (radius / 2)), y, (x + (radius / 2)), y);
      drawLineSegment(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }
   
   public void drawCircleWithCross(Point2d center, Vector2d radii)
   {
      drawOval(center, radii);
      drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(center.getX(), center.getY() - radii.getY(), center.getX(), center.getY() + radii.getY());
   }

   @Deprecated
   public void drawRotatedCross(int x, int y, int radius, Color color)
   {
      setColor(color);
      int distance = (int)Math.round(0.707 * radius / 2);
      drawLineSegment((x - distance), (y - distance), (x + distance), (y + distance));
      drawLineSegment((x - distance), (y + distance), (x + distance), (y - distance));
   }

   public void drawRotatedCross(Point2d center, Vector2d radii)
   {
      double distanceX = 0.707 * radii.getX();
      double distanceY = 0.707 * radii.getY();
      drawLineSegment(center.getX() - distanceX, center.getY() - distanceY, center.getX() + distanceX, center.getY() + distanceY);
      drawLineSegment(center.getX() - distanceX, center.getY() + distanceY, center.getX() + distanceX, center.getY() - distanceY);
   }

   @Deprecated
   public void drawCircleWithRotatedCross(int x, int y, int radius, Color color)
   {
      setColor(color);
      int distance = (int)Math.round(0.707 * radius / 2);
      drawOval((x - radius / 2), (y - radius / 2), radius, radius);
      drawLineSegment((x - distance), (y - distance), (x + distance), (y + distance));
      drawLineSegment((x - distance), (y + distance), (x + distance), (y - distance));
   }
   
   public void drawCircleWithRotatedCross(Point2d center, Vector2d radii)
   {
      double distanceX = 0.707 * radii.getX();
      double distanceY = 0.707 * radii.getY();
      drawOval(center, radii);
      drawLineSegment(center.getX() - distanceX, center.getY() - distanceY, center.getX() + distanceX, center.getY() + distanceY);
      drawLineSegment(center.getX() - distanceX, center.getY() + distanceY, center.getX() + distanceX, center.getY() - distanceY);
   }

   @Deprecated
   public void drawDiamond(int x, int y, int radius, Color color)
   {
      setColor(color);
      drawLineSegment((x - (radius / 2)), y, x, (y + radius / 2));
      drawLineSegment((x - (radius / 2)), y, x, (y - radius / 2));
      drawLineSegment(x, (y + radius / 2), x + radius / 2, y);
      drawLineSegment(x, (y - radius / 2), x + radius / 2, y);
   }
   
   public void drawDiamond(Point2d center, Vector2d radii)
   {
      drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX(), center.getY() + radii.getY());
      drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX(), center.getY() - radii.getY());
      drawLineSegment(center.getX(), center.getY() + radii.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(center.getX(), center.getY() - radii.getY(), center.getX() + radii.getX(), center.getY());
   }

   @Deprecated
   public void drawDiamondWithCross(int x, int y, int radius, Color color)
   {
      setColor(color);
      drawLineSegment((x - (radius / 2)), y, x, (y + radius / 2));
      drawLineSegment((x - (radius / 2)), y, x, (y - radius / 2));
      drawLineSegment(x, (y + radius / 2), x + radius / 2, y);
      drawLineSegment(x, (y - radius / 2), x + radius / 2, y);
      drawLineSegment((x - (radius / 2)), y, (x + (radius / 2)), y);
      drawLineSegment(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }
   
   public void drawDiamondWithCross(Point2d center, Vector2d radii)
   {
      drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX(), center.getY() + radii.getY());
      drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX(), center.getY() - radii.getY());
      drawLineSegment(center.getX(), center.getY() + radii.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(center.getX(), center.getY() - radii.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(center.getX(), center.getY() - radii.getY(), center.getX(), center.getY() + radii.getY());
   }

   @Deprecated
   public void drawSquare(int x, int y, int radius, Color color)
   {
      setColor(color);
      drawLineSegment((x - (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y - (radius / 2)));
      drawLineSegment((x + (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
      drawLineSegment((x - (radius / 2)), (y - (radius / 2)), (x - (radius / 2)), (y + (radius / 2)));
      drawLineSegment((x - (radius / 2)), (y + (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
   }
   
   public void drawSquare(Point2d center, Vector2d radii)
   {
      drawLineSegment(center.getX() - radii.getX(), center.getY() - radii.getY(), center.getX() + radii.getX(), center.getY() - radii.getY());
      drawLineSegment(center.getX() + radii.getX(), center.getY() - radii.getY(), center.getX() + radii.getX(), center.getY() + radii.getY());
      drawLineSegment(center.getX() - radii.getX(), center.getY() - radii.getY(), center.getX() - radii.getX(), center.getY() + radii.getY());
      drawLineSegment(center.getX() - radii.getX(), center.getY() + radii.getY(), center.getX() + radii.getX(), center.getY() + radii.getY());
   }
   
   public void drawSquareFilled(Point2d center, Vector2d radii)
   {
      PlotterPoint2d centerFramePoint = pointBin[0];
      PlotterVector2d radiiFrameVector = vectorBin[0];
      centerFramePoint.setIncludingFrame(metersFrame, center);
      radiiFrameVector.setIncludingFrame(metersFrame, radii);
      centerFramePoint.add(-radiiFrameVector.getX(), radiiFrameVector.getY());
      centerFramePoint.changeFrame(screenFrame);
      radiiFrameVector.changeFrame(screenFrame);
      drawRectangleFilled(pixelate(centerFramePoint.getX()), pixelate(centerFramePoint.getY()), pixelate(2.0 * radiiFrameVector.getX()), pixelate(2.0 * radiiFrameVector.getY()));
   }

   @Deprecated
   public void drawSquareWithCross(int x, int y, int radius, Color color)
   {
      setColor(color);
      drawLineSegment((x - (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y - (radius / 2)));
      drawLineSegment((x + (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
      drawLineSegment((x - (radius / 2)), (y - (radius / 2)), (x - (radius / 2)), (y + (radius / 2)));
      drawLineSegment((x - (radius / 2)), (y + (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
      drawLineSegment((x - (radius / 2)), y, (x + (radius / 2)), y);
      drawLineSegment(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }
   
   public void drawSquareWithCross(Point2d center, Vector2d radii)
   {
      drawSquare(center, radii);
      drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(center.getX(), center.getY() - radii.getY(), center.getX(), center.getY() + radii.getY());
   }

   @Deprecated
   public void drawLineSegment(int x1, int y1, int x2, int y2)
   {
      graphics2d.drawLine(x1, y1, x2, y2);
   }
   
   public void drawLineSegment(LineSegment2d lineSegment)
   {
      PlotterPoint2d firstEndpoint = pointBin[0];
      PlotterPoint2d secondEndpoint = pointBin[1];
      firstEndpoint.setIncludingFrame(metersFrame, lineSegment.getFirstEndpoint());
      secondEndpoint.setIncludingFrame(metersFrame, lineSegment.getSecondEndpoint());
      firstEndpoint.changeFrame(screenFrame);
      secondEndpoint.changeFrame(screenFrame);
      graphics2d.drawLine(pixelate(firstEndpoint.getX()), pixelate(firstEndpoint.getY()), pixelate(secondEndpoint.getX()), pixelate(secondEndpoint.getY()));
   }
   
   public void drawLineSegment(double firstPointX, double firstPointY, double secondPointX, double secondPointY)
   {
      PlotterPoint2d firstEndpoint = pointBin[0];
      PlotterPoint2d secondEndpoint = pointBin[1];
      firstEndpoint.setIncludingFrame(metersFrame, firstPointX, firstPointY);
      secondEndpoint.setIncludingFrame(metersFrame, secondPointX, secondPointY);
      firstEndpoint.changeFrame(screenFrame);
      secondEndpoint.changeFrame(screenFrame);
      graphics2d.drawLine(pixelate(firstEndpoint.getX()), pixelate(firstEndpoint.getY()), pixelate(secondEndpoint.getX()), pixelate(secondEndpoint.getY()));
   }
   
   public void drawPoint(Point2d point)
   {
      PlotterPoint2d plotterPoint = pointBin[0];
      plotterPoint.setIncludingFrame(metersFrame, point);
      plotterPoint.changeFrame(screenFrame);
      graphics2d.drawLine(pixelate(plotterPoint.getX()), pixelate(plotterPoint.getY()), pixelate(plotterPoint.getX()), pixelate(plotterPoint.getY()));
   }
   
   public void drawLine(Line2d line)
   {
      PlotterPoint2d start = pointBin[0];
      PlotterVector2d direction = vectorBin[0];
      start.setIncludingFrame(metersFrame, line.getPoint());
      direction.setIncludingFrame(metersFrame, line.getNormalizedVector());
      PlotterPoint2d farPointPositive = pointBin[1];
      PlotterPoint2d farPointNegative = pointBin[2];
      PlotterVector2d far = vectorBin[1];
      far.setIncludingFrame(direction);
      far.scale(2000.0);
      farPointPositive.setIncludingFrame(start);
      farPointNegative.setIncludingFrame(start);
      farPointPositive.add(far);
      farPointNegative.sub(far);
      farPointPositive.changeFrame(screenFrame);
      farPointNegative.changeFrame(screenFrame);
      graphics2d.drawLine(pixelate(farPointNegative.getX()), pixelate(farPointNegative.getY()), pixelate(farPointPositive.getX()), pixelate(farPointPositive.getY()));
   }

   @Deprecated
   public void drawPolyline(int[] xPoints, int[] yPoints, int nPoints)
   {
      graphics2d.drawPolyline(xPoints, yPoints, nPoints);
   }

   @Deprecated
   public void drawOval(int x, int y, int width, int height)
   {
      graphics2d.drawOval(x, y, width, height);
   }
   
   public void drawOval(Point2d center, Vector2d radii)
   {
      PlotterPoint2d centerFramePoint = pointBin[0];
      PlotterVector2d radiiFrameVector = vectorBin[0];
      centerFramePoint.setIncludingFrame(metersFrame, center);
      radiiFrameVector.setIncludingFrame(metersFrame, radii);
      centerFramePoint.add(-radiiFrameVector.getX(), radiiFrameVector.getY());
      centerFramePoint.changeFrame(screenFrame);
      radiiFrameVector.changeFrame(screenFrame);
      graphics2d.drawOval(pixelate(centerFramePoint.getX()), pixelate(centerFramePoint.getY()), pixelate(2.0 * radiiFrameVector.getX()), -pixelate(2.0 * radiiFrameVector.getY()));
   }
   
   public void drawOvalFilled(Point2d center, Vector2d radii)
   {
      PlotterPoint2d centerFramePoint = pointBin[0];
      PlotterVector2d radiiFrameVector = vectorBin[0];
      centerFramePoint.setIncludingFrame(metersFrame, center);
      radiiFrameVector.setIncludingFrame(metersFrame, radii);
      centerFramePoint.add(-radiiFrameVector.getX(), radiiFrameVector.getY());
      centerFramePoint.changeFrame(screenFrame);
      radiiFrameVector.changeFrame(screenFrame);
      graphics2d.fillOval(pixelate(centerFramePoint.getX()), pixelate(centerFramePoint.getY()), pixelate(2.0 * radiiFrameVector.getX()), -pixelate(2.0 * radiiFrameVector.getY()));
   }
   
   @Deprecated
   public void drawOvalFilled(int x, int y, int width, int height)
   {
      graphics2d.fillOval(x, y, width, height);
   }

   @Deprecated
   public void drawRectangle(int x, int y, int width, int height)
   {
      graphics2d.drawRect(x, y, width, height);
   }

   @Deprecated
   public void drawRectangleFilled(int x, int y, int width, int height)
   {
      graphics2d.fillRect(x, y, width, height);
   }

   @Deprecated
   public void drawRectangle3D(int x, int y, int width, int height, boolean raised)
   {
      graphics2d.draw3DRect(x, y, width, height, raised);
   }

   @Deprecated
   public void drawRectangle3DFilled(int x, int y, int width, int height, boolean raised)
   {
      graphics2d.fill3DRect(x, y, width, height, raised);
   }

   @Deprecated
   public void drawRectangleRounded(int x, int y, int width, int height, int arcWidth, int arcHeight)
   {
      graphics2d.drawRoundRect(x, y, width, height, arcWidth, arcHeight);
   }

   @Deprecated
   public void drawRectangleRoundedFilled(int x, int y, int width, int height, int arcWidth, int arcHeight)
   {
      graphics2d.fillRoundRect(x, y, width, height, arcWidth, arcHeight);
   }

   @Deprecated
   public void drawPolygonFilled(int[] xPoints, int[] yPoints, int nPoints)
   {
      graphics2d.fillPolygon(xPoints, yPoints, nPoints);
   }

   @Deprecated
   public void drawPolygon(int[] xPoints, int[] yPoints, int nPoints)
   {
      graphics2d.drawPolygon(xPoints, yPoints, nPoints);
   }

   @Deprecated
   public void drawPolygonFilled(Polygon p)
   {
      graphics2d.fillPolygon(p);
   }
   
   public void drawPolygonFilled(ConvexPolygon2d convexPolygon2d)
   {
      setupForDrawPolygon(convexPolygon2d);
      
      graphics2d.fillPolygon(tempPoints[0], tempPoints[1], convexPolygon2d.getNumberOfVertices());
   }
   
   public void drawPolygon(ConvexPolygon2d convexPolygon2d)
   {
      setupForDrawPolygon(convexPolygon2d);
      
      graphics2d.drawPolygon(tempPoints[0], tempPoints[1], convexPolygon2d.getNumberOfVertices());
   }

   private void setupForDrawPolygon(ConvexPolygon2d convexPolygon2d)
   {
      for (int i = 0; i < convexPolygon2d.getNumberOfVertices(); i++)
      {
         pointBin[i].setIncludingFrame(metersFrame, convexPolygon2d.getVertex(i));
         pointBin[i].changeFrame(screenFrame);
         tempPoints[0][i] = pixelate(pointBin[i].getX());
         tempPoints[1][i] = pixelate(pointBin[i].getY());
      }
   }

   @Deprecated
   public void drawPolygon(Polygon p)
   {
      graphics2d.drawPolygon(p);
   }

   @Deprecated
   public void drawArc(int x, int y, int width, int height, int startAngle, int arcAngle)
   {
      graphics2d.drawArc(x, y, width, height, startAngle, arcAngle);
   }
   
   public void drawArc(Point2d center, Vector2d radii, double startAngle, double arcAngle)
   {
      PlotterPoint2d plotCenter = pointBin[0];
      PlotterVector2d plotRadii = vectorBin[0];
      plotCenter.setIncludingFrame(metersFrame, center);
      plotRadii.setIncludingFrame(metersFrame, radii);
      plotCenter.changeFrame(screenFrame);
      plotRadii.changeFrame(screenFrame);
      graphics2d.drawArc(pixelate(plotCenter.getX()),
                         pixelate(plotCenter.getY()),
                         pixelate(2.0 * plotRadii.getX()),
                         pixelate(2.0 * plotRadii.getY()),
                         pixelate(BodeUnitsConverter.convertRadianToDegrees(startAngle)),
                         pixelate(BodeUnitsConverter.convertRadianToDegrees(arcAngle)));
   }

   @Deprecated
   public void drawArcFilled(int x, int y, int width, int height, int startAngle, int arcAngle)
   {
      graphics2d.fillArc(x, y, width, height, startAngle, arcAngle);
   }

   @Deprecated
   public void drawString(String str, int x, int y)
   {
      graphics2d.drawString(str, x, y);
   }

   public void drawString(Point2d startPoint, String string)
   {
      PlotterPoint2d plotStart = pointBin[0];
      plotStart.setIncludingFrame(metersFrame, startPoint);
      plotStart.changeFrame(screenFrame);
      graphics2d.drawString(string, pixelate(plotStart.getX()), pixelate(plotStart.getY()));
   }
   
   @Deprecated
   public void drawString(String str, float x, float y)
   {
      graphics2d.drawString(str, x, y);
   }

   @Deprecated
   public void drawString(AttributedCharacterIterator iterator, int x, int y)
   {
      graphics2d.drawString(iterator, x, y);
   }

   @Deprecated
   public void drawString(AttributedCharacterIterator iterator, float x, float y)
   {
      graphics2d.drawString(iterator, x, y);
   }

   @Deprecated
   public void drawGlyphVector(GlyphVector g, float x, float y)
   {
      graphics2d.drawGlyphVector(g, x, y);
   }

   @Deprecated
   public void drawBytes(byte[] data, int offset, int length, int x, int y)
   {
      graphics2d.drawBytes(data, offset, length, x, y);
   }

   @Deprecated
   public void drawChars(char[] data, int offset, int length, int x, int y)
   {
      graphics2d.drawChars(data, offset, length, x, y);
   }

   @Deprecated
   public void drawRenderedImage(RenderedImage img, AffineTransform xform)
   {
      graphics2d.drawRenderedImage(img, xform);
   }

   @Deprecated
   public void drawRenderableImage(RenderableImage img, AffineTransform xform)
   {
      graphics2d.drawRenderableImage(img, xform);
   }

   @Deprecated
   public boolean drawImage(Image img, AffineTransform xform, ImageObserver obs)
   {
      return graphics2d.drawImage(img, xform, obs);
   }

   @Deprecated
   public void drawImage(BufferedImage img, BufferedImageOp op, int x, int y)
   {
      graphics2d.drawImage(img, op, x, y);
   }

   @Deprecated
   public boolean drawImage(Image img, int x, int y, ImageObserver observer)
   {
      return graphics2d.drawImage(img, x, y, observer);
   }

   @Deprecated
   public boolean drawImage(Image img, int x, int y, Color bgcolor, ImageObserver observer)
   {
      return graphics2d.drawImage(img, x, y, bgcolor, observer);
   }

   @Deprecated
   public boolean drawImage(Image img, int x, int y, int width, int height, ImageObserver observer)
   {
      return graphics2d.drawImage(img, x, y, width, height, observer);
   }

   @Deprecated
   public boolean drawImage(Image img, int x, int y, int width, int height, Color bgcolor, ImageObserver observer)
   {
      return graphics2d.drawImage(img, x, y, width, height, bgcolor, observer);
   }

   @Deprecated
   public boolean drawImage(Image img, int dx1, int dy1, int dx2, int dy2, int sx1, int sy1, int sx2, int sy2, ImageObserver observer)
   {
      return graphics2d.drawImage(img, dx1, dy1, dx2, dy2, sx1, sy1, sx2, sy2, observer);
   }

   @Deprecated
   public boolean drawImage(Image img, int dx1, int dy1, int dx2, int dy2, int sx1, int sy1, int sx2, int sy2, Color bgcolor, ImageObserver observer)
   {
      return graphics2d.drawImage(img, dx1, dy1, dx2, dy2, sx1, sy1, sx2, sy2, bgcolor, observer);
   }

   public Graphics create()
   {
      return graphics2d.create();
   }

   @Deprecated
   public Graphics create(int x, int y, int width, int height)
   {
      return graphics2d.create(x, y, width, height);
   }

   @Deprecated
   public void clearRectangle(int x, int y, int width, int height)
   {
      graphics2d.clearRect(x, y, width, height);
   }

   @Deprecated
   public void clipRectangle(int x, int y, int width, int height)
   {
      graphics2d.clipRect(x, y, width, height);
   }

   @Deprecated
   public void setClip(int x, int y, int width, int height)
   {
      graphics2d.setClip(x, y, width, height);
   }

   @Deprecated
   public void copyArea(int x, int y, int width, int height, int dx, int dy)
   {
      graphics2d.copyArea(x, y, width, height, dx, dy);
   }

   @Deprecated
   public boolean hitClip(int x, int y, int width, int height)
   {
      return graphics2d.hitClip(x, y, width, height);
   }

   @Deprecated
   public void setClip(Shape clip)
   {
      graphics2d.setClip(clip);
   }

   @Deprecated
   public boolean hit(Rectangle rect, Shape s, boolean onStroke)
   {
      return graphics2d.hit(rect, s, onStroke);
   }

   @Deprecated
   public Rectangle getClipBounds(Rectangle r)
   {
      return graphics2d.getClipBounds(r);
   }

   @Deprecated
   public void clip(Shape s)
   {
      graphics2d.clip(s);
   }

   @Deprecated
   public void setTransform(AffineTransform Tx)
   {
      graphics2d.setTransform(Tx);
   }

   @Deprecated
   public AffineTransform getTransform()
   {
      return graphics2d.getTransform();
   }

   @Deprecated
   public Shape getClip()
   {
      return graphics2d.getClip();
   }

   @Deprecated
   public Rectangle getClipBounds()
   {
      return graphics2d.getClipBounds();
   }

   public Color getColor()
   {
      return graphics2d.getColor();
   }

   public void setColor(Color c)
   {
      graphics2d.setColor(c);
   }

   public void setPaintMode()
   {
      graphics2d.setPaintMode();
   }

   public void setXORMode(Color c1)
   {
      graphics2d.setXORMode(c1);
   }

   public Font getFont()
   {
      return graphics2d.getFont();
   }

   public void setFont(Font font)
   {
      graphics2d.setFont(font);
   }

   public FontMetrics getFontMetrics()
   {
      return graphics2d.getFontMetrics();
   }

   public FontMetrics getFontMetrics(Font f)
   {
      return graphics2d.getFontMetrics(f);
   }

   public GraphicsConfiguration getDeviceConfiguration()
   {
      return graphics2d.getDeviceConfiguration();
   }

   public void setComposite(Composite comp)
   {
      graphics2d.setComposite(comp);
   }

   public void setPaint(Paint paint)
   {
      graphics2d.setPaint(paint);
   }

   public void setStroke(Stroke s)
   {
      graphics2d.setStroke(s);
   }

   public void setRenderingHint(Key hintKey, Object hintValue)
   {
      graphics2d.setRenderingHint(hintKey, hintValue);
   }

   public Object getRenderingHint(Key hintKey)
   {
      return graphics2d.getRenderingHint(hintKey);
   }

   public void setRenderingHints(Map<?, ?> hints)
   {
      graphics2d.setRenderingHints(hints);
   }

   public void addRenderingHints(Map<?, ?> hints)
   {
      graphics2d.addRenderingHints(hints);
   }

   public RenderingHints getRenderingHints()
   {
      return graphics2d.getRenderingHints();
   }

   public Paint getPaint()
   {
      return graphics2d.getPaint();
   }

   public Composite getComposite()
   {
      return graphics2d.getComposite();
   }

   public void setBackground(Color color)
   {
      graphics2d.setBackground(color);
   }

   public Color getBackground()
   {
      return graphics2d.getBackground();
   }

   public Stroke getStroke()
   {
      return graphics2d.getStroke();
   }

   public FontRenderContext getFontRenderContext()
   {
      return graphics2d.getFontRenderContext();
   }

   public void dispose()
   {
      graphics2d.dispose();
   }
}
