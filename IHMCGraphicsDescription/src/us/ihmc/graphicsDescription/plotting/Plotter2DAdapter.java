package us.ihmc.graphicsDescription.plotting;

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
import java.awt.RenderingHints;
import java.awt.RenderingHints.Key;
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

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.plotting.frames.MetersReferenceFrame;
import us.ihmc.graphicsDescription.plotting.frames.PixelsReferenceFrame;
import us.ihmc.graphicsDescription.plotting.frames.PlotterReferenceFrame;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.linearDynamicSystems.BodeUnitsConverter;
import us.ihmc.tools.io.printing.PrintTools;

@SuppressWarnings("unused") // it's a wrapper, unused is fine
/**
 * This is a class internal to Plotter which uses PlotterReferenceFrames.
 */
public class Plotter2DAdapter
{
   private final MetersReferenceFrame metersFrame;
   private final PixelsReferenceFrame pixelsFrame;
   private final PixelsReferenceFrame screenFrame;

   private final PlotterPoint2d[] pointBin = new PlotterPoint2d[50];
   private final PlotterVector2d[] vectorBin = new PlotterVector2d[pointBin.length];
   private final int[][] tempPoints = new int[2][pointBin.length];

   private Graphics2D graphics2d;

   public Plotter2DAdapter(MetersReferenceFrame metersFrame, PixelsReferenceFrame screenFrame, PixelsReferenceFrame pixelsFrame)
   {
      this.metersFrame = metersFrame;
      this.screenFrame = screenFrame;
      this.pixelsFrame = pixelsFrame;

      for (int i = 0; i < pointBin.length; i++)
      {
         pointBin[i] = new PlotterPoint2d(metersFrame);
         vectorBin[i] = new PlotterVector2d(metersFrame);
      }
   }

   public void setGraphics2d(Graphics2D graphics2d)
   {
      this.graphics2d = graphics2d;
      this.graphics2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
   }

   public Graphics getGraphicsContext()
   {
      return graphics2d;
   }

   private int pixelate(double continuous)
   {
      return (int) Math.round(continuous);
   }

   // for graphics 2d adapter use

   public void drawCross(PlotterReferenceFrame frame, Point2D center, Vector2D radii)
   {
      drawLineSegment(frame, center.getX() - radii.getX(), center.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(frame, center.getX(), center.getY() - radii.getY(), center.getX(), center.getY() + radii.getY());
   }

   public void drawCircleWithCross(PlotterReferenceFrame frame, Point2D center, Vector2D radii)
   {
      drawOval(frame, center, radii);
      drawLineSegment(frame, center.getX() - radii.getX(), center.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(frame, center.getX(), center.getY() - radii.getY(), center.getX(), center.getY() + radii.getY());
   }

   public void drawRotatedCross(PlotterReferenceFrame frame, Point2D center, Vector2D radii)
   {
      double distanceX = 0.707 * radii.getX();
      double distanceY = 0.707 * radii.getY();
      drawLineSegment(frame, center.getX() - distanceX, center.getY() - distanceY, center.getX() + distanceX, center.getY() + distanceY);
      drawLineSegment(frame, center.getX() - distanceX, center.getY() + distanceY, center.getX() + distanceX, center.getY() - distanceY);
   }

   public void drawCircleWithRotatedCross(PlotterReferenceFrame frame, Point2D center, Vector2D radii)
   {
      double distanceX = 0.707 * radii.getX();
      double distanceY = 0.707 * radii.getY();
      drawOval(frame, center, radii);
      drawLineSegment(frame, center.getX() - distanceX, center.getY() - distanceY, center.getX() + distanceX, center.getY() + distanceY);
      drawLineSegment(frame, center.getX() - distanceX, center.getY() + distanceY, center.getX() + distanceX, center.getY() - distanceY);
   }

   public void drawDiamond(PlotterReferenceFrame frame, Point2D center, Vector2D radii)
   {
      drawLineSegment(frame, center.getX() - radii.getX(), center.getY(), center.getX(), center.getY() + radii.getY());
      drawLineSegment(frame, center.getX() - radii.getX(), center.getY(), center.getX(), center.getY() - radii.getY());
      drawLineSegment(frame, center.getX(), center.getY() + radii.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(frame, center.getX(), center.getY() - radii.getY(), center.getX() + radii.getX(), center.getY());
   }

   public void drawDiamondWithCross(PlotterReferenceFrame frame, Point2D center, Vector2D radii)
   {
      drawLineSegment(frame, center.getX() - radii.getX(), center.getY(), center.getX(), center.getY() + radii.getY());
      drawLineSegment(frame, center.getX() - radii.getX(), center.getY(), center.getX(), center.getY() - radii.getY());
      drawLineSegment(frame, center.getX(), center.getY() + radii.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(frame, center.getX(), center.getY() - radii.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(frame, center.getX() - radii.getX(), center.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(frame, center.getX(), center.getY() - radii.getY(), center.getX(), center.getY() + radii.getY());
   }

   public void drawSquareWithCross(PlotterReferenceFrame frame, Point2D center, Vector2D radii)
   {
      drawRectangle(frame, center, radii);
      drawLineSegment(frame, center.getX() - radii.getX(), center.getY(), center.getX() + radii.getX(), center.getY());
      drawLineSegment(frame, center.getX(), center.getY() - radii.getY(), center.getX(), center.getY() + radii.getY());
   }

   public void drawRectangle(PlotterReferenceFrame frame, Point2D center, Vector2D radii)
   {
      PlotterPoint2d centerFramePoint = pointBin[0];
      PlotterVector2d radiiFrameVector = vectorBin[0];
      centerFramePoint.setIncludingFrame(frame, center);
      radiiFrameVector.setIncludingFrame(frame, radii.getX(), radii.getY());
      centerFramePoint.changeFrame(screenFrame);
      radiiFrameVector.changeFrame(screenFrame);
      radiiFrameVector.set(Math.abs(radiiFrameVector.getX()), Math.abs(radiiFrameVector.getY()));
      centerFramePoint.sub(radiiFrameVector);
      drawRectangle(pixelate(centerFramePoint.getX()), pixelate(centerFramePoint.getY()), pixelate(2.0 * radiiFrameVector.getX()), pixelate(2.0 * radiiFrameVector.getY()));
   }

   public void drawRectangle(PlotterReferenceFrame frame, double x, double y, double width, double height)
   {
      PlotterPoint2d position = pointBin[0];
      PlotterVector2d dimensions = vectorBin[0];
      position.setIncludingFrame(frame, x, y);
      dimensions.setIncludingFrame(frame, Math.abs(width), Math.abs(height));
      position.changeFrame(screenFrame);
      dimensions.changeFrame(screenFrame);
      drawRectangle(pixelate(position.getX()), pixelate(position.getY()), pixelate(dimensions.getX()), pixelate(dimensions.getY()));
   }

   public void drawSquareFilled(PlotterReferenceFrame frame, Point2D center, Vector2D radii)
   {
      PlotterPoint2d centerFramePoint = pointBin[0];
      PlotterVector2d radiiFrameVector = vectorBin[0];
      centerFramePoint.setIncludingFrame(frame, center);
      radiiFrameVector.setIncludingFrame(frame, radii.getX(), radii.getY());
      centerFramePoint.changeFrame(screenFrame);
      radiiFrameVector.changeFrame(screenFrame);
      radiiFrameVector.set(Math.abs(radiiFrameVector.getX()), Math.abs(radiiFrameVector.getY()));
      centerFramePoint.sub(radiiFrameVector);
      drawRectangleFilled(pixelate(centerFramePoint.getX()), pixelate(centerFramePoint.getY()), pixelate(2.0 * radiiFrameVector.getX()), pixelate(2.0 * radiiFrameVector.getY()));
   }

   public void drawOval(PlotterReferenceFrame frame, Point2D center, Vector2D radii)
   {
      PlotterPoint2d centerFramePoint = pointBin[0];
      PlotterVector2d radiiFrameVector = vectorBin[0];
      centerFramePoint.setIncludingFrame(frame, center);
      radiiFrameVector.setIncludingFrame(frame, radii.getX(), radii.getY());
      centerFramePoint.changeFrame(screenFrame);
      radiiFrameVector.changeFrame(screenFrame);
      radiiFrameVector.set(Math.abs(radiiFrameVector.getX()), Math.abs(radiiFrameVector.getY()));
      centerFramePoint.sub(radiiFrameVector);
      drawOval(pixelate(centerFramePoint.getX()), pixelate(centerFramePoint.getY()), pixelate(2.0 * radiiFrameVector.getX()), pixelate(2.0 * radiiFrameVector.getY()));
   }

   public void drawOvalFilled(PlotterReferenceFrame frame, Point2D center, Vector2D radii)
   {
      PlotterPoint2d centerFramePoint = pointBin[0];
      PlotterVector2d radiiFrameVector = vectorBin[0];
      centerFramePoint.setIncludingFrame(frame, center);
      radiiFrameVector.setIncludingFrame(frame, radii.getX(), radii.getY());
      centerFramePoint.changeFrame(screenFrame);
      radiiFrameVector.changeFrame(screenFrame);
      radiiFrameVector.set(Math.abs(radiiFrameVector.getX()), Math.abs(radiiFrameVector.getY()));
      centerFramePoint.sub(radiiFrameVector);
      drawOvalFilled(pixelate(centerFramePoint.getX()), pixelate(centerFramePoint.getY()), pixelate(2.0 * radiiFrameVector.getX()), pixelate(2.0 * radiiFrameVector.getY()));
   }

   public void drawLineSegment(PlotterReferenceFrame frame, LineSegment2d lineSegment)
   {
      PlotterPoint2d firstEndpoint = pointBin[0];
      PlotterPoint2d secondEndpoint = pointBin[1];
      firstEndpoint.setIncludingFrame(frame, lineSegment.getFirstEndpoint());
      secondEndpoint.setIncludingFrame(frame, lineSegment.getSecondEndpoint());
      firstEndpoint.changeFrame(screenFrame);
      secondEndpoint.changeFrame(screenFrame);
      drawLineSegment(pixelate(firstEndpoint.getX()), pixelate(firstEndpoint.getY()), pixelate(secondEndpoint.getX()), pixelate(secondEndpoint.getY()));
   }

   public void drawLineSegment(PlotterReferenceFrame frame, double firstPointX, double firstPointY, double secondPointX, double secondPointY)
   {
      PlotterPoint2d firstEndpoint = pointBin[0];
      PlotterPoint2d secondEndpoint = pointBin[1];
      firstEndpoint.setIncludingFrame(frame, firstPointX, firstPointY);
      secondEndpoint.setIncludingFrame(frame, secondPointX, secondPointY);
      firstEndpoint.changeFrame(screenFrame);
      secondEndpoint.changeFrame(screenFrame);
      drawLineSegment(pixelate(firstEndpoint.getX()), pixelate(firstEndpoint.getY()), pixelate(secondEndpoint.getX()), pixelate(secondEndpoint.getY()));
   }

   public void drawPoint(PlotterReferenceFrame frame, Point2D point)
   {
      PlotterPoint2d plotterPoint = pointBin[0];
      plotterPoint.setIncludingFrame(frame, point);
      plotterPoint.changeFrame(screenFrame);
      drawLineSegment(pixelate(plotterPoint.getX()), pixelate(plotterPoint.getY()), pixelate(plotterPoint.getX()), pixelate(plotterPoint.getY()));
   }

   public void drawLine(PlotterReferenceFrame frame, Line2d line)
   {
      PlotterPoint2d start = pointBin[0];
      PlotterVector2d direction = vectorBin[0];
      start.setIncludingFrame(frame, line.getPoint());
      direction.setIncludingFrame(frame, line.getNormalizedVector());
      start.changeFrame(screenFrame);
      direction.changeFrame(screenFrame);
      direction.normalize();
      PlotterPoint2d farPointPositive = pointBin[1];
      PlotterPoint2d farPointNegative = pointBin[2];
      PlotterVector2d far = vectorBin[1];
      far.setIncludingFrame(direction);
      far.scale(1e4);
      farPointPositive.setIncludingFrame(start);
      farPointNegative.setIncludingFrame(start);
      farPointPositive.add(far);
      farPointNegative.sub(far);
      drawLineSegment(pixelate(farPointNegative.getX()), pixelate(farPointNegative.getY()), pixelate(farPointPositive.getX()), pixelate(farPointPositive.getY()));
   }

   public void drawPolygonFilled(PlotterReferenceFrame frame, ConvexPolygon2d convexPolygon2d)
   {
      if (setupForDrawPolygon(frame, convexPolygon2d))
         drawPolygonFilled(tempPoints[0], tempPoints[1], convexPolygon2d.getNumberOfVertices());
   }

   public void drawPolygon(PlotterReferenceFrame frame, ConvexPolygon2d convexPolygon2d)
   {
      if (setupForDrawPolygon(frame, convexPolygon2d))
         drawPolygon(tempPoints[0], tempPoints[1], convexPolygon2d.getNumberOfVertices());
   }

   public void drawArc(PlotterReferenceFrame frame, Point2D center, Vector2D radii, double startAngle, double arcAngle)
   {
      PlotterPoint2d plotCenter = pointBin[0];
      PlotterVector2d plotRadii = vectorBin[0];
      plotCenter.setIncludingFrame(frame, center);
      plotRadii.setIncludingFrame(frame, radii);
      plotCenter.changeFrame(screenFrame);
      plotRadii.changeFrame(screenFrame);
      graphics2d.drawArc(pixelate(plotCenter.getX()),
                         pixelate(plotCenter.getY()),
                         pixelate(2.0 * plotRadii.getX()),
                         pixelate(2.0 * plotRadii.getY()),
                         pixelate(BodeUnitsConverter.convertRadianToDegrees(startAngle)),
                         pixelate(BodeUnitsConverter.convertRadianToDegrees(arcAngle)));
   }

   public void drawString(PlotterReferenceFrame frame, String string, Point2D startPoint)
   {
      PlotterPoint2d plotStart = pointBin[0];
      plotStart.setIncludingFrame(frame, startPoint);
      plotStart.changeFrame(screenFrame);
      drawString(string, pixelate(plotStart.getX()), pixelate(plotStart.getY()));
   }

   private boolean setupForDrawPolygon(PlotterReferenceFrame frame, ConvexPolygon2d convexPolygon2d)
   {
      for (int i = 0; i < convexPolygon2d.getNumberOfVertices(); i++)
      {
         if (i == pointBin.length)
         {
            PrintTools.warn("You are attempting to draw a polygon with too many points: " + convexPolygon2d.getNumberOfVertices() + " points but graphics only allows " + pointBin.length);
            return false;
         }
         pointBin[i].setIncludingFrame(frame, convexPolygon2d.getVertex(i));
         pointBin[i].changeFrame(screenFrame);
         tempPoints[0][i] = pixelate(pointBin[i].getX());
         tempPoints[1][i] = pixelate(pointBin[i].getY());
      }
      return true;
   }

   // For plotter use

   public void drawString(String string, PlotterPoint2d startPoint)
   {
      startPoint.changeFrame(screenFrame);
      drawString(string, pixelate(startPoint.getX()), pixelate(startPoint.getY()));
   }

   public void drawImage(Image image, PlotterPoint2d firstCornerDestination, PlotterPoint2d secondCornerDesination,
                                      PlotterPoint2d firstCornerSource, PlotterPoint2d secondCornerSource, ImageObserver observer)
   {
      firstCornerDestination.changeFrame(screenFrame);
      secondCornerDesination.changeFrame(screenFrame);
      firstCornerSource.changeFrame(screenFrame);
      secondCornerSource.changeFrame(screenFrame);
      drawImage(image, pixelate(firstCornerDestination.getX()),
                       pixelate(firstCornerDestination.getY()),
                       pixelate(secondCornerDesination.getX()),
                       pixelate(secondCornerDesination.getY()),
                       pixelate(firstCornerSource.getX()),
                       pixelate(firstCornerSource.getY()),
                       pixelate(secondCornerSource.getX()),
                       pixelate(secondCornerSource.getY()), observer);
   }

   // Swing Graphics wrapper

   private void drawPolyline(int[] xPoints, int[] yPoints, int nPoints)
   {
      graphics2d.drawPolyline(xPoints, yPoints, nPoints);
   }

   private void drawLineSegment(int x1, int y1, int x2, int y2)
   {
      graphics2d.drawLine(x1, y1, x2, y2);
   }

   private void drawOval(int x, int y, int width, int height)
   {
      graphics2d.drawOval(x, y, width, height);
   }

   private void drawOvalFilled(int x, int y, int width, int height)
   {
      graphics2d.fillOval(x, y, width, height);
   }

   private void drawRectangle(int x, int y, int width, int height)
   {
      graphics2d.drawRect(x, y, width, height);
   }

   private void drawRectangleFilled(int x, int y, int width, int height)
   {
      graphics2d.fillRect(x, y, width, height);
   }

   private void drawRectangle3D(int x, int y, int width, int height, boolean raised)
   {
      graphics2d.draw3DRect(x, y, width, height, raised);
   }

   private void drawRectangle3DFilled(int x, int y, int width, int height, boolean raised)
   {
      graphics2d.fill3DRect(x, y, width, height, raised);
   }

   private void drawRectangleRounded(int x, int y, int width, int height, int arcWidth, int arcHeight)
   {
      graphics2d.drawRoundRect(x, y, width, height, arcWidth, arcHeight);
   }

   private void drawRectangleRoundedFilled(int x, int y, int width, int height, int arcWidth, int arcHeight)
   {
      graphics2d.fillRoundRect(x, y, width, height, arcWidth, arcHeight);
   }

   private void drawPolygonFilled(int[] xPoints, int[] yPoints, int nPoints)
   {
      graphics2d.fillPolygon(xPoints, yPoints, nPoints);
   }

   private void drawPolygon(int[] xPoints, int[] yPoints, int nPoints)
   {
      graphics2d.drawPolygon(xPoints, yPoints, nPoints);
   }

   private void drawPolygonFilled(Polygon p)
   {
      graphics2d.fillPolygon(p);
   }

   private void drawPolygon(Polygon p)
   {
      graphics2d.drawPolygon(p);
   }

   private void drawArc(int x, int y, int width, int height, int startAngle, int arcAngle)
   {
      graphics2d.drawArc(x, y, width, height, startAngle, arcAngle);
   }

   private void drawArcFilled(int x, int y, int width, int height, int startAngle, int arcAngle)
   {
      graphics2d.fillArc(x, y, width, height, startAngle, arcAngle);
   }

   private void drawString(String str, int x, int y)
   {
      graphics2d.drawString(str, x, y);
   }

   private void drawString(String str, float x, float y)
   {
      graphics2d.drawString(str, x, y);
   }

   private void drawString(AttributedCharacterIterator iterator, int x, int y)
   {
      graphics2d.drawString(iterator, x, y);
   }

   private void drawString(AttributedCharacterIterator iterator, float x, float y)
   {
      graphics2d.drawString(iterator, x, y);
   }

   private void drawGlyphVector(GlyphVector g, float x, float y)
   {
      graphics2d.drawGlyphVector(g, x, y);
   }

   private void drawBytes(byte[] data, int offset, int length, int x, int y)
   {
      graphics2d.drawBytes(data, offset, length, x, y);
   }

   private void drawChars(char[] data, int offset, int length, int x, int y)
   {
      graphics2d.drawChars(data, offset, length, x, y);
   }

   private void drawRenderedImage(RenderedImage img, AffineTransform xform)
   {
      graphics2d.drawRenderedImage(img, xform);
   }

   private void drawRenderableImage(RenderableImage img, AffineTransform xform)
   {
      graphics2d.drawRenderableImage(img, xform);
   }

   private boolean drawImage(Image img, AffineTransform xform, ImageObserver obs)
   {
      return graphics2d.drawImage(img, xform, obs);
   }

   private void drawImage(BufferedImage img, BufferedImageOp op, int x, int y)
   {
      graphics2d.drawImage(img, op, x, y);
   }

   private boolean drawImage(Image img, int x, int y, ImageObserver observer)
   {
      return graphics2d.drawImage(img, x, y, observer);
   }

   private boolean drawImage(Image img, int x, int y, Color bgcolor, ImageObserver observer)
   {
      return graphics2d.drawImage(img, x, y, bgcolor, observer);
   }

   private boolean drawImage(Image img, int x, int y, int width, int height, ImageObserver observer)
   {
      return graphics2d.drawImage(img, x, y, width, height, observer);
   }

   private boolean drawImage(Image img, int x, int y, int width, int height, Color bgcolor, ImageObserver observer)
   {
      return graphics2d.drawImage(img, x, y, width, height, bgcolor, observer);
   }

   private boolean drawImage(Image img, int dx1, int dy1, int dx2, int dy2, int sx1, int sy1, int sx2, int sy2, ImageObserver observer)
   {
      return graphics2d.drawImage(img, dx1, dy1, dx2, dy2, sx1, sy1, sx2, sy2, observer);
   }

   private boolean drawImage(Image img, int dx1, int dy1, int dx2, int dy2, int sx1, int sy1, int sx2, int sy2, Color bgcolor, ImageObserver observer)
   {
      return graphics2d.drawImage(img, dx1, dy1, dx2, dy2, sx1, sy1, sx2, sy2, bgcolor, observer);
   }

   public Graphics create()
   {
      return graphics2d.create();
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

   public PlotterReferenceFrame getMetersFrame()
   {
      return metersFrame;
   }

   public PlotterReferenceFrame getPixelsFrame()
   {
      return pixelsFrame;
   }

   public PlotterReferenceFrame getScreenFrame()
   {
      return screenFrame;
   }
}
