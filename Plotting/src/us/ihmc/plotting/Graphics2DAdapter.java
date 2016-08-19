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

public class Graphics2DAdapter
{
   private final MetersReferenceFrame metersFrame;
   private final PixelsReferenceFrame screenFrame;
   
   private final PlotterPoint2d[] pointBin = new PlotterPoint2d[1];
   private final Vector2d[] vectorBin = new Vector2d[pointBin.length];
   
   private Graphics2D graphics2d;
   
   public Graphics2DAdapter(MetersReferenceFrame metersFrame, PixelsReferenceFrame screenFrame)
   {
      this.metersFrame = metersFrame;
      this.screenFrame = screenFrame;
      
      for (int i = 0; i < pointBin.length; i++)
      {
         pointBin[i] = new PlotterPoint2d(metersFrame);
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

   public void drawLine(int x1, int y1, int x2, int y2)
   {
      graphics2d.drawLine(x1, y1, x2, y2);
   }

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
      pointBin[0].setIncludingFrame(metersFrame, center);
      pointBin[0].add(-radii.getX(), radii.getY());
      pointBin[0].changeFrame(screenFrame);
      graphics2d.drawOval(pixelate(pointBin[0].getX()), pixelate(pointBin[0].getX()), pixelate(radii.getX() * 2.0), pixelate(radii.getY() * 2.0));
   }
   
   public void drawOvalFilled(int x, int y, int width, int height)
   {
      graphics2d.fillOval(x, y, width, height);
   }

   public void drawRectangle(int x, int y, int width, int height)
   {
      graphics2d.drawRect(x, y, width, height);
   }

   public void drawRectangleFilled(int x, int y, int width, int height)
   {
      graphics2d.fillRect(x, y, width, height);
   }

   public void drawRectangle3D(int x, int y, int width, int height, boolean raised)
   {
      graphics2d.draw3DRect(x, y, width, height, raised);
   }

   public void drawRectangle3DFilled(int x, int y, int width, int height, boolean raised)
   {
      graphics2d.fill3DRect(x, y, width, height, raised);
   }

   public void drawRectangleRounded(int x, int y, int width, int height, int arcWidth, int arcHeight)
   {
      graphics2d.drawRoundRect(x, y, width, height, arcWidth, arcHeight);
   }

   public void drawRectangleRoundedFilled(int x, int y, int width, int height, int arcWidth, int arcHeight)
   {
      graphics2d.fillRoundRect(x, y, width, height, arcWidth, arcHeight);
   }

   public void drawPolygonFilled(int[] xPoints, int[] yPoints, int nPoints)
   {
      graphics2d.fillPolygon(xPoints, yPoints, nPoints);
   }

   public void drawPolygon(int[] xPoints, int[] yPoints, int nPoints)
   {
      graphics2d.drawPolygon(xPoints, yPoints, nPoints);
   }

   public void drawPolygonFilled(Polygon p)
   {
      graphics2d.fillPolygon(p);
   }

   public void drawPolygon(Polygon p)
   {
      graphics2d.drawPolygon(p);
   }

   public void drawArc(int x, int y, int width, int height, int startAngle, int arcAngle)
   {
      graphics2d.drawArc(x, y, width, height, startAngle, arcAngle);
   }

   public void drawArcFilled(int x, int y, int width, int height, int startAngle, int arcAngle)
   {
      graphics2d.fillArc(x, y, width, height, startAngle, arcAngle);
   }

   public void drawString(String str, int x, int y)
   {
      graphics2d.drawString(str, x, y);
   }

   public void drawString(String str, float x, float y)
   {
      graphics2d.drawString(str, x, y);
   }

   public void drawString(AttributedCharacterIterator iterator, int x, int y)
   {
      graphics2d.drawString(iterator, x, y);
   }

   public void drawString(AttributedCharacterIterator iterator, float x, float y)
   {
      graphics2d.drawString(iterator, x, y);
   }

   public void drawGlyphVector(GlyphVector g, float x, float y)
   {
      graphics2d.drawGlyphVector(g, x, y);
   }

   public void drawBytes(byte[] data, int offset, int length, int x, int y)
   {
      graphics2d.drawBytes(data, offset, length, x, y);
   }

   public void drawChars(char[] data, int offset, int length, int x, int y)
   {
      graphics2d.drawChars(data, offset, length, x, y);
   }

   public void drawRenderedImage(RenderedImage img, AffineTransform xform)
   {
      graphics2d.drawRenderedImage(img, xform);
   }

   public void drawRenderableImage(RenderableImage img, AffineTransform xform)
   {
      graphics2d.drawRenderableImage(img, xform);
   }

   public boolean drawImage(Image img, AffineTransform xform, ImageObserver obs)
   {
      return graphics2d.drawImage(img, xform, obs);
   }

   public void drawImage(BufferedImage img, BufferedImageOp op, int x, int y)
   {
      graphics2d.drawImage(img, op, x, y);
   }

   public boolean drawImage(Image img, int x, int y, ImageObserver observer)
   {
      return graphics2d.drawImage(img, x, y, observer);
   }

   public boolean drawImage(Image img, int x, int y, Color bgcolor, ImageObserver observer)
   {
      return graphics2d.drawImage(img, x, y, bgcolor, observer);
   }

   public boolean drawImage(Image img, int x, int y, int width, int height, ImageObserver observer)
   {
      return graphics2d.drawImage(img, x, y, width, height, observer);
   }

   public boolean drawImage(Image img, int x, int y, int width, int height, Color bgcolor, ImageObserver observer)
   {
      return graphics2d.drawImage(img, x, y, width, height, bgcolor, observer);
   }

   public boolean drawImage(Image img, int dx1, int dy1, int dx2, int dy2, int sx1, int sy1, int sx2, int sy2, ImageObserver observer)
   {
      return graphics2d.drawImage(img, dx1, dy1, dx2, dy2, sx1, sy1, sx2, sy2, observer);
   }

   public boolean drawImage(Image img, int dx1, int dy1, int dx2, int dy2, int sx1, int sy1, int sx2, int sy2, Color bgcolor, ImageObserver observer)
   {
      return graphics2d.drawImage(img, dx1, dy1, dx2, dy2, sx1, sy1, sx2, sy2, bgcolor, observer);
   }

   public Graphics create()
   {
      return graphics2d.create();
   }

   public Graphics create(int x, int y, int width, int height)
   {
      return graphics2d.create(x, y, width, height);
   }

   public void clearRectangle(int x, int y, int width, int height)
   {
      graphics2d.clearRect(x, y, width, height);
   }

   public void clipRectangle(int x, int y, int width, int height)
   {
      graphics2d.clipRect(x, y, width, height);
   }

   public void setClip(int x, int y, int width, int height)
   {
      graphics2d.setClip(x, y, width, height);
   }

   public void copyArea(int x, int y, int width, int height, int dx, int dy)
   {
      graphics2d.copyArea(x, y, width, height, dx, dy);
   }

   public boolean hitClip(int x, int y, int width, int height)
   {
      return graphics2d.hitClip(x, y, width, height);
   }

   public void setClip(Shape clip)
   {
      graphics2d.setClip(clip);
   }

   public boolean hit(Rectangle rect, Shape s, boolean onStroke)
   {
      return graphics2d.hit(rect, s, onStroke);
   }

   public Rectangle getClipBounds(Rectangle r)
   {
      return graphics2d.getClipBounds(r);
   }

   public void clip(Shape s)
   {
      graphics2d.clip(s);
   }

   public void translate(int x, int y)
   {
      graphics2d.translate(x, y);
   }

   public void rotate(double theta)
   {
      graphics2d.rotate(theta);
   }

   public void translate(double tx, double ty)
   {
      graphics2d.translate(tx, ty);
   }

   public void rotate(double theta, double x, double y)
   {
      graphics2d.rotate(theta, x, y);
   }

   public void scale(double sx, double sy)
   {
      graphics2d.scale(sx, sy);
   }

   public void shear(double shx, double shy)
   {
      graphics2d.shear(shx, shy);
   }

   public void transform(AffineTransform Tx)
   {
      graphics2d.transform(Tx);
   }

   public void setTransform(AffineTransform Tx)
   {
      graphics2d.setTransform(Tx);
   }

   public AffineTransform getTransform()
   {
      return graphics2d.getTransform();
   }

   public Shape getClip()
   {
      return graphics2d.getClip();
   }

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
