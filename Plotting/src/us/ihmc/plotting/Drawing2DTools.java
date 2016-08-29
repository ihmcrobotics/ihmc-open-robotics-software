package us.ihmc.plotting;

import java.awt.Color;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

public class Drawing2DTools
{
   @Deprecated
   public static void drawEmptyCircle(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawOval((x - (radius / 2)), (y - (radius / 2)), radius, radius);
   }
   
   public static void drawEmptyCircle(Graphics2DAdapter graphics, Point2d center, Vector2d radii)
   {
      graphics.drawOval(center, radii);
   }

   @Deprecated
   public static void drawFilledCircle(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawOvalFilled((x - (radius / 2)), (y - (radius / 2)), radius, radius);
   }
   
   public static void drawFilledCircle(Graphics2DAdapter graphics, Point2d center, Vector2d radii)
   {
      graphics.drawOvalFilled(center, radii);
   }
   
   @Deprecated
   public static void drawCross(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawLineSegment((x - (radius / 2)), y, (x + (radius / 2)), y);
      graphics2d.drawLineSegment(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }
   
   public static void drawCross(Graphics2DAdapter graphics, Point2d center, Vector2d radii)
   {
      graphics.drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX() + radii.getX(), center.getY());
      graphics.drawLineSegment(center.getX(), center.getY() - radii.getY(), center.getX(), center.getY() + radii.getY());
   }

   @Deprecated
   public static void drawCircleWithCross(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawOval((x - (radius / 2)), (y - (radius / 2)), radius, radius);
      graphics2d.drawLineSegment((x - (radius / 2)), y, (x + (radius / 2)), y);
      graphics2d.drawLineSegment(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }
   
   public static void drawCircleWithCross(Graphics2DAdapter graphics, Point2d center, Vector2d radii)
   {
      graphics.drawOval(center, radii);
      graphics.drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX() + radii.getX(), center.getY());
      graphics.drawLineSegment(center.getX(), center.getY() - radii.getY(), center.getX(), center.getY() + radii.getY());
   }

   @Deprecated
   public static void drawRotatedCross(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      int distance = (int)Math.round(0.707 * radius / 2);
      graphics2d.drawLineSegment((x - distance), (y - distance), (x + distance), (y + distance));
      graphics2d.drawLineSegment((x - distance), (y + distance), (x + distance), (y - distance));
   }

   public static void drawRotatedCross(Graphics2DAdapter graphics, Point2d center, Vector2d radii)
   {
      double distanceX = 0.707 * radii.getX();
      double distanceY = 0.707 * radii.getY();
      graphics.drawLineSegment(center.getX() - distanceX, center.getY() - distanceY, center.getX() + distanceX, center.getY() + distanceY);
      graphics.drawLineSegment(center.getX() - distanceX, center.getY() + distanceY, center.getX() + distanceX, center.getY() - distanceY);
   }

   @Deprecated
   public static void drawCircleWithRotatedCross(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      int distance = (int)Math.round(0.707 * radius / 2);
      graphics2d.drawOval((x - radius / 2), (y - radius / 2), radius, radius);
      graphics2d.drawLineSegment((x - distance), (y - distance), (x + distance), (y + distance));
      graphics2d.drawLineSegment((x - distance), (y + distance), (x + distance), (y - distance));
   }
   
   public static void drawCircleWithRotatedCross(Graphics2DAdapter graphics, Point2d center, Vector2d radii)
   {
      double distanceX = 0.707 * radii.getX();
      double distanceY = 0.707 * radii.getY();
      graphics.drawOval(center, radii);
      graphics.drawLineSegment(center.getX() - distanceX, center.getY() - distanceY, center.getX() + distanceX, center.getY() + distanceY);
      graphics.drawLineSegment(center.getX() - distanceX, center.getY() + distanceY, center.getX() + distanceX, center.getY() - distanceY);
   }

   @Deprecated
   public static void drawDiamond(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawLineSegment((x - (radius / 2)), y, x, (y + radius / 2));
      graphics2d.drawLineSegment((x - (radius / 2)), y, x, (y - radius / 2));
      graphics2d.drawLineSegment(x, (y + radius / 2), x + radius / 2, y);
      graphics2d.drawLineSegment(x, (y - radius / 2), x + radius / 2, y);
   }
   
   public static void drawDiamond(Graphics2DAdapter graphics, Point2d center, Vector2d radii)
   {
      graphics.drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX(), center.getY() + radii.getY());
      graphics.drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX(), center.getY() - radii.getY());
      graphics.drawLineSegment(center.getX(), center.getY() + radii.getY(), center.getX() + radii.getX(), center.getY());
      graphics.drawLineSegment(center.getX(), center.getY() - radii.getY(), center.getX() + radii.getX(), center.getY());
   }

   @Deprecated
   public static void drawDiamondWithCross(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawLineSegment((x - (radius / 2)), y, x, (y + radius / 2));
      graphics2d.drawLineSegment((x - (radius / 2)), y, x, (y - radius / 2));
      graphics2d.drawLineSegment(x, (y + radius / 2), x + radius / 2, y);
      graphics2d.drawLineSegment(x, (y - radius / 2), x + radius / 2, y);
      graphics2d.drawLineSegment((x - (radius / 2)), y, (x + (radius / 2)), y);
      graphics2d.drawLineSegment(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }
   
   public static void drawDiamondWithCross(Graphics2DAdapter graphics, Point2d center, Vector2d radii)
   {
      graphics.drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX(), center.getY() + radii.getY());
      graphics.drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX(), center.getY() - radii.getY());
      graphics.drawLineSegment(center.getX(), center.getY() + radii.getY(), center.getX() + radii.getX(), center.getY());
      graphics.drawLineSegment(center.getX(), center.getY() - radii.getY(), center.getX() + radii.getX(), center.getY());
      graphics.drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX() + radii.getX(), center.getY());
      graphics.drawLineSegment(center.getX(), center.getY() - radii.getY(), center.getX(), center.getY() + radii.getY());
   }

   @Deprecated
   public static void drawSquare(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawLineSegment((x - (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y - (radius / 2)));
      graphics2d.drawLineSegment((x + (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
      graphics2d.drawLineSegment((x - (radius / 2)), (y - (radius / 2)), (x - (radius / 2)), (y + (radius / 2)));
      graphics2d.drawLineSegment((x - (radius / 2)), (y + (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
   }
   
   public static void drawSquare(Graphics2DAdapter graphics, Point2d center, Vector2d radii)
   {
      graphics.drawLineSegment(center.getX() - radii.getX(), center.getY() - radii.getY(), center.getX() + radii.getX(), center.getY() - radii.getY());
      graphics.drawLineSegment(center.getX() + radii.getX(), center.getY() - radii.getY(), center.getX() + radii.getX(), center.getY() + radii.getY());
      graphics.drawLineSegment(center.getX() - radii.getX(), center.getY() - radii.getY(), center.getX() - radii.getX(), center.getY() + radii.getY());
      graphics.drawLineSegment(center.getX() - radii.getX(), center.getY() + radii.getY(), center.getX() + radii.getX(), center.getY() + radii.getY());
   }

   @Deprecated
   public static void drawSquareWithCross(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawLineSegment((x - (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y - (radius / 2)));
      graphics2d.drawLineSegment((x + (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
      graphics2d.drawLineSegment((x - (radius / 2)), (y - (radius / 2)), (x - (radius / 2)), (y + (radius / 2)));
      graphics2d.drawLineSegment((x - (radius / 2)), (y + (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
      graphics2d.drawLineSegment((x - (radius / 2)), y, (x + (radius / 2)), y);
      graphics2d.drawLineSegment(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }
   
   public static void drawSquareWithCross(Graphics2DAdapter graphics, Point2d center, Vector2d radii)
   {
      drawSquare(graphics, center, radii);
      graphics.drawLineSegment(center.getX() - radii.getX(), center.getY(), center.getX() + radii.getX(), center.getY());
      graphics.drawLineSegment(center.getX(), center.getY() - radii.getY(), center.getX(), center.getY() + radii.getY());
   }
}
