package us.ihmc.plotting;

import java.awt.Color;

public class Drawing2DTools
{
   public static void drawEmptyCircle(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawOval((x - (radius / 2)), (y - (radius / 2)), radius, radius);
   }

   public static void drawFilledCircle(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.fillOval((x - (radius / 2)), (y - (radius / 2)), radius, radius);
   }

   public static void drawCircleWithCross(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawOval((x - (radius / 2)), (y - (radius / 2)), radius, radius);
      graphics2d.drawLine((x - (radius / 2)), y, (x + (radius / 2)), y);
      graphics2d.drawLine(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }

   public static void drawCircleWithRotatedCross(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      int distance = (int)Math.round(0.707 * radius / 2);
      graphics2d.drawOval((x - radius / 2), (y - radius / 2), radius, radius);
      graphics2d.drawLine((x - distance), (y - distance), (x + distance), (y + distance));
      graphics2d.drawLine((x - distance), (y + distance), (x + distance), (y - distance));
   }

   public static void drawDiamond(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawLine((x - (radius / 2)), y, x, (y + radius / 2));
      graphics2d.drawLine((x - (radius / 2)), y, x, (y - radius / 2));
      graphics2d.drawLine(x, (y + radius / 2), x + radius / 2, y);
      graphics2d.drawLine(x, (y - radius / 2), x + radius / 2, y);
   }

   public static void drawDiamondWithCross(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawLine((x - (radius / 2)), y, x, (y + radius / 2));
      graphics2d.drawLine((x - (radius / 2)), y, x, (y - radius / 2));
      graphics2d.drawLine(x, (y + radius / 2), x + radius / 2, y);
      graphics2d.drawLine(x, (y - radius / 2), x + radius / 2, y);
      graphics2d.drawLine((x - (radius / 2)), y, (x + (radius / 2)), y);
      graphics2d.drawLine(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }

   public static void drawSquare(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawLine((x - (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y - (radius / 2)));
      graphics2d.drawLine((x + (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
      graphics2d.drawLine((x - (radius / 2)), (y - (radius / 2)), (x - (radius / 2)), (y + (radius / 2)));
      graphics2d.drawLine((x - (radius / 2)), (y + (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
   }

   public static void drawSquareWithCross(Graphics2DAdapter graphics2d, int x, int y, int radius, Color color)
   {
      graphics2d.setColor(color);
      graphics2d.drawLine((x - (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y - (radius / 2)));
      graphics2d.drawLine((x + (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
      graphics2d.drawLine((x - (radius / 2)), (y - (radius / 2)), (x - (radius / 2)), (y + (radius / 2)));
      graphics2d.drawLine((x - (radius / 2)), (y + (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
      graphics2d.drawLine((x - (radius / 2)), y, (x + (radius / 2)), y);
      graphics2d.drawLine(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }
}
