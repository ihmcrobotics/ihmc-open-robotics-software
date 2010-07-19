package us.ihmc.plotting;

import java.awt.Color;
import java.awt.Graphics;

public class Drawing2DTools
{
   public static void drawEmptyCircle(Graphics g, int x, int y, int radius, Color color)
   {
      g.setColor(color);
      g.drawOval((x - (radius / 2)), (y - (radius / 2)), radius, radius);
   }

   public static void drawFilledCircle(Graphics g, int x, int y, int radius, Color color)
   {
      g.setColor(color);
      g.fillOval((x - (radius / 2)), (y - (radius / 2)), radius, radius);
   }

   public static void drawCircleWithCross(Graphics g, int x, int y, int radius, Color color)
   {
      g.setColor(color);
      g.drawOval((x - (radius / 2)), (y - (radius / 2)), radius, radius);
      g.drawLine((x - (radius / 2)), y, (x + (radius / 2)), y);
      g.drawLine(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }

   public static void drawCircleWithRotatedCross(Graphics g, int x, int y, int radius, Color color)
   {
      g.setColor(color);
      int distance = (int)Math.round(0.707 * radius / 2);
      g.drawOval((x - radius / 2), (y - radius / 2), radius, radius);
      g.drawLine((x - distance), (y - distance), (x + distance), (y + distance));
      g.drawLine((x - distance), (y + distance), (x + distance), (y - distance));
   }

   public static void drawDiamond(Graphics g, int x, int y, int radius, Color color)
   {
      g.setColor(color);
      g.drawLine((x - (radius / 2)), y, x, (y + radius / 2));
      g.drawLine((x - (radius / 2)), y, x, (y - radius / 2));
      g.drawLine(x, (y + radius / 2), x + radius / 2, y);
      g.drawLine(x, (y - radius / 2), x + radius / 2, y);
   }

   public static void drawDiamondWithCross(Graphics g, int x, int y, int radius, Color color)
   {
      g.setColor(color);
      g.drawLine((x - (radius / 2)), y, x, (y + radius / 2));
      g.drawLine((x - (radius / 2)), y, x, (y - radius / 2));
      g.drawLine(x, (y + radius / 2), x + radius / 2, y);
      g.drawLine(x, (y - radius / 2), x + radius / 2, y);
      g.drawLine((x - (radius / 2)), y, (x + (radius / 2)), y);
      g.drawLine(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }

   public static void drawSquare(Graphics g, int x, int y, int radius, Color color)
   {
      g.setColor(color);
      g.drawLine((x - (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y - (radius / 2)));
      g.drawLine((x + (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
      g.drawLine((x - (radius / 2)), (y - (radius / 2)), (x - (radius / 2)), (y + (radius / 2)));
      g.drawLine((x - (radius / 2)), (y + (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
   }

   public static void drawSquareWithCross(Graphics g, int x, int y, int radius, Color color)
   {
      g.setColor(color);
      g.drawLine((x - (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y - (radius / 2)));
      g.drawLine((x + (radius / 2)), (y - (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
      g.drawLine((x - (radius / 2)), (y - (radius / 2)), (x - (radius / 2)), (y + (radius / 2)));
      g.drawLine((x - (radius / 2)), (y + (radius / 2)), (x + (radius / 2)), (y + (radius / 2)));
      g.drawLine((x - (radius / 2)), y, (x + (radius / 2)), y);
      g.drawLine(x, (y - (radius / 2)), x, (y + (radius / 2)));
   }

}
