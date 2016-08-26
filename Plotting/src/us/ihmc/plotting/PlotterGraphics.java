package us.ihmc.plotting;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.LineSegment2d;

@Deprecated
public class PlotterGraphics
{
   private int xCenter, yCenter;
   private double xScale, yScale;

   // private int width, height;

   public PlotterGraphics()
   {
   }

   public void setXCenter(int xCenter)
   {
      this.xCenter = xCenter;
   }

   public void setYCenter(int yCenter)
   {
      this.yCenter = yCenter;
   }

   public void setCenter(int xCenter, int yCenter)
   {
      this.xCenter = xCenter;
      this.yCenter = yCenter;
   }

   public void setScale(double scale)
   {
      this.xScale = scale;
      this.yScale = scale;
   }

   // public void setWidth(int width)
   // {
   //     this.width = width;
   // }
   //
   // public void setHeight(int height)
   // {
   //     this.height = height;
   // }

   public void drawLineGivenStartAndVector(Graphics2DAdapter graphics, double x0, double y0, double vx, double vy)
   {
      double bigNumber = 2000.0 / xScale;
      vx = vx * bigNumber;
      vy = vy * bigNumber;

      double farX0 = x0 + vx;
      double farY0 = y0 + vy;

      double farX1 = x0 - vx;
      double farY1 = y0 - vy;

      drawLineSegment(graphics, farX0, farY0, farX1, farY1);
   }

   public void drawLine(Graphics2DAdapter graphics, Line2d line2d)
   {
      Vector2d vector2d = new Vector2d();
      line2d.getNormalizedVector(vector2d);

      double bigNumber = 2000.0 / xScale;
      vector2d.scale(bigNumber);

      Point2d farPoint1 = new Point2d();
      line2d.getPoint(farPoint1);
      farPoint1.add(vector2d);

      Point2d farPoint2 = new Point2d();
      line2d.getPoint(farPoint2);
      farPoint2.sub(vector2d);

      drawLineSegment(graphics, farPoint1, farPoint2);
   }

   public void drawLineSegment(Graphics2DAdapter graphics, LineSegment2d lineSegment)
   {
      drawLineSegment(graphics, lineSegment.getEndpointsCopy());
   }

   public void drawLineSegment(Graphics2DAdapter graphics, Point2d[] endPoints)
   {
      drawLineSegment(graphics, endPoints[0], endPoints[1]);
   }

   public void drawLineSegment(Graphics2DAdapter graphics, Point2d point1, Point2d point2)
   {
      drawLineSegment(graphics, point1.getX(), point1.getY(), point2.getX(), point2.getY());
   }

   public void drawLineSegment(Graphics2DAdapter graphics, double x1, double y1, double x2, double y2)
   {
      int x1Int = xDoubleToInt(x1);
      int y1Int = yDoubleToInt(y1);
      int x2Int = xDoubleToInt(x2);
      int y2Int = yDoubleToInt(y2);

      graphics.drawLineSegment(x1Int, y1Int, x2Int, y2Int);
   }

   public void drawPolygon(Graphics2DAdapter graphics, double[][] polygonPoints, int nPoints)
   {
      int[] xPoints = new int[nPoints];
      int[] yPoints = new int[nPoints];

      for (int i = 0; i < nPoints; i++)
      {
         xPoints[i] = xDoubleToInt(polygonPoints[i][0]);
         yPoints[i] = yDoubleToInt(polygonPoints[i][1]);
      }

      graphics.drawPolygon(xPoints, yPoints, nPoints);
   }

   public void drawPolygon(Graphics2DAdapter graphics, double[][] polygonPoints)
   {
      int nPoints = polygonPoints.length;

      drawPolygon(graphics, polygonPoints, nPoints);
   }

   public void drawPolygon(Graphics2DAdapter graphics, ConvexPolygon2d polygon)
   {
      int nPoints = polygon.getNumberOfVertices();

      if (nPoints < 1)
         return;

      int[] xPoints = new int[nPoints];
      int[] yPoints = new int[nPoints];

      for (int i = 0; i < nPoints; i++)
      {
         Point2d point2d = polygon.getVertex(i);

         xPoints[i] = xDoubleToInt(point2d.getX());
         yPoints[i] = yDoubleToInt(point2d.getY());
      }

      graphics.drawPolygon(xPoints, yPoints, nPoints);
   }

   public void drawPolygon(Graphics2DAdapter graphics, Point2d[] polygonPoints)
   {
      int nPoints = polygonPoints.length;

      int[] xPoints = new int[nPoints];
      int[] yPoints = new int[nPoints];

      for (int i = 0; i < nPoints; i++)
      {
         Point2d point2d = polygonPoints[i];

         xPoints[i] = xDoubleToInt(point2d.getX());
         yPoints[i] = yDoubleToInt(point2d.getY());
      }

      graphics.drawPolygon(xPoints, yPoints, nPoints);
   }

   public void fillPolygon(Graphics2DAdapter graphics, double[][] polygonPoints, int nPoints)
   {
      int[] xPoints = new int[nPoints];
      int[] yPoints = new int[nPoints];

      for (int i = 0; i < nPoints; i++)
      {
         xPoints[i] = xDoubleToInt(polygonPoints[i][0]);
         yPoints[i] = yDoubleToInt(polygonPoints[i][1]);
      }

      graphics.drawPolygonFilled(xPoints, yPoints, nPoints);
   }

   public void fillPolygon(Graphics2DAdapter graphics, double[][] polygonPoints)
   {
      int nPoints = polygonPoints.length;

      fillPolygon(graphics, polygonPoints, nPoints);
   }

   public void fillPolygon(Graphics2DAdapter graphics, ConvexPolygon2d polygon)
   {
      int nPoints = polygon.getNumberOfVertices();

      if (nPoints < 1)
         return;

      int[] xPoints = new int[nPoints];
      int[] yPoints = new int[nPoints];

      for (int i = 0; i < nPoints; i++)
      {
         Point2d point2d = polygon.getVertex(i);

         xPoints[i] = xDoubleToInt(point2d.getX());
         yPoints[i] = yDoubleToInt(point2d.getY());
      }

      graphics.drawPolygonFilled(xPoints, yPoints, nPoints);
   }

   public void fillPolygon(Graphics2DAdapter graphics, ArrayList<Point2d> polygonPoints)
   {
      int nPoints = polygonPoints.size();

      int[] xPoints = new int[nPoints];
      int[] yPoints = new int[nPoints];

      for (int i = 0; i < nPoints; i++)
      {
         Point2d point2d = polygonPoints.get(i);

         xPoints[i] = xDoubleToInt(point2d.getX());
         yPoints[i] = yDoubleToInt(point2d.getY());
      }

      graphics.drawPolygonFilled(xPoints, yPoints, nPoints);
   }

   public void fillPolygon(Graphics2DAdapter graphics, Point2d[] polygonPoints)
   {
      int nPoints = polygonPoints.length;

      int[] xPoints = new int[nPoints];
      int[] yPoints = new int[nPoints];

      for (int i = 0; i < nPoints; i++)
      {
         Point2d point2d = polygonPoints[i];

         xPoints[i] = xDoubleToInt(point2d.getX());
         yPoints[i] = yDoubleToInt(point2d.getY());
      }

      graphics.drawPolygonFilled(xPoints, yPoints, nPoints);
   }

   // private double intToDouble(int value, double center, double scale, int extent)
   // {
   //    return (value - extent/2) / scale + center;
   // }

   private int xDoubleToInt(double x)
   {
      return ((int) (x * xScale + xCenter));
   }

   private int yDoubleToInt(double y)
   {
      return ((int) (-y * yScale + yCenter));
   }
}
