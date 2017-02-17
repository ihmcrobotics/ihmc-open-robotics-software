package us.ihmc.graphicsDescription.plotting;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.LineSegment2d;

/**
 * This is a public interface for drawing geometry in the Plotter.
 * All geometry is in meters frame.
 */
public class Graphics2DAdapter
{
   private final Plotter2DAdapter plotter2dAdapter;
   
   public Graphics2DAdapter(Plotter2DAdapter plotter2dAdapter)
   {
      this.plotter2dAdapter = plotter2dAdapter;
   }

   public void setColor(Color color)
   {
      plotter2dAdapter.setColor(color);
   }

   public void setStroke(BasicStroke stroke)
   {
      plotter2dAdapter.setStroke(stroke);
   }

   public void setFont(Font font)
   {
      plotter2dAdapter.setFont(font);
   }

   public void drawCircleWithCross(Point2D center, Vector2D radii)
   {
      plotter2dAdapter.drawCircleWithCross(plotter2dAdapter.getMetersFrame(), center, radii);
   }

   public void drawCircleWithRotatedCross(Point2D center, Vector2D radii)
   {
      plotter2dAdapter.drawCircleWithRotatedCross(plotter2dAdapter.getMetersFrame(), center, radii);
   }

   public void drawDiamond(Point2D center, Vector2D radii)
   {
      plotter2dAdapter.drawDiamond(plotter2dAdapter.getMetersFrame(), center, radii);
   }

   public void drawDiamondWithCross(Point2D center, Vector2D radii)
   {
      plotter2dAdapter.drawDiamondWithCross(plotter2dAdapter.getMetersFrame(), center, radii);
   }

   public void drawSquare(Point2D center, Vector2D radii)
   {
      plotter2dAdapter.drawRectangle(plotter2dAdapter.getMetersFrame(), center, radii);
   }

   public void drawSquareWithCross(Point2D center, Vector2D radii)
   {
      plotter2dAdapter.drawSquareWithCross(plotter2dAdapter.getMetersFrame(), center, radii);
   }

   public void drawSquareFilled(Point2D center, Vector2D radii)
   {
      plotter2dAdapter.drawSquareFilled(plotter2dAdapter.getMetersFrame(), center, radii);
   }

   public void drawCross(Point2D center, Vector2D radii)
   {
      plotter2dAdapter.drawCross(plotter2dAdapter.getMetersFrame(), center, radii);
   }

   public void drawRotatedCross(Point2D center, Vector2D radii)
   {
      plotter2dAdapter.drawRotatedCross(plotter2dAdapter.getMetersFrame(), center, radii);
   }

   public void drawPolygon(ConvexPolygon2d polygon)
   {
      plotter2dAdapter.drawPolygon(plotter2dAdapter.getMetersFrame(), polygon);
   }

   public void drawPolygonFilled(ConvexPolygon2d polygon)
   {
      plotter2dAdapter.drawPolygonFilled(plotter2dAdapter.getMetersFrame(), polygon);
   }

   public void drawString(String string, Point2D start)
   {
      plotter2dAdapter.drawString(plotter2dAdapter.getMetersFrame(), string, start);
   }

   public void drawLine(Line2d line)
   {
      plotter2dAdapter.drawLine(plotter2dAdapter.getMetersFrame(), line);
   }

   public void drawLineSegment(LineSegment2d lineSegment)
   {
      plotter2dAdapter.drawLineSegment(plotter2dAdapter.getMetersFrame(), lineSegment);
   }

   public void drawLineSegment(double firstPointX, double firstPointY, double secondPointX, double secondPointY)
   {
      plotter2dAdapter.drawLineSegment(plotter2dAdapter.getMetersFrame(), firstPointX, firstPointY, secondPointX, secondPointY);
   }

   public void drawOval(Point2D center, Vector2D radii)
   {
      plotter2dAdapter.drawOval(plotter2dAdapter.getMetersFrame(), center, radii);
   }

   public void drawOvalFilled(Point2D center, Vector2D radii)
   {
      plotter2dAdapter.drawOvalFilled(plotter2dAdapter.getMetersFrame(), center, radii);
   }

   public void drawPoint(Point2D point)
   {
      plotter2dAdapter.drawPoint(plotter2dAdapter.getMetersFrame(), point);
   }

   public void drawArc(Point2D center, Vector2D radii, double startAngle, double arcAngle)
   {
      plotter2dAdapter.drawArc(plotter2dAdapter.getMetersFrame(), center, radii, startAngle, arcAngle);
   }
}
