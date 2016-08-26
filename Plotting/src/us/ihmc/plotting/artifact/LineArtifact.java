package us.ihmc.plotting.artifact;

import java.awt.BasicStroke;
import java.awt.Stroke;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.plotting.Graphics2DAdapter;
import us.ihmc.robotics.geometry.Line2d;

public class LineArtifact extends Artifact
{
   private final Point2d point1 = new Point2d();
   private final Point2d point2 = new Point2d(0.01, 0.01);
   private int lineThickness = 1;

   public LineArtifact(String id)
   {
      super(id);
   }

   /**
    * Set to normalized line. Length = 1.
    */
   public LineArtifact(String id, Line2d line)
   {
      super(id);
      setLine(line);
   }

   public LineArtifact(String id, Point2d point1, Point2d point2)
   {
      super(id);
      this.point1.set(point1);
      this.point2.set(point2);
   }

   public void setLine(Line2d line)
   {
      line.getTwoPointsOnLine(point1, point2);
   }

   public void setPoints(Point2d point1, Point2d point2)
   {
      this.point1.set(point1);
      this.point2.set(point2);
   }

   public void setPoints(Point2d point, Vector2d vector)
   {
      point1.set(point);
      point2.add(point1, vector);
   }

   public void setLineThicknessInPixels(int pixels)
   {
      lineThickness = pixels;
   }

   /**
    * Must provide a draw method for plotter to render artifact
    */
   @Override
   public void draw(Graphics2DAdapter graphics2d, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (isVisible)
      {
         int x1 = Xcenter + ((int) Math.round(point1.getX() * scaleFactor));
         int y1 = Ycenter - ((int) Math.round(point1.getY() * scaleFactor));
         int x2 = Xcenter + ((int) Math.round(point2.getX() * scaleFactor));
         int y2 = Ycenter - ((int) Math.round(point2.getY() * scaleFactor));
         graphics2d.setColor(color);
         Stroke currentStroke = graphics2d.getStroke();
         graphics2d.setStroke(new BasicStroke(lineThickness));
         graphics2d.drawLineSegment(x1, y1, x2, y2);
         graphics2d.setStroke(currentStroke);
      }
   }

   @Override
   public void drawLegend(Graphics2DAdapter graphics2d, int Xcenter, int Ycenter, double scaleFactor)
   {
   }

   @Override
   public void drawHistory(Graphics2DAdapter graphics2d, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }
}
