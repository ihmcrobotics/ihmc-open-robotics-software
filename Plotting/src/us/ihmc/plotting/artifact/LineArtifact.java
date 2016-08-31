package us.ihmc.plotting.artifact;

import java.awt.BasicStroke;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.plotting.Graphics2DAdapter;
import us.ihmc.robotics.geometry.Line2d;

public class LineArtifact extends Artifact
{
   private static final BasicStroke STROKE = new BasicStroke(1.0f);
   
   private final Point2d point1 = new Point2d();
   private final Point2d point2 = new Point2d(0.01, 0.01);

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

   /**
    * Must provide a draw method for plotter to render artifact
    */
   @Override
   public void draw(Graphics2DAdapter graphics)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);
      
      graphics.drawLineSegment(point1.getX(), point1.getY(), point2.getX(), point2.getY());
   }

   @Override
   public void drawLegend(Graphics2DAdapter graphics, int centerX, int centerY)
   {
   }

   @Override
   public void drawHistory(Graphics2DAdapter graphics)
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }
}
