package us.ihmc.graphics3DDescription.plotting.artifact;

import java.awt.BasicStroke;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.graphics3DDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphics3DDescription.plotting.Plotter2DAdapter;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.LineSegment2d;

public class LineArtifact extends Artifact
{
   private static final BasicStroke STROKE = new BasicStroke(1.0f);
   
   private final Point2d point1 = new Point2d();
   private final Point2d point2 = new Point2d(0.01, 0.01);

   private final LineSegment2d tempLineSegment = new LineSegment2d();
   
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
      
      if (LineSegment2d.areEndpointsTheSame(point1, point2))
      {
         graphics.drawPoint(point1);
      }
      else
      {
         tempLineSegment.set(point1, point2);
         graphics.drawLineSegment(tempLineSegment);
      }
   }

   @Override
   public void drawLegend(Plotter2DAdapter graphics, Point2d origin)
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
