package us.ihmc.graphicsDescription.plotting.artifact;

import java.awt.BasicStroke;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.LineSegment2d;

public class LineArtifact extends Artifact
{
   private static final BasicStroke STROKE = new BasicStroke(1.0f);
   
   private final Point2D point1 = new Point2D();
   private final Point2D point2 = new Point2D(0.01, 0.01);

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

   public LineArtifact(String id, Point2D point1, Point2D point2)
   {
      super(id);
      this.point1.set(point1);
      this.point2.set(point2);
   }

   public void setLine(Line2d line)
   {
      line.getTwoPointsOnLine(point1, point2);
   }

   public void setPoints(Point2D point1, Point2D point2)
   {
      this.point1.set(point1);
      this.point2.set(point2);
   }

   public void setPoints(Point2D point, Vector2D vector)
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
   public void drawLegend(Plotter2DAdapter graphics, Point2D origin)
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
