package us.ihmc.graphicsDescription.plotting.artifact;

import java.awt.BasicStroke;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;

public class LineArtifact extends Artifact
{
   private static final BasicStroke STROKE = new BasicStroke(1.0f);
   
   private final Point2D point1 = new Point2D();
   private final Point2D point2 = new Point2D(0.01, 0.01);

   private final LineSegment2D tempLineSegment = new LineSegment2D();
   
   public LineArtifact(String id)
   {
      super(id);
   }

   /**
    * Set to normalized line. Length = 1.
    */
   public LineArtifact(String id, Line2D line)
   {
      super(id);
      setLine(line);
   }

   public LineArtifact(String id, Point2DReadOnly point1, Point2DReadOnly point2)
   {
      super(id);
      this.point1.set(point1);
      this.point2.set(point2);
   }

   public void setLine(Line2D line)
   {
      line.getTwoPointsOnLine(point1, point2);
   }

   public void setPoints(Point2DReadOnly point1, Point2DReadOnly point2)
   {
      this.point1.set(point1);
      this.point2.set(point2);
   }

   public void setPoints(Point2DReadOnly point, Vector2DReadOnly vector)
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
      
      if (point1.equals(point2))
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
