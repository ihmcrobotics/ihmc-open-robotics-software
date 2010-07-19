package us.ihmc.plotting.shapes;

import java.awt.*;
import java.io.*;

import javax.vecmath.*;

import us.ihmc.plotting.*;
import us.ihmc.utilities.math.geometry.*;

public class LineArtifact extends Artifact implements Serializable
{
   /**
    * 
    */
   private static final long serialVersionUID = -4292453853922762342L;
   private Point2d point1;
   private Point2d point2;
   private int lineThickness = 1;

   public LineArtifact(String id)
   {
      super(id);
   }

   public LineArtifact(String id, Line2d line)
   {
      super(id);
      setLine(line);
   }

   public LineArtifact(String id, Point2d point1, Point2d point2)
   {
      super(id);
      this.point1 = point1;
      this.point2 = point2;
   }


   public void setLine(Line2d line)
   {
      setPoints(line.getPointCopy(), line.getNormalizedVectorCopy());
   }

   public void setPoints(Point2d point1, Point2d point2)
   {
      this.point1 = point1;
      this.point2 = point2;
   }

   public void setPoints(Point2d point, Vector2d vector)
   {
      this.point1 = point;
      Point2d endOfVector = new Point2d(point);
      endOfVector.add(vector);
      this.point2 = endOfVector;
   }

   public void setLineThicknessInPixels(int pixels)
   {
      this.lineThickness = pixels;
   }


   /**
    * Must provide a draw method for plotter to render artifact
    */
   public void draw(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      if (isVisible)
      {
         int x1 = Xcenter + ((int)Math.round(point1.x * scaleFactor));
         int y1 = Ycenter - ((int)Math.round(point1.y * scaleFactor));
         int x2 = Xcenter + ((int)Math.round(point2.x * scaleFactor));
         int y2 = Ycenter - ((int)Math.round(point2.y * scaleFactor));
         g.setColor(color);
         Graphics2D g2d = (Graphics2D) g;
         Stroke currentStroke = g2d.getStroke();
         g2d.setStroke(new BasicStroke(lineThickness));
         g.drawLine(x1, y1, x2, y2);
         g2d.setStroke(currentStroke);
      }
   }

   public void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
   }
}
