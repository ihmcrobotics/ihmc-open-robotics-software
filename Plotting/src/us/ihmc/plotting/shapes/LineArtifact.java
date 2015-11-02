package us.ihmc.plotting.shapes;

import java.awt.BasicStroke;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.io.Serializable;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.plotting.Artifact;
import us.ihmc.robotics.geometry.Line2d;

public class LineArtifact extends Artifact implements Serializable
{
   private static final long serialVersionUID = -4292453853922762342L;
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
   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
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

   @Override
   public void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
   }

   @Override
   public void drawHistory(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }
}
