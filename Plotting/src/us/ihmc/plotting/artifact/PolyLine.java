package us.ihmc.plotting.artifact;

import java.awt.BasicStroke;
import java.awt.Stroke;
import java.util.ArrayList;

import javax.vecmath.Point2d;

import us.ihmc.plotting.Graphics2DAdapter;

public class PolyLine extends Artifact
{
   private ArrayList<Point2d> points;
   private int lineThickness = 1;

   public PolyLine(String id)
   {
      super(id);
   }

   public PolyLine(String id, ArrayList<Point2d> points)
   {
      super(id);
      this.points = points;
   }

   public void setPoints(ArrayList<Point2d> points)
   {
      this.points = points;
   }

   public void setLineThicknessInPixels(int pixels)
   {
      this.lineThickness = pixels;
   }

   /**
    * Must provide a draw method for plotter to render artifact
    */
   @Override
   public void draw(Graphics2DAdapter graphics2d, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (isVisible)
      {
         for (int i = 0; i < points.size(); i++)
         {
            int x1 = Xcenter + ((int) Math.round(points.get(i - 1).getX() * scaleFactor));
            int y1 = Ycenter - ((int) Math.round(points.get(i - 1).getY() * scaleFactor));
            int x2 = Xcenter + ((int) Math.round(points.get(i).getX() * scaleFactor));
            int y2 = Ycenter - ((int) Math.round(points.get(i).getY() * scaleFactor));
            graphics2d.setColor(color);
            Stroke currentStroke = graphics2d.getStroke();
            graphics2d.setStroke(new BasicStroke(lineThickness));
            graphics2d.drawLineSegment(x1, y1, x2, y2);
            graphics2d.setStroke(currentStroke);
         }
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
