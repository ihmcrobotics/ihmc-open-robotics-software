package us.ihmc.plotting.shapes;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;

import javax.vecmath.Point2d;

import us.ihmc.plotting.Artifact;

public class PolygonArtifact extends Artifact
{
   /**
    * 
    */
   private static final long serialVersionUID = -1874963815283452026L;
   private ArrayList<Point2d> points = new ArrayList<Point2d>();
   private boolean FILL_POLYGON = false;

   public PolygonArtifact(String id)
   {
      super(id);
      setLevel(2);
   }

   public PolygonArtifact(String id, boolean fill)
   {
      super(id);
      setLevel(2);
      FILL_POLYGON = fill;
   }

   public PolygonArtifact(String id, boolean fill, Color color)
   {
      super(id);
      setLevel(2);
      FILL_POLYGON = fill;
      this.color = color;
   }

   public PolygonArtifact(String id, boolean fill, Color color, ArrayList<Point2d> points)
   {
      super(id);
      setLevel(2);
      FILL_POLYGON = fill;
      this.color = color;
      setPoints(points);
   }



   public void setPoints(ArrayList<Point2d> points)
   {
      this.points.clear();
      this.points.addAll(points);
   }

   public void addPoint(Point2d point)
   {
      this.points.add(point);
   }

   public void clearAllPoints()
   {
      this.points.clear();
   }

   public ArrayList<Point2d> getPoints()
   {
      return this.points;
   }

   public int getSize()
   {
      return points.size();
   }

   public String getID()
   {
      return id;
   }


   /**
    * Must provide a draw method for plotter to render artifact
    */
   public void draw(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      int nPoints = points.size();
      int[] xPoints = new int[nPoints];
      int[] yPoints = new int[nPoints];

      if (nPoints == 1)
      {
         int x = Xcenter + (int)Math.round(points.get(0).x * scaleFactor);
         int y = Ycenter - (int)Math.round(points.get(0).y * scaleFactor);
         g.fillOval(x, y, 4, 4);
      }
      else
      {
         for (int i = 0; i < nPoints; i++)
         {
            int x = Xcenter + (int)Math.round(points.get(i).x * scaleFactor);
            int y = Ycenter - (int)Math.round(points.get(i).y * scaleFactor);
            xPoints[i] = x;
            yPoints[i] = y;
         }

         g.setColor(color);

         if (FILL_POLYGON)
         {
            g.fillPolygon(xPoints, yPoints, nPoints);
         }
         else
         {
            g.drawPolygon(xPoints, yPoints, nPoints);
         }
      }
   }


   public void drawLegend(Graphics g, int xCenter, int yCenter, double scaleFactor)
   {
      g.drawString("Polygon", xCenter, yCenter);
   }
   
   public void drawHistory(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }
   
   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }
}
