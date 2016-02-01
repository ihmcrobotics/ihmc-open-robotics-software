package us.ihmc.plotting.shapes;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.plotting.Artifact;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.geometry.ConvexPolygon2d;

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

   public PolygonArtifact(String id, boolean fill, Color color, List<Point2d> points)
   {
      super(id);
      setLevel(2);
      FILL_POLYGON = fill;
      this.color = color;
      setPoints(points);
   }

   public PolygonArtifact(String id, boolean fill, Color color, ConvexPolygon2d convexPolygon2d)
   {
      super(id);
      setLevel(2);
      FILL_POLYGON = fill;
      this.color = color;

      setPoints(convexPolygon2d);
   }

   public PolygonArtifact(String id, boolean fill, Color color, int level, ConvexPolygon2d convexPolygon2d)
   {
      super(id);
      setLevel(level);
      FILL_POLYGON = fill;
      this.color = color;

      setPoints(convexPolygon2d);
   }

   public PolygonArtifact(String id, boolean fill, Color color, BoundingBox2d boundingBox2d)
   {
      super(id);
      setLevel(2);
      FILL_POLYGON = fill;
      this.color = color;
      Point2d minPoint = new Point2d();
      boundingBox2d.getMinPoint(minPoint);
      Point2d maxPoint = new Point2d();
      boundingBox2d.getMaxPoint(maxPoint);
      Point2d leftUpper = new Point2d(minPoint.getX(), maxPoint.getY());
      Point2d rightLower = new Point2d(maxPoint.getX(), minPoint.getY());
      ArrayList<Point2d> points = new ArrayList<Point2d>();
      points.add(minPoint);
      points.add(leftUpper);
      points.add(maxPoint);
      points.add(rightLower);
      setPoints(points);
   }


   public void setPoints(List<Point2d> points)
   {
      this.points.clear();
      this.points.addAll(points);
   }

   public void setPoints(ConvexPolygon2d polygon)
   {
      this.points.clear();
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
         this.points.add(polygon.getVertex(i));
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
   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      ArrayList<Point2d> pointsCopy = new ArrayList<>(points);
      int nPoints = pointsCopy.size();
      int[] xPoints = new int[nPoints];
      int[] yPoints = new int[nPoints];

      if (nPoints == 1)
      {
         int x = Xcenter + (int) Math.round(pointsCopy.get(0).x * scaleFactor);
         int y = Ycenter - (int) Math.round(pointsCopy.get(0).y * scaleFactor);
         g.fillOval(x, y, 4, 4);
      }
      else
      {
         for (int i = 0; i < nPoints; i++)
         {
            if (pointsCopy.get(i) != null)
            {
               int x = Xcenter + (int) Math.round(pointsCopy.get(i).x * scaleFactor);
               int y = Ycenter - (int) Math.round(pointsCopy.get(i).y * scaleFactor);
               xPoints[i] = x;
               yPoints[i] = y;
            }
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
