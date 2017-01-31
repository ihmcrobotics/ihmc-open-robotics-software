package us.ihmc.graphicsDescription.plotting.artifact;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.geometry.ConvexPolygon2d;

public class PolygonArtifact extends Artifact
{
   private ArrayList<Point2d> points = new ArrayList<Point2d>();
   private boolean FILL_POLYGON = false;
   
   private final ConvexPolygon2d tempPolygon = new ConvexPolygon2d();
   private final Point2d tempPoint = new Point2d();
   private final Vector2d tempRadii = new Vector2d();

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

   @Override
   public String getID()
   {
      return id;
   }

   /**
    * Must provide a draw method for plotter to render artifact
    */
   @Override
   public void draw(Graphics2DAdapter graphics)
   {
      if (points.isEmpty())
         return;
      
      ArrayList<Point2d> pointsCopy = new ArrayList<>(points);
      
      if (pointsCopy.size() == 1)
      {
         tempPoint.set(pointsCopy.get(0).getX(), pointsCopy.get(0).getY());
         tempRadii.set(4.0, 4.0);
         graphics.drawOvalFilled(tempPoint, tempRadii);
      }
      else
      {
         graphics.setColor(color);
         tempPolygon.setAndUpdate(pointsCopy, pointsCopy.size());

         if (FILL_POLYGON)
         {
            graphics.drawPolygonFilled(tempPolygon);
         }
         else
         {
            graphics.drawPolygon(tempPolygon);
         }
      }
   }

   @Override
   public void drawLegend(Plotter2DAdapter graphics, Point2d origin)
   {
      graphics.drawString(graphics.getScreenFrame(), "Polygon", origin);
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
