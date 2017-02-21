package us.ihmc.graphicsDescription.plotting.artifact;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.geometry.ConvexPolygon2d;

public class PolygonArtifact extends Artifact
{
   private ArrayList<Point2DReadOnly> points = new ArrayList<>();
   private boolean FILL_POLYGON = false;
   
   private final ConvexPolygon2d tempPolygon = new ConvexPolygon2d();
   private final Point2D tempPoint = new Point2D();
   private final Vector2D tempRadii = new Vector2D();

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

   public PolygonArtifact(String id, boolean fill, Color color, List<Point2D> points)
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
      Point2D minPoint = new Point2D();
      boundingBox2d.getMinPoint(minPoint);
      Point2D maxPoint = new Point2D();
      boundingBox2d.getMaxPoint(maxPoint);
      Point2D leftUpper = new Point2D(minPoint.getX(), maxPoint.getY());
      Point2D rightLower = new Point2D(maxPoint.getX(), minPoint.getY());
      ArrayList<Point2D> points = new ArrayList<Point2D>();
      points.add(minPoint);
      points.add(leftUpper);
      points.add(maxPoint);
      points.add(rightLower);
      setPoints(points);
   }

   public void setPoints(List<Point2D> points)
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

   public void addPoint(Point2D point)
   {
      this.points.add(point);
   }

   public void clearAllPoints()
   {
      this.points.clear();
   }

   public ArrayList<Point2DReadOnly> getPoints()
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
      
      ArrayList<Point2DReadOnly> pointsCopy = new ArrayList<>(points);
      
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
   public void drawLegend(Plotter2DAdapter graphics, Point2D origin)
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
