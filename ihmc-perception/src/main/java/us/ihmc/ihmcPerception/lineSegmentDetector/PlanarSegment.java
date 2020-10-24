package us.ihmc.ihmcPerception.lineSegmentDetector;

import us.ihmc.euclid.tuple2D.Point2D;

import java.util.ArrayList;
import java.util.Collections;

public class PlanarSegment
{
   private int regionId;
   private double area;
   private ArrayList<Point2D> vertices;
   private Point2D centroid;

   public PlanarSegment(int id)
   {
      this.regionId = id;
      this.vertices = new ArrayList<>();
   }

   public double updateArea()
   {
      int j = vertices.size() - 1;
      for (int i = 0; i < vertices.size(); i++)
      {
         Point2D vi = vertices.get(i);
         Point2D vj = vertices.get(j);
         area += (vj.getX() + vi.getX()) * (vj.getY() - vi.getY());
      }
      return area;
   }

   public void updateOrder()
   {
      Collections.sort(vertices, (a, b) ->
      {
         double a1 = (Math.toDegrees(Math.atan2(a.getX() - centroid.getX(), a.getY() - centroid.getY())) + 360) % 360;
         double a2 = (Math.toDegrees(Math.atan2(b.getX() - centroid.getX(), b.getY() - centroid.getY())) + 360) % 360;
         return (int) (a1 - a2);
      });
   }

   public void setCentroid(Point2D centroid)
   {
      this.centroid = centroid;
   }

   public int getRegionId()
   {
      return regionId;
   }

   public double getArea()
   {
      return area;
   }

   public ArrayList<Point2D> getVertices()
   {
      return vertices;
   }

   public Point2D getCentroid()
   {
      return centroid;
   }
}
