package us.ihmc.imageProcessing.segmentation;

import boofcv.struct.FastQueue;
import boofcv.struct.GrowQueue_I32;
import boofcv.struct.PointIndex_I32;
import georegression.metric.Intersection2D_F64;
import georegression.struct.line.LineSegment2D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.shapes.Polygon2D_F64;

import java.util.Arrays;
import java.util.List;

/**
 * @author Peter Abeles
 */
public class SelectRobotPath {

   GrowQueue_I32 intersections = new GrowQueue_I32();


   FastQueue<LineSegment2D_F64> lines = new FastQueue<LineSegment2D_F64>(LineSegment2D_F64.class,true);
   FastQueue<WayPoint> waypoints = new FastQueue<WayPoint>(WayPoint.class,true);

   Polygon2D_F64 polyF;

   int width;
   int height;

   public void setImageSize(int width, int height) {
      this.width = width;
      this.height = height;
   }

   public void process( List<PointIndex_I32> poly ) {

      convertToF(poly);
      convert(poly);

      LineSegment2D_F64 horizontal = new LineSegment2D_F64();

      Point2D_F64 where = new Point2D_F64();

      waypoints.reset();

      for( int y = 0; y < height; y++  ) {
         horizontal.set(0, y, width, y);

         intersections.reset();
         for( LineSegment2D_F64 l : lines.toList() ) {
            // horizontal lines will have an infinite number of intersections
            if( l.a.y == l.b.y && (int)l.a.y == y ) {

            } else if( Intersection2D_F64.intersection(horizontal,l,where) != null) {
               // avoid double counting at end points
               intersections.add((int)where.x);
            }
         }

         Arrays.sort(intersections.data, 0,intersections.size);

         int largest = 0;
         int locationX = -1;
         Point2D_F64 tmp = new Point2D_F64();
         for( int i = 0; i < intersections.size-1; i++ ) {
            int x0 = intersections.get(i);
            int x1 = intersections.get(i+1);

            tmp.x = (x0+x1)/2.0;
            tmp.y = y;

            if( Intersection2D_F64.containConvex(polyF,tmp) ) {
               int width = x1-x0;
               if( width > largest ) {
                  largest = width;
                  locationX = (x0+x1)/2;
               }
            }
         }

         if( locationX != - 1 ) {
            WayPoint p = waypoints.grow();
            p.x = locationX;
            p.y = y;

            int x0 = intersections.get(0);
            int x1 = intersections.get(intersections.size-1);
            if( x0 != 0 && x1 != width-1 ) {
               p.fraction = largest/(double)(x1-x0);
            } else {
               p.fraction = 0;
            }
         }
      }
   }

   private void convertToF( List<PointIndex_I32> poly ) {
      polyF = new Polygon2D_F64(poly.size());
      for( int i = 0; i < poly.size(); i++ ) {
         PointIndex_I32 p = poly.get(i);
         polyF.vertexes[i].set(p.x,p.y);
      }
   }

   private void convert( List<PointIndex_I32> poly ) {
      lines.reset();
      for( int i = 0; i < poly.size()-1; i++ ) {
         PointIndex_I32 a = poly.get(i);
         PointIndex_I32 b = poly.get(i + 1);

         LineSegment2D_F64 s = lines.grow();

         s.a.set(a.x,a.y);
         s.b.set(b.x, b.y);
      }

      PointIndex_I32 a = poly.get(poly.size()-1);
      PointIndex_I32 b = poly.get(0);

      LineSegment2D_F64 s = lines.grow();

      s.a.set(a.x,a.y);
      s.b.set(b.x,b.y);
   }

   public FastQueue<WayPoint> getWaypoints() {
      return waypoints;
   }

   public static class WayPoint
   {
      public int x;
      public int y;
      public double fraction;
   }

}
