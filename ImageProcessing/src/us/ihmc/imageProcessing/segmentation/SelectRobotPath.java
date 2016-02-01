package us.ihmc.imageProcessing.segmentation;

import georegression.metric.Intersection2D_F64;
import georegression.struct.line.LineSegment2D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.shapes.Polygon2D_F64;

import java.util.Arrays;
import java.util.List;

import org.ddogleg.struct.FastQueue;
import org.ddogleg.struct.GrowQueue_I32;

import boofcv.struct.PointIndex_I32;

/**
 * @author Peter Abeles
 */
public class SelectRobotPath {

   double fracSwitch = 0.75;

   GrowQueue_I32 intersections = new GrowQueue_I32();


   FastQueue<LineSegment2D_F64> lines = new FastQueue<LineSegment2D_F64>(LineSegment2D_F64.class,true);
   FastQueue<WayPoint> waypoints = new FastQueue<WayPoint>(WayPoint.class,true);

   LineSegment2D_F64 line = new LineSegment2D_F64();
   Point2D_F64 where = new Point2D_F64();

   Polygon2D_F64 polyF;

   // y-coordinate of highest point in the polygon
   int highestPoint;

   int width;
   int height;

   public void setImageSize(int width, int height) {
      this.width = width;
      this.height = height;
   }

   public void process( List<PointIndex_I32> poly ) {

      // top of image is 0 and bottom is height
      highestPoint = height;
      for( PointIndex_I32 p : poly ) {
         if( p.y < highestPoint )
            highestPoint = p.y;
      }

      convertToF(poly);
      convert(poly);

      waypoints.reset();

      verticalSearch();
   }

   private void verticalSearch() {

      int prevX0 = 0;
      int prevX1 = width;

      for( int y = height-20; y >= 0; y--  ) {
         line.set(0, y, width, y);

         // ----- Find intersections of the polygon against this horizontal line
         intersections.reset();
         for( LineSegment2D_F64 l : lines.toList() ) {
            // horizontal lines will have an infinite number of intersections
            if( l.a.y == l.b.y && (int)l.a.y == y ) {

            } else if( Intersection2D_F64.intersection(line, l, where) != null) {
               // avoid double counting at end points
               intersections.add((int)where.x);
            }
         }

         // ------- Find largest interior segment
         Arrays.sort(intersections.data, 0, intersections.size);

         int largest = 0;
         int locationX = -1;
         int largestIndex = 0;
         Point2D_F64 tmp = new Point2D_F64();
         for( int i = 0; i < intersections.size-1; i++ ) {
            int x0 = intersections.get(i);
            int x1 = intersections.get(i+1);

            tmp.x = (x0+x1)/2.0;
            tmp.y = y;

            if( Intersection2D_F64.containConcave(polyF,tmp) ) {
               int width = x1-x0;
               if( width > largest && intersection(prevX0,prevX1,x0,x1) ) {
                  largest = width;
                  locationX = (x0+x1)/2;
                  largestIndex = i;
               }
            } else {
//               System.out.println("  not contained "+tmp.x);
//               Intersection2D_F64.containConcave(polyF,tmp);
            }
         }
//         System.out.println(" y = "+y+" num intersections "+intersections.size+"  largest "+largest);

         if( locationX != - 1 ) {
            int x0 = intersections.get(largestIndex);
            int x1 = intersections.get(largestIndex+1);

            WayPoint p = waypoints.grow();
            p.x = locationX;
            p.y = y;

            int leftMost = intersections.get(0);
            int rightMost = intersections.get(intersections.size-1);
            if( leftMost != 0 && rightMost != width-1 ) {
               p.fraction = largest/(double)(rightMost-leftMost);
            } else {
               p.fraction = 0;
            }

            int dir = checkSwitchHorizontal(p.x,p.y,x0,x1);
            if( dir != 0 ) {
               System.out.println("switch "+dir);
               horizontalSearch(p.x, p.y, dir);
               break;
            }
//            System.out.println(" prev "+prevX0+" "+prevX1+"  curr "+x0+" "+x1);
            prevX0 = x0;
            prevX1 = x1;
         } else {
//            System.out.println("No matches");
         }
      }
   }

   private boolean intersection( int a0 , int a1 , int b0 , int b1 ) {
      if( b0 >= a0 ) {
         if( b0 <= a1 )
            return true;
      } else if( b1 >= a0 )
         return true;
      return false;
   }

   /**
    *
    * @param x Selected mid point
    * @param y Selected mid point
    * @param x0 left side of passable region
    * @param x1 right side of passable region
    * @return true if it should switch to horizontal
    */
   private int checkSwitchHorizontal( int x , int y , int x0 , int x1 ) {

      double polyX0 = x0;
      double polyX1 = x1;

      for( int i = 0; i < polyF.size(); i++ ) {
         Point2D_F64 p = polyF.vertexes.get(i);
         if( p.y <= y ) {
            if( polyX0 > p.x )
               polyX0 = p.x;
            if( polyX1 < p.x )
               polyX1 = p.x;
         }
      }

      int distY = y-highestPoint;
      int widthSegment = x1-x0;
      double widthPoly = polyX1-polyX0;


      if( widthSegment < widthPoly )
         System.out.println(" segment "+widthSegment+"  poly "+widthPoly+" distY "+distY);
      if( widthSegment > distY && widthSegment < widthPoly*fracSwitch ) {
//         System.out.println(" switch horizontal");
         if( polyX1 - x > x - polyX0 )
            return 1;
         else
            return -1;
      }
      return 0;
   }

   private void horizontalSearch( int x_start , int bottomY , int x_dir ) {
      int length = x_dir > 0 ? width-x_start : x_start;


      for( int loc = 0; loc < length; loc++ ) {
         int x = x_start + x_dir * loc;
         line.set(x, height, x, 0);

         // ----- Find intersections of the polygon against this horizontal line
         intersections.reset();
         for( LineSegment2D_F64 l : lines.toList() ) {
            // vertical lines will have an infinite number of intersections
            if( l.a.x == l.b.x && (int)l.a.x == x ) {

            } else if( Intersection2D_F64.intersection(line, l, where) != null) {
               intersections.add((int)where.y);
            }
         }

         // ------- Find largest interior segment
         Arrays.sort(intersections.data, 0, intersections.size);

         int largest = 0;
         int locationY = -1;

         Point2D_F64 tmp = new Point2D_F64();
         for( int i = 0; i < intersections.size-1; i++ ) {
            int y0 = intersections.get(i);
            int y1 = intersections.get(i+1);

            if( y0 > bottomY ) y0 = bottomY;
            if( y1 > bottomY ) y1 = bottomY;

            tmp.x = x;
            tmp.y = (y0+y1)/2.0;

            if( Intersection2D_F64.containConcave(polyF,tmp) ) {
               int width = y1-y0;
               if( width > largest ) {
                  largest = width;
                  locationY = (y0+y1)/2;
               }
            }
         }

//         if( x_dir < 0 )
//            System.out.println("  x = "+x+" locationY "+locationY+"  intersections.size "+intersections.size+" bottomY "+bottomY);

         if( locationY != - 1 ) {
            WayPoint p = waypoints.grow();
            p.y = locationY;
            p.x = x;
         } else {
            break;
         }
      }

   }

   private void convertToF( List<PointIndex_I32> poly ) {
      polyF = new Polygon2D_F64(poly.size());
      for( int i = 0; i < poly.size(); i++ ) {
         PointIndex_I32 p = poly.get(i);
         polyF.vertexes.data[i].set(p.x,p.y);
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
