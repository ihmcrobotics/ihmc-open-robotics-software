package us.ihmc.imageProcessing.segmentation;

import georegression.geometry.UtilPoint2D_I32;
import georegression.metric.Distance2D_F64;
import georegression.struct.line.LineParametric2D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point2D_I32;

import java.util.ArrayList;
import java.util.List;

import org.ddogleg.struct.FastQueue;

/**
 * @author Peter Abeles
 */
public class FitPolyLineToUnorderedPoints
{

   int distanceSqTol;

   LineParametric2D_F64 line = new LineParametric2D_F64();

   List<Point2D_I32> polyline = new ArrayList<Point2D_I32>();

   public FitPolyLineToUnorderedPoints(int distanceSqTol)
   {
      this.distanceSqTol = distanceSqTol;
   }

   public void process( FastQueue<Point2D_I32> points ) {
      // find centroid
      Point2D_I32 center = UtilPoint2D_I32.mean(points.toList(), null);

      // look for two extreme points
      Point2D_I32 a = findFarthest(points,center);
      Point2D_I32 b = findFarthest(points,a);

      int middle = split(points,a,b);

      polyline.clear();
      if( middle != -1 ) {
         polyline.add(a);
         polyline.add(points.get(middle));
         polyline.add(b);
      } else {
         polyline.add(a);
         polyline.add(b);
      }
   }

   private Point2D_I32 findFarthest( FastQueue<Point2D_I32> points , Point2D_I32 a ) {
      Point2D_I32 best = null;
      int bestDistance = 0;

      for( int i = 0; i < points.size(); i++ ) {
         Point2D_I32 p = points.get(i);

         int dist = p.distance2(a);
         if( dist > bestDistance ) {
            bestDistance = dist;
            best = p;
         }
      }

      return best;
   }

   private int split( FastQueue<Point2D_I32> points , Point2D_I32 a , Point2D_I32 b ) {
      line.p.set(a.x,a.y);
      line.slope.set(b.x-a.x,b.y-a.x);

      double maxDistanceSq = 0;
      int maxIndex = -1;

      Point2D_F64 x = new Point2D_F64();

      for( int i = 0; i < points.size(); i++ ) {
         Point2D_I32 p = points.get(i);
         x.set(p.x,p.y);

         double d = Distance2D_F64.distanceSq(line,x);
         if( d > maxDistanceSq ) {
            maxDistanceSq = d;
            maxIndex = i;
         }
      }

      if( maxDistanceSq > distanceSqTol ) {
         return maxIndex;
      }
      return -1;
   }
}
