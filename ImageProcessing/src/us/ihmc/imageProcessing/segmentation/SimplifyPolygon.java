package us.ihmc.imageProcessing.segmentation;

import georegression.struct.point.Point2D_I32;

import org.ddogleg.struct.FastQueue;

import boofcv.struct.PointIndex_I32;

/**
 * @author Peter Abeles
 */
public class SimplifyPolygon {

   public static FastQueue<PointIndex_I32> process(  FastQueue<PointIndex_I32> vertexes) {
      if( vertexes.size() < 4 )
         return vertexes;

      FastQueue<PointIndex_I32> ret = new FastQueue<PointIndex_I32>(PointIndex_I32.class,true);

      int N = vertexes.size();

      for( int i = 1; i < N+1; i++ ) {
         Point2D_I32 a = vertexes.get(i-1);
         Point2D_I32 b = vertexes.get(i%N);
         Point2D_I32 c = vertexes.get((i+1)%N);

         double distAC = a.distance(c);
         double distABC = a.distance(b) + b.distance(c);

         if( distAC >= distABC*0.1 ) {
            ret.grow().set(b);
         } else {
            System.out.println("simplified");
         }
      }

      return ret;
   }
}
