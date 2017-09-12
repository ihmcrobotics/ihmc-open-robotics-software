package us.ihmc.imageProcessing.segmentation;

import boofcv.alg.filter.binary.Contour;
import boofcv.alg.shapes.ShapeFittingOps;
import boofcv.alg.shapes.polyline.SplitMergeLineFitLoop;
import boofcv.struct.PointIndex_I32;
import boofcv.struct.image.InterleavedS32;
import georegression.geometry.UtilLine2D_F64;
import georegression.metric.UtilAngle;
import georegression.struct.line.LineParametric2D_F64;
import georegression.struct.line.LinePolar2D_F64;
import georegression.struct.line.LineSegment2D_F64;
import georegression.struct.line.LineSegment2D_I32;
import georegression.struct.point.Point2D_I32;
import org.ddogleg.sorting.QuickSelect;
import org.ddogleg.struct.FastQueue;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Peter Abeles
 */
public class ClassifyRoadLines {

   InterleavedS32 labeled;

   SplitMergeLineFitLoop polygonFit = new SplitMergeLineFitLoop(5,0.1,50);

   int minimumLengthSq = 45*45;

   FastQueue<LineSegment2D_I32> candidates = new FastQueue<LineSegment2D_I32>(LineSegment2D_I32.class,true);

   FastQueue<PointIndex_I32> vertexes = new FastQueue<PointIndex_I32>(PointIndex_I32.class,true);

   LineSegment2D_I32 left;
   LineSegment2D_I32 right;

   Contour largest;

   public boolean process( InterleavedS32 labeled , List<Contour> contours ) {

      this.labeled = labeled;

      int largestCount = 0;
      largest = null;
      for( Contour c : contours ) {
         if( c.external.size() > largestCount ) {
            largestCount = c.external.size();
            largest = c;
         }
      }
      if( largest == null )
         return false;

      polygonFit.process(largest.external);
      vertexes.reset();
      ShapeFittingOps.indexToPointIndex(largest.external, polygonFit.getSplits(), vertexes);
//      vertexes = SimplifyPolygon.process(vertexes);

      findCandidates(vertexes);
//      combineLines();

      if( candidates.size < 2 )
         return false;

      selectRoadLines();

      return true;
   }

   private void findCandidates( FastQueue<PointIndex_I32> vertexes ) {
      candidates.reset();

      for( int i = 0; i < vertexes.size-1; i++ ) {
         Point2D_I32 a = vertexes.get(i);
         Point2D_I32 b = vertexes.get(i+1);

         if( isCandidate(a,b) ) {
            candidates.grow().set(a,b);
         }
      }
      Point2D_I32 a = vertexes.get(vertexes.size-1);
      Point2D_I32 b = vertexes.get(0);

      if( isCandidate(a,b) ) {
         candidates.grow().set(a,b);
      }
   }

   private boolean isCandidate( Point2D_I32 a , Point2D_I32 b ) {
      if( (a.y >= labeled.height-20 || b.y >= labeled.height-20) &&
      a.distance2(b) < minimumLengthSq )
         return false;

      return !isAgainstBoundary(a,b);
   }

   private boolean isAgainstBoundary( Point2D_I32 a , Point2D_I32 b ) {

      int r = 20;

      int w = labeled.width-r;
      int h = labeled.height-r;

      if( a.x < r && b.x < r )
         return true;
      if( a.x >= w && b.x >= w )
         return true;
      if( a.y < r && b.y < r )
         return true;
      if( a.y >= h && b.y >= h )
         return true;

      return false;
   }

   private void combineLines() {
      // Convert lines into polar notation
      List<LinePolar2D_F64> listPolar = new ArrayList<LinePolar2D_F64>();
      LineParametric2D_F64 parametric = new LineParametric2D_F64();
      LineSegment2D_F64 segment = new LineSegment2D_F64();
      for( int i = 0; i < candidates.size; i++ ) {
         LineSegment2D_I32 s = candidates.get(i);
         segment.set(s.a.x,s.a.y,s.b.x,s.b.y);
         UtilLine2D_F64.convert(segment,parametric);
         LinePolar2D_F64 polar = UtilLine2D_F64.convert(parametric, (LinePolar2D_F64) null);
         if( polar.distance < 0 ) {
            polar.distance = -polar.distance;
            polar.angle = UtilAngle.bound(polar.angle + Math.PI);
         }
         listPolar.add(polar);
         System.out.println("polar " + polar.angle + "  " + polar.distance);
      }

      List<LineSegment2D_I32> listSegments = new ArrayList<LineSegment2D_I32>();
      listSegments.addAll(candidates.toList());

      candidates.reset();
      List<LineSegment2D_I32> similar = new ArrayList<LineSegment2D_I32>();
      for( int i = 0; i < listPolar.size(); i++ ) {

         similar.clear();
         LinePolar2D_F64 a = listPolar.get(i);

         similar.add(listSegments.get(i));

         for( int j = i+1; j < listPolar.size(); ) {
            LinePolar2D_F64 b = listPolar.get(j);

            if( Math.abs(a.distance-b.distance) < 20 && UtilAngle.dist(a.angle,b.angle) < 0.2 ) {
               listPolar.remove(j);
               similar.add(listSegments.remove(j));
            } else {
               j++;
            }
         }
         System.out.println("similar.size = "+similar.size());
         combine(similar,candidates.grow());
      }
   }

   private void combine( List<LineSegment2D_I32> lines , LineSegment2D_I32 result ) {
      // select the two most distance end points
      List<Point2D_I32> points = new ArrayList<Point2D_I32>();
      for( LineSegment2D_I32 s : lines ) {
         points.add(s.a);
         points.add(s.b);
      }

      int bestDistance = 0;
      Point2D_I32 pointA=null;
      Point2D_I32 pointB=null;

      for( int i = 0; i < points.size(); i++ ) {
         Point2D_I32 a = points.get(i);

         for( int j = i+1; j < points.size(); j++ ) {
            Point2D_I32 b = points.get(j);

            int distSq = a.distance2(b);
            if( distSq > bestDistance ) {
               bestDistance = distSq;
               pointA = a;
               pointB = b;
            }
         }
      }

      result.set(pointA, pointB);
   }


   /**
    * Select the two lines with the greatest change in Y and the closest to the bottom
    */
   private void selectRoadLines() {
      LineSegment2D_I32 a,b;
      if( candidates.size == 2 ) {
         a = candidates.get( 0 );
         b = candidates.get( 1 );
      } else {
         double score[] = new double[ candidates.size ];
         int indexes[] = new int[ score.length ];

         for( int i = 0; i < score.length; i++ ) {

            LineSegment2D_I32 l = candidates.get(i);

            int maxY = Math.max(l.a.y , l.b.y);
            score[i] = -(Math.abs(l.a.y - l.b.y));// - (labeled.height - maxY));
         }

         QuickSelect.selectIndex(score,2,score.length,indexes);

         a = candidates.get( indexes[0] );
         b = candidates.get( indexes[1] );
      }

      if( bottomX(a) < bottomX(b) ) {
         left = a;
         right = b;
      } else {
         left = b;
         right = a;
      }
   }

   private int bottomX( LineSegment2D_I32 line ) {
      if( line.a.y < line.b.y ) {
         return line.a.x;
      } else {
         return line.b.x;
      }
   }

   public LineSegment2D_I32 getLeft() {
      return left;
   }

   public LineSegment2D_I32 getRight() {
      return right;
   }

   public FastQueue<PointIndex_I32> getVertexes() {
      return vertexes;
   }

   public FastQueue<LineSegment2D_I32> getCandidates() {
      return candidates;
   }

   public Contour getLargest() {
      return largest;
   }

   public SplitMergeLineFitLoop getPolygonFit() {
      return polygonFit;
   }
}
