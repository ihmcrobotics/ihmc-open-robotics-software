package us.ihmc.ihmcPerception.depthData;

import boofcv.struct.FastQueueArray_F64;
import georegression.fitting.MotionTransformPoint;
import georegression.fitting.se.MotionSe3PointSVD_F64;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Rodrigues_F64;
import georegression.transform.se.SePointOps_F64;
import org.ddogleg.nn.FactoryNearestNeighbor;
import org.ddogleg.nn.NearestNeighbor;
import org.ddogleg.nn.NnData;
import org.ddogleg.struct.FastQueue;

import java.util.ArrayList;
import java.util.List;

/**
 * Estimates the rigid body motion between two point clouds.  Guaranteed to find a locally optical solution, and will
 * most likely need a good initial estimate. The found motion is from reference to current.  The algorithm works by
 * associating features together based on the nearest-neighbor.  If the nearest-neighbor is too far away then no
 * association is made.  After a set of association points has been found the motion which minimizes the error between
 * the two sets is found.  The process is repeated until convergence.
 *
 * @author Peter Abeles
 */
public class IcpCloud3D {

   // tolerance for how close two points need to be before they are matched
   double maxDistance;
   // maximum number of iterations
   int maxIterations;
   // tolerance for change in translation between iterations
   double convergenceTol;

   // Estimations motion from two sets of 3D point clounds
   MotionTransformPoint<Se3_F64, Point3D_F64> motionAlg;

   // Finds the nearest neighbor.
   NearestNeighbor<SrcData> nn;

   FastQueue<SrcData> ref = new FastQueue<SrcData>(SrcData.class,true);
   FastQueue<double[]> refArray = new FastQueueArray_F64(3);

   // keep track of which reference data has been used.
   List<SrcData> usedRef = new ArrayList<SrcData>();

   // modified list of 'current' point cloud after applying the most recent estimate of camera motion
   // Much less numerical errors when applying the total transform as compared to compounding transforms
   FastQueue<Point3D_F64> currentModified = new FastQueue<Point3D_F64>(Point3D_F64.class,true);

   double[] work = new double[3];
   NnData<SrcData> bestMatch = new NnData<SrcData>();

   FastQueue<Point3D_F64> src = new FastQueue<Point3D_F64>(Point3D_F64.class,false);
   FastQueue<Point3D_F64> dst = new FastQueue<Point3D_F64>(Point3D_F64.class,false);

   Se3_F64 work0 = new Se3_F64();
   Se3_F64 total = new Se3_F64();

   // used to check for convergence
   Rodrigues_F64 rod = new Rodrigues_F64();

   // fraction of reference points which were matched with dst points
   double fitFraction;
   // fraction of dst points which were matched with referece points
   double fractionNearTemplate;
   int dstPointsWithNearestNeighbors = 0;

   /**
    * Constructor in which parameters and internal algorithms are specified
    *
    * @param maxDistance Maximum distance two points can be apart for them to be associated
    * @param maxIterations Maximum number of iterations.  Try 20
    * @param convergenceTol Tolerance for convergence.  Try 1e-12
    * @param motionAlg Computes motion between point clouds
    * @param nn Nearest-Neighbor
    */
   public IcpCloud3D(double maxDistance, int maxIterations, double convergenceTol,
                     MotionTransformPoint<Se3_F64, Point3D_F64> motionAlg, NearestNeighbor<SrcData> nn)
   {
      this.maxDistance = maxDistance;
      this.maxIterations = maxIterations;
      this.convergenceTol = convergenceTol;
      this.motionAlg = motionAlg;
      this.nn = nn;

      nn.init(3);
   }

   /**
    * Constructor where default algorithms are used for motion estimation and nearest-neighbor search
    * @param maxDistance Maximum distance two points can be apart for them to be associated
    * @param maxIterations Maximum number of iterations.  Try 20
    * @param convergenceTol Tolerance for convergence.  Try 1e-12
    */
   public IcpCloud3D(double maxDistance, int maxIterations, double convergenceTol )
   {
      this.maxDistance = maxDistance;
      this.maxIterations = maxIterations;
      this.convergenceTol = convergenceTol;
      motionAlg = new MotionSe3PointSVD_F64();
      nn = FactoryNearestNeighbor.kdtree();

      nn.init(3);
   }

   /**
    * Specify the reference point cloud.  This must be called before {@link #setCurrent(java.util.List)} and is more
    * efficient if it is only called once for several calls to {@link #setCurrent(java.util.List)}.
    * @param reference List of points in reference point cloud
    */
   public void setReference( List<Point3D_F64> reference ) {
      ref.reset();
      refArray.reset();

      for( Point3D_F64 p : reference ) {
         ref.grow().set(p);
         double[] a = refArray.grow();
         a[0] = p.x;
         a[1] = p.y;
         a[2] = p.z;
      }

      nn.setPoints(refArray.toList(), ref.toList());
   }

   /**
    * Passes in 'current' point cloud and computes motion.
    * @param current List of points in 'current' set
    * @return true if motion was successfully found
    */
   public boolean setCurrent( List<Point3D_F64> current ) {

      currentModified.reset();
      for( Point3D_F64 p : current ) {
         currentModified.grow().set(p);
      }

      total.reset();
      for( int i = 0; i < maxIterations; i++ ) {
         usedRef.clear();

         src.reset();
         dst.reset();

         dstPointsWithNearestNeighbors=0;
         // find nearest-neighbors
         for( Point3D_F64 p : currentModified.toList() ) {

            work[0] = p.x;
            work[1] = p.y;
            work[2] = p.z;

            if( nn.findNearest(work,maxDistance, bestMatch) ) {
               usedRef.add(bestMatch.data);
               src.add( bestMatch.data.point );
               dst.add( p );
               dstPointsWithNearestNeighbors++;
            }
         }

         if( !motionAlg.process(src.toList(),dst.toList()) )
            return false;

         // apply found motion
         total.concat(motionAlg.getTransformSrcToDst(),work0);
         total.set(work0);

         Se3_F64 found = motionAlg.getTransformSrcToDst();
         double change = found.getT().normSq();
         ConvertRotation3D_F64.matrixToRodrigues(found.getR(), rod);

         if( change < convergenceTol && Math.abs(rod.theta) < convergenceTol )
            break;

         // apply found transform
         for( int j = 0; j < current.size(); j++ ) {
            Point3D_F64 p = current.get(j);
            SePointOps_F64.transformReverse(total,p,currentModified.get(j));
         }
      }

      computeQualityOfFit();

      return true;
   }

   /**
    * Computes the fraction of reference points which are used to compute the final motion
    */
   private void computeQualityOfFit()
   {
      // mark all points as not being used
      for( int j = 0; j < ref.size(); j++ ) {
         ref.get(j).hit = false;
      }

      // make used points
      for( int j = 0; j < usedRef.size(); j++ ) {
         usedRef.get(j).hit = true;
      }

      // count total points used
      int count = 0;
      for( int i = 0; i < ref.size(); i++ ) {
         if( ref.get(i).hit )
            count++;
      }

      fitFraction = count/(double)ref.size();
      fractionNearTemplate = (double)dstPointsWithNearestNeighbors/((double)currentModified.size());
   }


   public double getFitFraction()
   {
      return fitFraction;
   }
   public double getFractionNearTemplate()
   {
      return fractionNearTemplate;
   }

   /**
    * Found transform from reference to current point cloud frames.  Only has valid results if {@link #setCurrent(java.util.List)}
    * returns true.
    * @return Rigid body motion
    */
   public Se3_F64 getReferenceToCurrent() {
      return total;
   }

   public static class SrcData
   {
      // if
      boolean hit;
      Point3D_F64 point = new Point3D_F64();

      public void set( Point3D_F64 p ) {
         point.set(p);
      }
   }
}
