package us.ihmc.darpaRoboticsChallenge.odometry;

import boofcv.struct.FastQueue;
import boofcv.struct.FastQueueArray_F64;
import georegression.fitting.MotionTransformPoint;
import georegression.fitting.se.MotionSe3PointSVD_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;
import org.ddogleg.nn.FactoryNearestNeighbor;
import org.ddogleg.nn.NearestNeighbor;
import org.ddogleg.nn.NnData;

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
   NearestNeighbor<Point3D_F64> nn;

   FastQueue<Point3D_F64> ref = new FastQueue<Point3D_F64>(Point3D_F64.class,true);
   FastQueue<double[]> refArray = new FastQueueArray_F64(3);


   double[] work = new double[3];
   NnData<Point3D_F64> bestMatch = new NnData<Point3D_F64>();

   FastQueue<Point3D_F64> src = new FastQueue<Point3D_F64>(Point3D_F64.class,false);
   FastQueue<Point3D_F64> dst = new FastQueue<Point3D_F64>(Point3D_F64.class,false);

   Se3_F64 work0 = new Se3_F64();
   Se3_F64 total = new Se3_F64();

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
                     MotionTransformPoint<Se3_F64, Point3D_F64> motionAlg, NearestNeighbor<Point3D_F64> nn)
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

      total.reset();
      for( int i = 0; i < maxIterations; i++ ) {
         src.reset();
         dst.reset();

         for( Point3D_F64 p : current ) {

            work[0] = p.x;
            work[1] = p.y;
            work[2] = p.z;

            if( nn.findNearest(work,maxDistance, bestMatch) ) {
               src.add( bestMatch.data );
               dst.add( p );
            }
         }

         if( !motionAlg.process(src.toList(),dst.toList()) )
            return false;

         Se3_F64 found = motionAlg.getMotion();
         double change = found.getT().normSq();

         System.out.println("change "+change);
         if( change < convergenceTol )
            break;

         // apply found transform
         for( Point3D_F64 p : current ) {
            SePointOps_F64.transformReverse(found,p,p);
         }

         total.concat(motionAlg.getMotion(),work0);
         total.set(work0);
      }

      return true;
   }

   /**
    * Found transform from reference to current point cloud frames.  Only has valid results if {@link #setCurrent(java.util.List)}
    * returns true.
    * @return Rigid body motion
    */
   public Se3_F64 getReferenceToCurrent() {
      return total;
   }
}
