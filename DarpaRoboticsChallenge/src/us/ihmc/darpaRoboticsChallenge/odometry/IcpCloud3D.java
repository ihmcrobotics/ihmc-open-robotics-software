package us.ihmc.darpaRoboticsChallenge.odometry;

import boofcv.struct.FastQueue;
import boofcv.struct.FastQueueArray_F64;
import georegression.fitting.MotionTransformPoint;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;
import org.ddogleg.nn.NearestNeighbor;
import org.ddogleg.nn.NnData;

import java.util.List;

/**
 * @author Peter Abeles
 */
public class IcpCloud3D {

   double maxDistance;
   int maxIterations;
   double convergenceTol;

   MotionTransformPoint<Se3_F64, Point3D_F64> motionAlg;

   NearestNeighbor<Point3D_F64> nn;

   FastQueue<Point3D_F64> ref = new FastQueue<Point3D_F64>(Point3D_F64.class,true);
   FastQueue<double[]> refArray = new FastQueueArray_F64(3);


   double[] work = new double[3];
   NnData<Point3D_F64> bestMatch = new NnData<Point3D_F64>();

   FastQueue<Point3D_F64> src = new FastQueue<Point3D_F64>(Point3D_F64.class,false);
   FastQueue<Point3D_F64> dst = new FastQueue<Point3D_F64>(Point3D_F64.class,false);

   Se3_F64 work0 = new Se3_F64();
   Se3_F64 total = new Se3_F64();

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

   public Se3_F64 getReferenceToCurrent() {
      return total;
   }
}
