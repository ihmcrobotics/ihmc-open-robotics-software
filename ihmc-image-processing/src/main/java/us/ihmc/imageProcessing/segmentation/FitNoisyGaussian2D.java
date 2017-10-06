package us.ihmc.imageProcessing.segmentation;

import georegression.struct.point.Point2D_F64;

import org.ddogleg.sorting.QuickSelect;
import org.ddogleg.struct.FastQueue;
import org.ddogleg.struct.GrowQueue_F64;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CovarianceOps;

/**
 * Fits a gaussian to noisy 2D data.  First the distribution is computed from all points, then the points with the
 * largest chi-square value are removed and the distribution recomputed.  This process is repeated until it converges
 * or the maximum number of iterations has been exceeded.
 *
 * @author Peter Abeles
 */
public class FitNoisyGaussian2D {

   // 95% containment for chi-sq with 2-dof
   private double chisq95 = 5.99;

   // maximum number of iterations
   private int maxIterations;
   // tolerance for convergence.
   private double tol;
   // fraction of included points
   private double fraction;

   private FastQueue<Point2D_F64> points = new FastQueue<Point2D_F64>(Point2D_F64.class,true);
   private FastQueue<Point2D_F64> active = new FastQueue<Point2D_F64>(Point2D_F64.class,false);

   // storage for chi-square values
   private GrowQueue_F64 chisqs = new GrowQueue_F64();
   private GrowQueue_F64 workArray = new GrowQueue_F64();

   private Gaussian2D_F64 found = new Gaussian2D_F64();

   private DenseMatrix64F Q = new DenseMatrix64F(2,2);


   public FitNoisyGaussian2D(int maxIterations, double tol, double fraction) {
      this.maxIterations = maxIterations;
      this.tol = tol;
      this.fraction = fraction;
   }

   public void reset() {
      points.reset();
   }

   public void addPoint( double hue , double saturation ) {
      if( Double.isNaN(hue) )
         return;

      points.grow().set(hue,saturation);
   }

   public void process() {

      // initial set contains all points
      active.reset();
      for( int i = 0; i < points.size(); i++ )
         active.add( points.get(i) );

      computeGaussian();
      computeInverseCovariance();

      // recompute until convergence
      double oldX = found.x;
      double oldY = found.y;

      for( int i = 0; i < maxIterations; i++ ) {
         if( selectPoints() )
            break;
         computeGaussian();
         computeInverseCovariance();

         double delta = Math.max( Math.abs(oldX - found.x), Math.abs(oldY - found.y) );
         System.out.println("FitGaussian delta "+delta);
         if( delta < tol )
            break;
         oldX = found.x;
         oldY = found.y;
      }

   }

   /**
    * Selects specific fraction of points to be the active list of points based on their chi-square score.
    *
    * @return Returns true if the current model is a good fit to a Gaussian and no more iterations are needed
    */
   private boolean selectPoints() {

      // compute a list of chi-square values
      chisqs.reset();

      int numContained = 0;
      for( int i = 0; i < points.size; i++ ) {
         Point2D_F64 p = points.get(i);
         double chisq = found.chisq(p.x, p.y);
         if( chisq <= chisq95)
            numContained++;
         chisqs.push(chisq);
      }

      // See if it's a good fit for a gaussian
      if( numContained <= 0.96*points.size ) {
         System.out.println("  Fit Gaussian:  "+numContained/(double)points.size);
         return true;
      }

      // make a copy since it will be modified
      workArray.resize(chisqs.size);
      System.arraycopy(chisqs.data,0,workArray.data,0,chisqs.size);

      // select a threshold which will contain the specified fraction of points
      double threshold = QuickSelect.select(workArray.data, (int) (chisqs.size * fraction), chisqs.size);
      // TODO is a more intelligent way to identify outliers to a Gaussian?

      System.out.println("Containment threshold = "+threshold);

      active.reset();
      for( int i = 0; i < points.size; i++ ) {
         if( chisqs.get(i) <= threshold ) {
            active.add( points.get(i));
         }
      }

      return false;
   }

   private void computeGaussian() {
      double sumX=0,sumY=0;

      for( int i = 0; i < active.size; i++ ) {
         Point2D_F64 p = active.get(i);

         sumX += p.x;
         sumY += p.y;
      }

      found.zero();
      found.x = sumX/active.size;
      found.y = sumY/active.size;

      for( int i = 0; i < active.size; i++ ) {
         Point2D_F64 p = active.get(i);
         double dx = p.x-found.x;
         double dy = p.y-found.y;

         found.cxx += dx*dx;
         found.cxy += dx*dy;
         found.cyy += dy*dy;
      }

      found.cxx /= active.size;
      found.cxy /= active.size;
      found.cyy /= active.size;

      System.out.println("  covariance xx = "+found.cxx+"  yy = "+found.cyy);
   }

   private void computeInverseCovariance() {

      Q.set(0,0,found.cxx);
      Q.set(0,1,found.cxy);
      Q.set(1,0,found.cxy);
      Q.set(1,1,found.cyy);

      Q.print();

      CovarianceOps.invert(Q);

      Q.print();

      // extract elements for speed
      found.sxx = Q.get(0,0);
      found.sxy = Q.get(1,0);
      found.syy = Q.get(1,1);
   }

   public Gaussian2D_F64 getFound() {
      return found;
   }
}
