package us.ihmc.imageProcessing.segmentation;


/**
 * Gaussian distribution in 2D
 *
 * @author Peter Abeles
 */
public class Gaussian2D_F64 {

   // mean values
   public double x,y;

   // covariance
   public double cxx,cxy,cyy;

   // inverse of the covariance
   public double sxx,sxy,syy;

   /**
    * Computes the Chi-Square value to the provided sample
    */
   public double chisq( double px , double py ) {
      double dx = px-x;
      double dy = py-y;

      // chi-square = d^T*S*d

      double tmp0 = dx*sxx + dy*sxy;
      double tmp1 = dx*sxy + dy*syy;

      return tmp0*dx + tmp1*dy;
   }

   /**
    * Inverts the covariance using matrix minor
    */
   public void invertCovariance() {
      double scale = 1.0/Math.max(Math.abs(cxx),Math.abs(cyy));

      double a11 = cxx*scale;
      double a12 = cxy*scale;
      double a22 = cyy*scale;

      double m11 = a22;
      double m12 = -a12;
      double m22 = a11;

      double det = (a11*m11 + a12*m12)/scale;

      sxx = m11 / det;
      sxy = m12 / det;
      syy = m22 / det;
   }

   public void zero() {
      x=y=cxx=cyy=cxy=sxx=sxy=syy=0;
   }

   public void set( Gaussian2D_F64 g ) {
      x = g.x;
      y = g.y;
      cxx = g.cxx;
      cxy = g.cxy;
      cyy = g.cyy;
      sxx = g.sxx;
      sxy = g.sxy;
      syy = g.syy;
   }

   public Gaussian2D_F64 copy() {
      Gaussian2D_F64 ret = new Gaussian2D_F64();
      ret.set(this);
      return ret;
   }
}
