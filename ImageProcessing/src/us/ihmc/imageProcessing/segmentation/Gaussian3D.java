package us.ihmc.imageProcessing.segmentation;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.CovarianceOps;

/**
 * @author Peter Abeles
 */
public class Gaussian3D {
   public double[] mean = new double[3];
   public DenseMatrix64F covariance = new DenseMatrix64F(3,3);
   public DenseMatrix64F covarianceInv = new DenseMatrix64F(3,3);

   public void setMean(double x0, double x1, double x2) {
      mean[0] = x0;
      mean[1] = x1;
      mean[2] = x2;
   }

   public void computeInverse() {
      CovarianceOps.invert(covariance,covarianceInv);
   }

   public void print() {
      System.out.printf("Mean = [%6.2f %6.2f %6.2f]\n",mean[0],mean[1],mean[2]);
      System.out.println("Covariance:");
      covariance.print();
   }
}
