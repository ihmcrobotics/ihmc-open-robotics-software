package us.ihmc.imageProcessing.segmentation;

import boofcv.struct.image.GrayF32;
import boofcv.struct.image.InterleavedU8;
import boofcv.struct.image.Planar;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;

import java.io.IOException;
import java.io.InputStream;

/**
 * @author Peter Abeles
 */
public class GaussianColorClassifier {

   public static Gaussian3D_F64 train( InputStream in ) throws IOException {
      LabeledPixelCodec.Set set = LabeledPixelCodec.read(in);

      Gaussian3D_F64 out = new Gaussian3D_F64();

      int N = set.values.size();

      // compute the mean
      double x0=0,x1=0,x2=0;

      for( double[] d : set.values ) {
         x0 += d[0];
         x1 += d[1];
         x2 += d[2];
      }
      x0 /= N; x1 /= N; x2 /= N;
      out.setMean(x0,x1,x2);

      // compute the covariance
      DenseMatrix64F Q = out.covariance;
      CommonOps.fill(Q,0);
      for( double[] d : set.values ) {
         double dx0 = d[0] - x0;
         double dx1 = d[1] - x1;
         double dx2 = d[2] - x2;

         out.covariance.data[0] += dx0*dx0;
         out.covariance.data[1] += dx0*dx1;
         out.covariance.data[2] += dx0*dx2;
         out.covariance.data[3] += dx1*dx0;
         out.covariance.data[4] += dx1*dx1;
         out.covariance.data[5] += dx1*dx2;
         out.covariance.data[6] += dx2*dx0;
         out.covariance.data[7] += dx2*dx1;
         out.covariance.data[8] += dx2*dx2;
      }

      CommonOps.divide(N, Q);

      // if perfectly gray scale all RGB values will be the same, creating a messed up covariance matrix
      out.computeInverse();
      if(MatrixFeatures.hasNaN(out.covarianceInv)) {
         for( int i = 0; i < 3; i++ ) {
            for( int j = 0; j < 3; j++ ) {
               if( i != j )
                  out.covariance.unsafe_set(i,j,0);
            }
         }
         out.computeInverse();
      }

      return out;
   }

   public static double chisq( double mean[] , DenseMatrix64F CovInv , double chan0 , double chan1 , double chan2 ) {
      chan0 -= mean[0];
      chan1 -= mean[1];
      chan2 -= mean[2];

      // X*inv(Q)*X
      // vector * inv(Q)
      double a = chan0*CovInv.data[0] + chan1*CovInv.data[3] + chan2*CovInv.data[6];
      double b = chan0*CovInv.data[1] + chan1*CovInv.data[4] + chan2*CovInv.data[7];
      double c = chan0*CovInv.data[2] + chan1*CovInv.data[5] + chan2*CovInv.data[8];

      //  (vector * inv(Q))*vector
      return a*chan0 + b*chan1 + c*chan2;
   }

   public static void classify(Planar<GrayF32> input , Gaussian3D_F64 model , double thresholdChiSq,
                               InterleavedU8 output ) {
      GrayF32 A = input.getBand(0);
      GrayF32 B = input.getBand(1);
      GrayF32 C = input.getBand(2);

      model.computeInverse();

      for( int y = 0; y < input.height; y++ ) {
         for( int x = 0; x < input.width; x++ ) {
            float a = A.unsafe_get(x,y);
            float b = B.unsafe_get(x,y);
            float c = C.unsafe_get(x,y);

            double dist = chisq(model.mean,model.covarianceInv,a,b,c);
            if( dist <= thresholdChiSq )
               output.set(x, y, 1);
            else
               output.set(x,y,0);
         }
      }
   }

   public static void classify(Planar<GrayF32> input , Gaussian3D_F64 models[] , double thresholdChiSq,
                               InterleavedU8 output ) {
      GrayF32 A = input.getBand(0);
      GrayF32 B = input.getBand(1);
      GrayF32 C = input.getBand(2);

      for( Gaussian3D_F64 model : models )
         model.computeInverse();

      for( int y = 0; y < input.height; y++ ) {
         for( int x = 0; x < input.width; x++ ) {
            float a = A.unsafe_get(x,y);
            float b = B.unsafe_get(x,y);
            float c = C.unsafe_get(x,y);

            boolean positive = false;
            for( int i = 0; i < models.length; i++ ) {
               Gaussian3D_F64 model = models[i];

               double dist = chisq(model.mean,model.covarianceInv,a,b,c);
               if( dist <= thresholdChiSq ) {
                  output.set(x, y, 1);
                  positive = true;
                  break;
               }
            }

            if( !positive)
               output.set(x,y,0);
         }
      }
   }

   public static void classifyEuclidean( Planar<GrayF32> input , Gaussian3D_F64 model , double threshold,
                                         InterleavedU8 output ) {
      threshold = threshold*threshold;

      GrayF32 A = input.getBand(0);
      GrayF32 B = input.getBand(1);
      GrayF32 C = input.getBand(2);

      double meanA = model.mean[0];
      double meanB = model.mean[1];
      double meanC = model.mean[2];

      for( int y = 0; y < input.height; y++ ) {
         for( int x = 0; x < input.width; x++ ) {
            double a = A.unsafe_get(x,y)-meanA;
            double b = B.unsafe_get(x,y)-meanB;
            double c = C.unsafe_get(x,y)-meanC;

            double dist = a*a + b*b + c*c;
            if( dist <= threshold )
               output.set(x,y,1);
            else
               output.set(x,y,0);
         }
      }
   }
}
