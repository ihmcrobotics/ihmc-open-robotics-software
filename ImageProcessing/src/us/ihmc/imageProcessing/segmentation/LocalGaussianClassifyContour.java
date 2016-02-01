package us.ihmc.imageProcessing.segmentation;

import georegression.struct.point.Point2D_I32;

import java.util.List;

import org.ddogleg.struct.FastQueue;

import boofcv.alg.filter.binary.Contour;
import boofcv.alg.misc.ImageMiscOps;
import boofcv.struct.image.ImageFloat32;
import boofcv.struct.image.ImageUInt8;
import boofcv.struct.image.MultiSpectral;

/**
 * @author Peter Abeles
 */
public class LocalGaussianClassifyContour {

   Gaussian3D_F64[]models;
   double thresholdChiSq;

   ImageUInt8 binary;
   ImageUInt8 changed = new ImageUInt8(1,1);

   ImageFloat32 band0,band1,band2;

   FastQueue<ChangeInfo> change = new FastQueue<ChangeInfo>(ChangeInfo.class,true);
   FastQueue<ChangeInfo> changeOld = new FastQueue<ChangeInfo>(ChangeInfo.class,true);

   int maxIterations = 5;

   public LocalGaussianClassifyContour(Gaussian3D_F64[] models, double thresholdChiSq ) {
      this.models = models;
      this.thresholdChiSq = thresholdChiSq;

      for( Gaussian3D_F64 model : models ) {
         model.computeInverse();
      }
   }

   public void process( List<Contour> contours , ImageUInt8 binary , MultiSpectral<ImageFloat32> color ) {
      band0 = color.getBand(0);
      band1 = color.getBand(1);
      band2 = color.getBand(2);
      this.binary = binary;

      changed.reshape(binary.width,binary.height);

      ImageMiscOps.fill(changed,0);

      change.reset();
      for( Contour c : contours ) {
         growMembers(c.external);
//         System.out.println("  total internal "+c.internal.size()+"  num change "+change.size);
         for( List<Point2D_I32> i : c.internal ) {
            growMembers(i);
//            System.out.println("  -- internal " + change.size);
         }
      }

//      System.out.println("total changed "+change.size);
      for( int i = 0; i < maxIterations && change.size != 0 ; i++ ) {
         applyChanges();
         FastQueue<ChangeInfo> tmp = change;
         change = changeOld;
         changeOld = tmp;
         change.reset();
         examineChanges();
//         System.out.println("  changes = "+change.size);
      }
   }


   private void growMembers(List<Point2D_I32> contour) {
      for( Point2D_I32 p : contour ) {
         checkNeighhood8(p.x,p.y);
      }
   }

   private void examineChanges() {
      for( int i = 0; i < changeOld.size; i++ ) {
         ChangeInfo p = changeOld.get(i);

         checkNeighhood8(p.x,p.y);
      }
   }

   private void checkNeighhood8(int x , int y ) {
      checkPixelOutside(x-1,y-1);
      checkPixelOutside(x  ,y-1);
      checkPixelOutside(x+1,y-1);
      checkPixelOutside(x-1,y);
      checkPixelOutside(x+1,y);
      checkPixelOutside(x-1,y+1);
      checkPixelOutside(x  ,y+1);
      checkPixelOutside(x+1,y+1);
   }

   private void checkPixelOutside( int x , int y ) {
      if( !binary.isInBounds(x,y))
         return;

      if( binary.unsafe_get(x, y) == 0 ) {
         float v0 = band0.unsafe_get(x,y);
         float v1 = band1.unsafe_get(x,y);
         float v2 = band2.unsafe_get(x,y);

         for( Gaussian3D_F64 model : models ) {
            double chisq = GaussianColorClassifier.chisq(model.mean,model.covarianceInv,v0,v1,v2);
            if( chisq <= thresholdChiSq ) {
               changeValue(x, y, 1);
               break;
            }
         }
      }
   }

   private void changeValue( int x , int y , int value ) {
      if( changed.get(x,y) == 0 ) {
         changed.set(x,y,1);
         change.grow().set(x,y,value);
      }
   }

   private void applyChanges() {
      for( int i = 0; i < change.size; i++ ) {
         ChangeInfo c = change.get(i);
         binary.unsafe_set(c.x, c.y, c.value);
//         System.out.println(" "+c.x+" "+c.y+" = "+c.value);
      }
   }

   public static class ChangeInfo extends Point2D_I32 {
      int value;

      public ChangeInfo() {
      }

      public void set( int x ,int y , int value ) {
         this.x = x;
         this.y = y;
         this.value = value;
      }
   }
}
