package us.ihmc.imageProcessing.segmentation;

import boofcv.alg.filter.binary.Contour;
import boofcv.alg.misc.ImageMiscOps;
import boofcv.misc.BoofMiscOps;
import boofcv.struct.ImageRectangle;
import boofcv.struct.image.Color3_F32;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.InterleavedU8;
import boofcv.struct.image.Planar;
import georegression.geometry.UtilPoint3D_F32;
import georegression.struct.point.Point2D_I32;
import org.ddogleg.struct.FastQueue;

import java.util.List;

/**
 * @author Peter Abeles
 */
public class LocalReclassifyContour {



   InterleavedU8 binary;
   InterleavedU8 changed = new InterleavedU8(1, 1, 1);

   ImageRectangle rect = new ImageRectangle();
   int radius = 2;

   GrayF32 band0,band1,band2;
   float distIn,distOut;
   int numIn,numOut;

   Color3_F32 meanOut = new Color3_F32();
   Color3_F32 meanIn = new Color3_F32();

   double maxDistance = 40;

   FastQueue<ChangeInfo> change = new FastQueue<ChangeInfo>(ChangeInfo.class,true);
   FastQueue<ChangeInfo> changeOld = new FastQueue<ChangeInfo>(ChangeInfo.class,true);
   int[] data = new int[1];

   public void process( List<Contour> contours , InterleavedU8 binary , Planar<GrayF32> color ) {
      band0 = color.getBand(0);
      band1 = color.getBand(1);
      band2 = color.getBand(2);
      this.binary = binary;

      changed.reshape(binary.width,binary.height);

      ImageMiscOps.fill(changed,0);

      change.reset();
      for( Contour c : contours ) {
//         shrinkMembers(c.external);
         growMembers(c.external);
         System.out.println("  total internal "+c.internal.size()+"  num change "+change.size);
         for( List<Point2D_I32> i : c.internal ) {
//            shrinkMembers(i);
            growMembers(i);
            System.out.println("  -- internal " + change.size);
         }
      }

      System.out.println("Blobs found "+contours.size());
      System.out.println("total changed "+change.size);
      while( change.size != 0 ) {
         applyChanges();
         FastQueue<ChangeInfo> tmp = change;
         change = changeOld;
         changeOld = tmp;
         change.reset();
         examineChanges();
//         System.out.println("  changes = "+change.size);
      }
   }

   private void shrinkMembers(List<Point2D_I32> contour) {
      for( Point2D_I32 p : contour ) {
         if (computeDistance(p.x,p.y))
            continue;

         if( distOut < distIn && distOut < maxDistance ) {
            changeValue(p.x,p.y,0);
         }
      }
   }

   private void growMembers(List<Point2D_I32> contour) {
      for( Point2D_I32 p : contour ) {

         // check 8-neighborhood
         checkPixelOutside(p.x-1,p.y-1);
         checkPixelOutside(p.x  ,p.y-1);
         checkPixelOutside(p.x+1,p.y-1);
         checkPixelOutside(p.x-1,p.y);
         checkPixelOutside(p.x+1,p.y);
         checkPixelOutside(p.x-1,p.y+1);
         checkPixelOutside(p.x  ,p.y+1);
         checkPixelOutside(p.x+1,p.y+1);
      }
   }

   private void examineChanges() {
      for( int i = 0; i < changeOld.size; i++ ) {
         ChangeInfo p = changeOld.get(i);

         computeDistance(p.x,p.y);

         binary.get(p.x,p.y, data);
         if( data[0] == 1 ) {
            if( distOut < distIn && distOut < maxDistance ) {
               changeValue(p.x,p.y,0);
            }
         } else {
            // If distances are similar or its a better fit for the 1 pixels, set value to 1
            if( Math.abs(distOut - distIn) < 20 || ( distIn < distOut && distIn < maxDistance) ) {
               changeValue(p.x,p.y,1);
            }
         }
      }
   }

   private void checkPixelOutside( int x , int y ) {
      if( !binary.isInBounds(x,y))
         return;

      binary.get(x, y, data);
      if( data[0] == 0 ) {
         if( computeDistance(x,y) )
            return;

         // If distances are similar or its a better fit for the 1 pixels, set value to 1
         if( numOut <= 2 || Math.abs(distOut - distIn) < 40 || ( distIn < distOut && distIn < maxDistance) ) {
            changeValue(x,y,1);
//         } else {
//            System.out.println(" check outside no change: "+distIn+" "+distOut);
         }
      }
   }

   private boolean computeDistance( int c_x , int c_y ) {
      rect.x0 = c_x-radius;
      rect.x1 = c_x+radius+1;
      rect.y0 = c_y-radius;
      rect.y1 = c_y+radius+1;
      BoofMiscOps.boundRectangleInside(binary, rect);

      meanOut.setAll(0);
      meanIn.setAll(0);
      numOut = 0;
      numIn = 0;
      for( int y = rect.y0; y < rect.y1; y++ ) {
         for( int x = rect.x0; x < rect.x1; x++ ) {
            binary.get(x,y,data);
            if( data[0] == 0 ) {
               meanOut.band0 += band0.unsafe_get(x,y);
               meanOut.band1 += band1.unsafe_get(x,y);
               meanOut.band2 += band2.unsafe_get(x,y);
               numOut++;
            } else {
               meanIn.band0 += band0.unsafe_get(x,y);
               meanIn.band1 += band1.unsafe_get(x,y);
               meanIn.band2 += band2.unsafe_get(x,y);
               numIn++;
            }
         }
      }

      // this can happen if a pixel is along the image border
      if( numOut == 0 ) {
         return true;
      }

      meanOut.band0 /= numOut;
      meanOut.band1 /= numOut;
      meanOut.band2 /= numOut;

      meanIn.band0 /= numIn;
      meanIn.band1 /= numIn;
      meanIn.band2 /= numIn;

      float target0 = band0.unsafe_get(c_x,c_y);
      float target1 = band1.unsafe_get(c_x,c_y);
      float target2 = band2.unsafe_get(c_x,c_y);

      distOut = UtilPoint3D_F32.distance(meanOut.band0, meanOut.band1, meanOut.band2, target0, target1, target2);
      distIn = UtilPoint3D_F32.distance(meanIn.band0, meanIn.band1, meanIn.band2, target0, target1, target2);
      return false;
   }

   private void changeValue( int x , int y , int value ) {
      changed.get(x,y,data);
      if( data[0] == 0 ) {
         changed.set(x,y,1);
         change.grow().set(x,y,value);
      }
   }

   private void applyChanges() {
      for( int i = 0; i < change.size; i++ ) {
         ChangeInfo c = change.get(i);
         binary.set(c.x, c.y, c.value);
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
