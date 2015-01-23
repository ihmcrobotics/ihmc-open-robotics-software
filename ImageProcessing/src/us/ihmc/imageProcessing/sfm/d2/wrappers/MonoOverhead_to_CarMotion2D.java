package us.ihmc.imageProcessing.sfm.d2.wrappers;

import georegression.struct.se.Se2_F64;
import georegression.struct.se.Se3_F64;
import us.ihmc.imageProcessing.sfm.d2.EstimateCarMotion2D;
import boofcv.abst.sfm.d2.ImageMotion2D;
import boofcv.alg.sfm.overhead.OverheadView;
import boofcv.core.image.ConvertImage;
import boofcv.struct.calib.StereoParameters;
import boofcv.struct.image.ImageFloat32;
import boofcv.struct.image.MultiSpectral;

/**
 * @author Peter Abeles
 */
public class MonoOverhead_to_CarMotion2D implements EstimateCarMotion2D
{
   // convert RGB to gray scale image
   ImageFloat32 gray = new ImageFloat32(1,1);

   // estimates 2D motion inside the overhead image
   private ImageMotion2D<ImageFloat32,Se2_F64> motion2D;

   private Se2_F64 motion = new Se2_F64();

   public MonoOverhead_to_CarMotion2D(ImageMotion2D<ImageFloat32, Se2_F64> motion2D)
   {
      this.motion2D = motion2D;
   }

   public void setParameters(StereoParameters config){}

   public void setGroundToCamera(Se3_F64 groundToCamera){}

   public void reset()
   {
      motion2D.reset();
   }

   public boolean process(ImageFloat32 left, ImageFloat32 right, OverheadView<MultiSpectral<ImageFloat32>> overhead)
   {
      MultiSpectral<ImageFloat32> overheadRGB = overhead.getImage();
      gray.reshape(overheadRGB.getWidth(),overheadRGB.getHeight());
      ConvertImage.average(overheadRGB,gray);

      if( !motion2D.process(gray) ) {
         return false;
      }

      // take in account map units
      Se2_F64 a = motion2D.getFirstToCurrent();

      motion.set(a);

      motion.T.x = a.T.x*overhead.cellSize;
      motion.T.y = a.T.y*overhead.cellSize;

      return true;
   }

   public Se2_F64 getCurrToWorld()
   {
      return motion;
   }
}
