package us.ihmc.imageProcessing.sfm.d2;

import georegression.struct.se.Se2_F64;
import georegression.struct.se.Se3_F64;
import boofcv.alg.sfm.overhead.OverheadView;
import boofcv.struct.calib.StereoParameters;
import boofcv.struct.image.ImageFloat32;
import boofcv.struct.image.MultiSpectral;

/**
 * @author Peter Abeles
 */
public interface EstimateCarMotion2D
{

   public void setParameters( StereoParameters config );

   public void setGroundToCamera( Se3_F64 groundToCamera );

   public void reset();

   public boolean process( ImageFloat32 left , ImageFloat32 right , OverheadView<MultiSpectral<ImageFloat32>> overhead );

   /**
    * Returns the estimated motion since the last frame.
    */
   public Se2_F64 getCurrToWorld();
}
