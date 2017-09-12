package us.ihmc.imageProcessing.sfm.d2;

import boofcv.alg.sfm.overhead.OverheadView;
import boofcv.struct.calib.StereoParameters;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.Planar;
import georegression.struct.se.Se2_F64;
import georegression.struct.se.Se3_F64;

/**
 * @author Peter Abeles
 */
public interface EstimateCarMotion2D
{

   public void setParameters( StereoParameters config );

   public void setGroundToCamera( Se3_F64 groundToCamera );

   public void reset();

   public boolean process( GrayF32 left , GrayF32 right , OverheadView<Planar<GrayF32>> overhead );

   /**
    * Returns the estimated motion since the last frame.
    */
   public Se2_F64 getCurrToWorld();
}
