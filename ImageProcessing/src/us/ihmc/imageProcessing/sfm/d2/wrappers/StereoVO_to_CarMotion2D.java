package us.ihmc.imageProcessing.sfm.d2.wrappers;

import georegression.geometry.RotationMatrixGenerator;
import georegression.struct.se.Se2_F64;
import georegression.struct.se.Se3_F64;
import us.ihmc.imageProcessing.sfm.d2.EstimateCarMotion2D;
import boofcv.abst.sfm.d3.StereoVisualOdometry;
import boofcv.alg.sfm.overhead.OverheadView;
import boofcv.struct.calib.StereoParameters;
import boofcv.struct.image.ImageFloat32;
import boofcv.struct.image.MultiSpectral;

/**
 * @author Peter Abeles
 */
public class StereoVO_to_CarMotion2D implements EstimateCarMotion2D
{
   StereoVisualOdometry<ImageFloat32> alg;

   Se3_F64 groundToCamera;

   Se3_F64 groundToWorld = new Se3_F64();
   Se2_F64 output = new Se2_F64();

   public StereoVO_to_CarMotion2D(StereoVisualOdometry<ImageFloat32> alg)
   {
      this.alg = alg;
   }

   public void setParameters(StereoParameters config)
   {
      alg.setCalibration(config);
   }

   public void setGroundToCamera(Se3_F64 groundToCamera)
   {
      this.groundToCamera = groundToCamera;
   }

   public void reset()
   {
      alg.reset();
   }

   public boolean process(ImageFloat32 left, ImageFloat32 right, OverheadView<MultiSpectral<ImageFloat32>> overhead)
   {
      if( !alg.process(left,right) )
         return false;

      groundToCamera.concat(alg.getCameraToWorld(), groundToWorld);

      return true;
   }

   public Se2_F64 getCurrToWorld()
   {
      output.T.x = groundToWorld.T.z;
      output.T.y = -groundToWorld.T.x;

      double[] euler = RotationMatrixGenerator.matrixToEulerXYZ(groundToWorld.getR(),(double[])null);

      output.setYaw(-euler[1]);

      return output;
   }
}
