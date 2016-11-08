package us.ihmc.imageProcessing.sfm.d2.wrappers;

import boofcv.abst.sfm.d3.StereoVisualOdometry;
import boofcv.alg.sfm.overhead.OverheadView;
import boofcv.struct.calib.StereoParameters;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.Planar;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.se.Se2_F64;
import georegression.struct.se.Se3_F64;
import us.ihmc.imageProcessing.sfm.d2.EstimateCarMotion2D;

/**
 * @author Peter Abeles
 */
public class StereoVO_to_CarMotion2D implements EstimateCarMotion2D
{
   StereoVisualOdometry<GrayF32> alg;

   Se3_F64 groundToCamera;

   Se3_F64 groundToWorld = new Se3_F64();
   Se2_F64 output = new Se2_F64();

   public StereoVO_to_CarMotion2D(StereoVisualOdometry<GrayF32> alg)
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

   public boolean process(GrayF32 left, GrayF32 right, OverheadView<Planar<GrayF32>> overhead)
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

      double[] euler = ConvertRotation3D_F64.matrixToEuler(groundToWorld.getR(), EulerType.XYZ, null);

      output.setYaw(-euler[1]);

      return output;
   }
}
