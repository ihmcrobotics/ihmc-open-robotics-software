package us.ihmc.imageProcessing.sfm.d2.wrappers;

import boofcv.abst.sfm.d3.MonocularPlaneVisualOdometry;
import boofcv.alg.sfm.overhead.OverheadView;
import boofcv.struct.calib.MonoPlaneParameters;
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
public class Mono_to_CarMotion2D implements EstimateCarMotion2D
{
   MonocularPlaneVisualOdometry<GrayF32> alg;

   MonoPlaneParameters param = new MonoPlaneParameters();


   Se3_F64 groundToCamera;

   Se3_F64 groundToWorld = new Se3_F64();
   Se2_F64 output = new Se2_F64();

   public Mono_to_CarMotion2D(MonocularPlaneVisualOdometry<GrayF32> alg)
   {
      this.alg = alg;
   }

   public void setParameters(StereoParameters config)
   {
      this.param.intrinsic = config.getLeft();
      if( param.planeToCamera != null )
         alg.setCalibration(param);
   }

   public void setGroundToCamera(Se3_F64 groundToCamera)
   {
      this.param.planeToCamera = groundToCamera;
      if( param.intrinsic != null )
         alg.setCalibration(param);
      this.groundToCamera = groundToCamera;
   }

   public void reset()
   {
      alg.reset();
   }

   public boolean process(GrayF32 left, GrayF32 right, OverheadView<Planar<GrayF32>> overhead)
   {
      if( !alg.process(left) )
         return false;

      groundToCamera.concat(alg.getCameraToWorld(), groundToWorld);

      return true;
   }

   public Se2_F64 getCurrToWorld()
   {
      output.T.x = groundToWorld.T.z;
      output.T.y = -groundToWorld.T.x;

      double[] euler = ConvertRotation3D_F64.matrixToEuler(groundToWorld.getR(), EulerType.XYZ, null);
      double rotY = -euler[1];

      output.setYaw(rotY);

      return output;
   }
}