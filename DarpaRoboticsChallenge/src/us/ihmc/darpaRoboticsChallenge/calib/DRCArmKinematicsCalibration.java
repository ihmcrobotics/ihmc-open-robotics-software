package us.ihmc.darpaRoboticsChallenge.calib;

import java.awt.image.BufferedImage;

import org.ejml.data.DenseMatrix64F;

import boofcv.abst.calib.ConfigChessboard;
import boofcv.abst.calib.PlanarCalibrationDetector;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.calibration.Zhang99ComputeTargetHomography;
import boofcv.alg.geo.calibration.Zhang99DecomposeHomography;
import boofcv.factory.calib.FactoryPlanarCalibrationTarget;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.image.ImageFloat32;
import georegression.struct.se.Se3_F64;
import us.ihmc.SdfLoader.models.FullRobotModel;

/**
 * Using a camera calibration target verify the robot arm's kinematics.
 *
 * @author Peter Abeles
 */
public class DRCArmKinematicsCalibration
{
   FullRobotModel robotModel;

   PlanarCalibrationDetector target = FactoryPlanarCalibrationTarget.detectorChessboard(new ConfigChessboard(5, 6, 0.01));

   Zhang99ComputeTargetHomography computeH = new Zhang99ComputeTargetHomography(target.getLayout());
   Zhang99DecomposeHomography decomposeH = new Zhang99DecomposeHomography();

   ImageFloat32 gray = new ImageFloat32(1, 1);

   boolean hasIntrinsic = false;

   Se3_F64 targetToOrigin = new Se3_F64();

   public DRCArmKinematicsCalibration()
   {
   }

   public void setIntrinsic(IntrinsicParameters intrinsic)
   {
      DenseMatrix64F K = PerspectiveOps.calibrationMatrix(intrinsic, null);
      decomposeH.setCalibrationMatrix(K);
      hasIntrinsic = true;
   }

   public boolean estimateCameraPose(BufferedImage leftEye)
   {

      if (!hasIntrinsic)
         return false;

      gray.reshape(leftEye.getWidth(), leftEye.getHeight());
      ConvertBufferedImage.convertFrom(leftEye, gray);

      if (!target.process(gray))
         return false;

      if (!computeH.computeHomography(target.getDetectedPoints()))
         return false;

      DenseMatrix64F H = computeH.getHomography();

      targetToOrigin.set(decomposeH.decompose(H));

      return true;
   }

   public Se3_F64 getTargetToOrigin()
   {
      return targetToOrigin;
   }
}
