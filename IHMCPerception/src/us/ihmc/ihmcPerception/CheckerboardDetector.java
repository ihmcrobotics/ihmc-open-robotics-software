package us.ihmc.ihmcPerception;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.ihmcPerception.chessboardDetection.OpenCVChessboardPoseEstimator;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.io.printing.PrintTools;

public class CheckerboardDetector
{
   private final OpenCVChessboardPoseEstimator detector;
   private boolean intrinsicSet = false;

   public CheckerboardDetector(int gridCollumns, int gridRows, double gridSize)
   {
      detector = new OpenCVChessboardPoseEstimator(gridRows, gridCollumns, gridSize);
   }

   public void setIntrinsicParameters(IntrinsicParameters intrinsicParameters)
   {

      detector.setCameraMatrix(intrinsicParameters.fx, intrinsicParameters.fy, intrinsicParameters.cx, intrinsicParameters.cy);
      intrinsicSet = true;
   }

   public RigidBodyTransform detectCheckerboard(BufferedImage image)
   {
      if (!intrinsicSet)
      {
         PrintTools.info("Intrinsic parameters not set");
         return null;
      }

      return detector.detect(image);
   }
}
