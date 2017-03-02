package us.ihmc.ihmcPerception.chessboardDetection;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.io.printing.PrintTools;

public class ChessboardDetector
{
   private final OpenCVChessboardPoseEstimator detector;
   private boolean intrinsicSet = false;

   public ChessboardDetector(int gridCollumns, int gridRows, double gridSize)
   {
      detector = new OpenCVChessboardPoseEstimator(gridRows, gridCollumns, gridSize);
   }
   
   public void setMaxIterationsAndAccuracy(int maxIterations, double accuracyEpsilon)
   {
      detector.setMaxIterationsAndAccuracy(maxIterations, accuracyEpsilon);
   }

   public void setIntrinsicParameters(IntrinsicParameters intrinsicParameters)
   {

      detector.setCameraMatrix(intrinsicParameters.fx, intrinsicParameters.fy, intrinsicParameters.cx, intrinsicParameters.cy);
      intrinsicSet = true;
   }

   public RigidBodyTransform detectChessboard(BufferedImage image, boolean attemptExtremePitchDetection)
   {
      if (!intrinsicSet)
      {
         PrintTools.info("Intrinsic parameters not set");
         return null;
      }

      return detector.detect(image, attemptExtremePitchDetection);
   }
}
