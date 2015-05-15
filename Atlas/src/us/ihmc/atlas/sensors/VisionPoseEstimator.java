package us.ihmc.atlas.sensors;

import java.util.concurrent.LinkedBlockingQueue;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.DetectedObjectPacket;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.humanoidOperatorInterface.sensors.DetectedObjectManager.DetectedObjectId;
import us.ihmc.ihmcPerception.chessboardDetection.OpenCVChessboardPoseEstimator;
import us.ihmc.sensorProcessing.sensorData.CameraData;
import us.ihmc.sensorProcessing.sensorData.DRCStereoListener;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import boofcv.struct.calib.IntrinsicParameters;

public class VisionPoseEstimator implements DRCStereoListener
{
   private final OpenCVChessboardPoseEstimator chessboardDetector;
   private final static boolean DEBUG = false;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;


   PacketCommunicator communicator;
   LinkedBlockingQueue<Pair<CameraData, RigidBodyTransform>> imagesToProcess = new LinkedBlockingQueue<>(2);

   public VisionPoseEstimator(PacketCommunicator packetCommunicator, boolean runningOnRealRobot)
   {
      if (runningOnRealRobot)
      {
         chessboardDetector = new OpenCVChessboardPoseEstimator(4, 5, 0.01);
      }
      else
      {
         chessboardDetector = new OpenCVChessboardPoseEstimator(5, 6, 0.00935);
      }

      this.communicator = packetCommunicator;

      this.robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
      this.communicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);
      startChessBoardDetector();
   }

   @Override
   public void newImageAvailable(CameraData data, RigidBodyTransform transformToWorld)
   {
      imagesToProcess.offer(new Pair<CameraData, RigidBodyTransform>(data, transformToWorld));
   }

   public void startChessBoardDetector()
   {
      new Thread(new Runnable()
      {

         @Override
         public void run()
         {

            while (true)
            {
               try
               {
                  Pair<CameraData, RigidBodyTransform> data = imagesToProcess.take();

                  //
                  long timeStart = System.currentTimeMillis();
                  
                  IntrinsicParameters intrinsicParameters = data.first().intrinsicParameters;
                  chessboardDetector.setCameraMatrix(intrinsicParameters.fx, intrinsicParameters.fy, intrinsicParameters.cx, intrinsicParameters.cy);
                  
                  RigidBodyTransform targetToCameraOpticalFrame = chessboardDetector.detect(data.first().image);
                  if (targetToCameraOpticalFrame != null)
                  {
                     RigidBodyTransform cameraToWorld = data.second();
                     RigidBodyTransform opticalFrameToCameraFrame = new RigidBodyTransform();
                     opticalFrameToCameraFrame.setEuler(-Math.PI / 2.0, 0.0, -Math.PI / 2);
                     RigidBodyTransform targetToWorld = new RigidBodyTransform();
                     
                     targetToWorld.multiply(cameraToWorld, opticalFrameToCameraFrame);
                     targetToWorld.multiply(targetToCameraOpticalFrame);
                     
                     communicator.send(new DetectedObjectPacket(targetToWorld, DetectedObjectId.RIGHT_HAND.ordinal()));
                  }

                  long detectionTimeInNanos = System.currentTimeMillis() - timeStart;
                  PrintTools.debug(VisionPoseEstimator.DEBUG, this, "detection time " + detectionTimeInNanos);

               }
               catch (InterruptedException e)
               {
                  e.printStackTrace();
               }
            }
         }
      }).start();

   }

}
