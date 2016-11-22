package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.ihmcPerception.fiducialDetector.FiducialDetectorFromCameraImages;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.thread.ThreadTools;

public class FiducialDetectorBehaviorService extends ThreadedBehaviorService
{
   private static final double DEFAULT_FIDUCIAL_SIZE = 1.0;
   private static final double DEFAULT_FIELD_OF_VIEW_X_IN_RADIANS = Math.toRadians(80.0);
   private static final double DEFAULT_FIELD_OF_VIEW_Y_IN_RADIANS = Math.toRadians(45.0);
   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<VideoPacket>(20);

   private final FiducialDetectorFromCameraImages fiducialDetectorFromCameraImages;
   private RigidBodyTransform transformFromReportedToFiducialFrame;

   public FiducialDetectorBehaviorService(CommunicationBridgeInterface communicationBridge,
                                          YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(FiducialDetectorBehaviorService.class.getSimpleName(), communicationBridge);

      getCommunicationBridge().attachNetworkListeningQueue(videoPacketQueue, VideoPacket.class);

      transformFromReportedToFiducialFrame = new RigidBodyTransform();
      fiducialDetectorFromCameraImages = new FiducialDetectorFromCameraImages(transformFromReportedToFiducialFrame, getYoVariableRegistry(), yoGraphicsListRegistry);

      fiducialDetectorFromCameraImages.setFieldOfView(DEFAULT_FIELD_OF_VIEW_X_IN_RADIANS, DEFAULT_FIELD_OF_VIEW_Y_IN_RADIANS);
      fiducialDetectorFromCameraImages.setLocationEnabled(false);
      fiducialDetectorFromCameraImages.setExpectedFiducialSize(DEFAULT_FIDUCIAL_SIZE);
   }

   @Override
   public void doThreadAction()
   {
      if (videoPacketQueue.isNewPacketAvailable())
      {
         VideoPacket videoPacket = videoPacketQueue.getLatestPacket();

         fiducialDetectorFromCameraImages.setNewVideoPacket(videoPacket);
      }
      else
      {
         ThreadTools.sleep(10);
      }
   }

   public void setTargetIDToLocate(long targetIDToLocate)
   {
      fiducialDetectorFromCameraImages.setTargetIDToLocate(targetIDToLocate);
   }

   public void setLocationEnabled(boolean locationEnabled)
   {
      fiducialDetectorFromCameraImages.setLocationEnabled(locationEnabled);
   }

   public boolean getTargetIDHasBeenLocated()
   {
      return fiducialDetectorFromCameraImages.getTargetIDHasBeenLocated();
   }

   public void getReportedFiducialPoseWorldFrame(FramePose framePoseToPack)
   {
      fiducialDetectorFromCameraImages.getReportedFiducialPoseWorldFrame(framePoseToPack);
   }
}
