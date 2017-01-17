package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.ihmcPerception.fiducialDetector.FiducialDetectorFromCameraImages;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.thread.ThreadTools;

public class FiducialDetectorBehaviorService extends GoalDetectorBehaviorService
{
   private static final double DEFAULT_FIDUCIAL_SIZE = 0.22;
   private static final double DEFAULT_FIELD_OF_VIEW_X_IN_RADIANS = Math.toRadians(80.0);
   private static final double DEFAULT_FIELD_OF_VIEW_Y_IN_RADIANS = Math.toRadians(45.0);
   
   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<VideoPacket>(2);

   private final Object fiducialDetectorFromCameraImagesConch = new Object();
   private final FiducialDetectorFromCameraImages fiducialDetectorFromCameraImages;
   private RigidBodyTransform transformFromReportedToFiducialFrame;
   
   private final BooleanYoVariable locationEnabled;

   public FiducialDetectorBehaviorService(CommunicationBridgeInterface communicationBridge,
                                          YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(FiducialDetectorBehaviorService.class.getSimpleName(), communicationBridge);

      getCommunicationBridge().attachNetworkListeningQueue(videoPacketQueue, VideoPacket.class);

      transformFromReportedToFiducialFrame = new RigidBodyTransform();
      fiducialDetectorFromCameraImages = new FiducialDetectorFromCameraImages(transformFromReportedToFiducialFrame, getYoVariableRegistry(), yoGraphicsListRegistry);

      fiducialDetectorFromCameraImages.setFieldOfView(DEFAULT_FIELD_OF_VIEW_X_IN_RADIANS, DEFAULT_FIELD_OF_VIEW_Y_IN_RADIANS);
      fiducialDetectorFromCameraImages.setExpectedFiducialSize(DEFAULT_FIDUCIAL_SIZE);
      
      String prefix = "fiducial";
      locationEnabled = new BooleanYoVariable(prefix + "LocationEnabled", getYoVariableRegistry());
      
      locationEnabled.set(false);
   }

   @Override
   public void doThreadAction()
   {
      if (videoPacketQueue.isNewPacketAvailable())
      {
         VideoPacket videoPacket = videoPacketQueue.getLatestPacket();

         synchronized (fiducialDetectorFromCameraImagesConch)
         {
            fiducialDetectorFromCameraImages.detectFromVideoPacket(videoPacket);
         }
      }
      else
      {
         ThreadTools.sleep(10);
      }
   }

   public void setTargetIDToLocate(long targetIDToLocate)
   {
      synchronized (fiducialDetectorFromCameraImagesConch)
      {
         fiducialDetectorFromCameraImages.setTargetIDToLocate(targetIDToLocate);
      }
   }
   
   public void setExpectedFiducialSize(double expectedFiducialSize)
   {
      synchronized (fiducialDetectorFromCameraImagesConch)
      {
         fiducialDetectorFromCameraImages.setExpectedFiducialSize(expectedFiducialSize);
      }
   }

   @Override
   public boolean getGoalHasBeenLocated()
   {
      synchronized (fiducialDetectorFromCameraImagesConch)
      {
         return fiducialDetectorFromCameraImages.getTargetIDHasBeenLocated();
      }
   }

   @Override
   public void getReportedGoalPoseWorldFrame(FramePose framePoseToPack)
   {
      synchronized (fiducialDetectorFromCameraImagesConch)
      {
         fiducialDetectorFromCameraImages.getReportedFiducialPoseWorldFrame(framePoseToPack);
      }
   }
   
   @Override
   public void run()
   {
      super.run();
      locationEnabled.set(true);
   }
   
   @Override
   public void pause()
   {
      super.pause();
      locationEnabled.set(false);
   }
   
   @Override
   public void destroy()
   {
      super.destroy();
      locationEnabled.set(false);
   }

   @Override
   public void initialize()
   {
      fiducialDetectorFromCameraImages.reset();
   }
}
