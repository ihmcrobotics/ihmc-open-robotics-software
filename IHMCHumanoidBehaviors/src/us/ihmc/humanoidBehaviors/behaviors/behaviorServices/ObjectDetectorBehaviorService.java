package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.ihmcPerception.objectDetector.ObjectDetectorFromCameraImages;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.thread.ThreadTools;

public class ObjectDetectorBehaviorService extends GoalDetectorBehaviorService
{
   private static final double DEFAULT_OBJECT_SIZE = 1.0;
   private static final double DEFAULT_FIELD_OF_VIEW_X_IN_RADIANS = Math.toRadians(80.0);
   private static final double DEFAULT_FIELD_OF_VIEW_Y_IN_RADIANS = Math.toRadians(45.0);

   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<VideoPacket>(20);

   private final Object detectorFromCameraImagesConch = new Object();
   private final ObjectDetectorFromCameraImages objectDetectorFromCameraImages;
   private RigidBodyTransform transformFromReportedToFiducialFrame;

   private final BooleanYoVariable locationEnabled;

   public ObjectDetectorBehaviorService(CommunicationBridgeInterface communicationBridge,
                                        YoGraphicsListRegistry yoGraphicsListRegistry) throws Exception
   {
      super(ObjectDetectorBehaviorService.class.getSimpleName(), communicationBridge);

      getCommunicationBridge().attachNetworkListeningQueue(videoPacketQueue, VideoPacket.class);

      transformFromReportedToFiducialFrame = new RigidBodyTransform();
      objectDetectorFromCameraImages = new ObjectDetectorFromCameraImages(transformFromReportedToFiducialFrame, getYoVariableRegistry(), yoGraphicsListRegistry);

      objectDetectorFromCameraImages.setFieldOfView(DEFAULT_FIELD_OF_VIEW_X_IN_RADIANS, DEFAULT_FIELD_OF_VIEW_Y_IN_RADIANS);
      objectDetectorFromCameraImages.setExpectedObjectSize(DEFAULT_OBJECT_SIZE);
      
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

         synchronized (detectorFromCameraImagesConch)
         {
            objectDetectorFromCameraImages.detectFromVideoPacket(videoPacket);
         }
      }
      else
      {
         ThreadTools.sleep(10);
      }
   }

   public void setExpectedObjectSize(double expectedFiducialSize)
   {
      synchronized (detectorFromCameraImagesConch)
      {
         objectDetectorFromCameraImages.setExpectedObjectSize(expectedFiducialSize);
      }
   }

   @Override
   public boolean getGoalHasBeenLocated()
   {
      synchronized (detectorFromCameraImagesConch)
      {
         return objectDetectorFromCameraImages.getTargetIDHasBeenLocated();
      }
   }

   @Override
   public void getReportedGoalPoseWorldFrame(FramePose framePoseToPack)
   {
      synchronized (detectorFromCameraImagesConch)
      {
         objectDetectorFromCameraImages.getReportedFiducialPoseWorldFrame(framePoseToPack);
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
}
