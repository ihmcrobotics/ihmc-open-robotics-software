package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.ihmcPerception.fiducialDetector.FiducialDetectorFromCameraImages;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoBoolean;

public class FiducialDetectorBehaviorService extends GoalDetectorBehaviorService
{
   private static final double DEFAULT_FIDUCIAL_SIZE = 1;
   private static final double DEFAULT_FIELD_OF_VIEW_X_IN_RADIANS = Math.toRadians(80.0);
   private static final double DEFAULT_FIELD_OF_VIEW_Y_IN_RADIANS = Math.toRadians(45.0);

   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<VideoPacket>(2);

   private final Object fiducialDetectorFromCameraImagesConch = new Object();
   private final FiducialDetectorFromCameraImages fiducialDetectorFromCameraImages;
   private RigidBodyTransform transformFromReportedToFiducialFrame;

   private final YoBoolean locationEnabled;

   public FiducialDetectorBehaviorService(String robotName, String ThreadName, Ros2Node ros2Node, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(robotName, ThreadName, ros2Node);

      createSubscriber(VideoPacket.class, ROS2Tools.getDefaultTopicNameGenerator(), videoPacketQueue::put);

      transformFromReportedToFiducialFrame = new RigidBodyTransform();
      fiducialDetectorFromCameraImages = new FiducialDetectorFromCameraImages(transformFromReportedToFiducialFrame, getYoVariableRegistry(),
                                                                              yoGraphicsListRegistry,ThreadName);

      fiducialDetectorFromCameraImages.setFieldOfView(DEFAULT_FIELD_OF_VIEW_X_IN_RADIANS, DEFAULT_FIELD_OF_VIEW_Y_IN_RADIANS);
      fiducialDetectorFromCameraImages.setExpectedFiducialSize(DEFAULT_FIDUCIAL_SIZE);

      String prefix = "fiducial";
      locationEnabled = new YoBoolean(prefix + "LocationEnabled", getYoVariableRegistry());

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
   public void getReportedGoalPoseWorldFrame(FramePose3D framePoseToPack)
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
