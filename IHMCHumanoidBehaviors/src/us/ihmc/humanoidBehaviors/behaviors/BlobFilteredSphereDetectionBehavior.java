package us.ihmc.humanoidBehaviors.behaviors;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3f;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.ColoredCircularBlobDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.ihmcPerception.vision.shapes.HSVRange;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class BlobFilteredSphereDetectionBehavior extends SphereDetectionBehavior
{
   private final ReferenceFrame headFrame;
   private static final double FILTERING_ANGLE = Math.toRadians(5.0);
   private static final double FILTERING_MIN_DISTANCE = 0.1;
   private static final double FILTERING_MAX_DISTANCE = 7.0;

   // http://www.bostondynamics.com/img/MultiSense_SL.pdf
   private static final double MUTLISENSE_HORIZONTAL_FOV = Math.toRadians(80.0);
   private static final double MULTISENSE_VERTICAL_FOV = Math.toRadians(45.0);

   private final BooleanYoVariable runBlobFilter = new BooleanYoVariable("runBlobFilter", registry);
   private int numBallsDetected;
   
   private final ColoredCircularBlobDetectorBehaviorService coloredCircularBlobDetectorBehaviorService;
   
   public BlobFilteredSphereDetectionBehavior(BehaviorCommunicationBridge behaviorCommunicationBridge, HumanoidReferenceFrames referenceFrames,
         SDFFullHumanoidRobotModel fullRobotModel)
   {
      super(behaviorCommunicationBridge, referenceFrames);

      behaviorCommunicationBridge.attachGlobalListener(getNetworkProcessorGlobalObjectConsumer());
      attachNetworkProcessorListeningQueue(pointCloudQueue, PointCloudWorldPacket.class);

      coloredCircularBlobDetectorBehaviorService = new ColoredCircularBlobDetectorBehaviorService(this);
//      HSVRange greenRange = new HSVRange(new HSVValue(78, 100, 100), new HSVValue(83, 255, 255));
//      coloredCircularBlobDetectorBehaviorService.addHSVRange(greenRange);

      runBlobFilter.set(false);
      this.headFrame = fullRobotModel.getHead().getBodyFixedFrame();
   }

   public void addHSVRange(HSVRange hsvRange)
   {
      coloredCircularBlobDetectorBehaviorService.addHSVRange(hsvRange);
   }

   public void resetHSVRanges()
   {
      coloredCircularBlobDetectorBehaviorService.clearHSVRanges();
   }

   @Override
   public void doControl()
   {
      while ((pointCloudPacket = pointCloudQueue.poll()) != null)
      {
         pointCloudPacketLatest = pointCloudPacket;
      }

      if (pointCloudPacketLatest != null)
      {
         Point3f[] fullPointCloud = pointCloudPacketLatest.getDecayingWorldScan();

         if(runBlobFilter.getBooleanValue())
         {
            Point3f[] filteredPointCloud = filterPointsNearBall(fullPointCloud);
            findBallsAndSaveResult(filteredPointCloud);
         }
         else
         {
            findBallsAndSaveResult(fullPointCloud);
         }
      }
   }

   private Point3f[] filterPointsNearBall(Point3f[] fullPointCloud)
   {
      List<Point3f> filteredPoints = new ArrayList<Point3f>();
      RigidBodyTransform worldToCameraTransform = headFrame.getTransformToWorldFrame();
      worldToCameraTransform.invert();

      BufferedImage latestCameraImage = coloredCircularBlobDetectorBehaviorService.getLatestUnmodifiedCameraImage();
      if (latestCameraImage != null)
      {
         int cameraPixelWidth = latestCameraImage.getWidth();
         int cameraPixelHeight = latestCameraImage.getHeight();
   
         for (Point2d latestBallPosition2d : coloredCircularBlobDetectorBehaviorService.getLatestBallPositionSet())
         {
            double ballCenterX = latestBallPosition2d.getX() - latestCameraImage.getMinX();
            double ballCenterY = latestBallPosition2d.getY() - latestCameraImage.getMinY();
      
            double desiredRayAngleX = MULTISENSE_VERTICAL_FOV * (-ballCenterY / cameraPixelHeight + 0.5);
            double desiredRayAngleY = MUTLISENSE_HORIZONTAL_FOV * (-ballCenterX / cameraPixelWidth - 0.5);
      
            Point3f candidateLidarPoint = new Point3f();
      
            for (int i = 0; i < fullPointCloud.length; i++)
            {
               candidateLidarPoint.set(fullPointCloud[i]);
               worldToCameraTransform.transform(candidateLidarPoint);
      
               if (candidateLidarPoint.x > FILTERING_MIN_DISTANCE && candidateLidarPoint.x < FILTERING_MAX_DISTANCE)
               {
                  // rayAngle axes are in terms of the buffered image (y-down), temp pnt axes are in terms of camera frame (z-up)
                  double rayAngleX = Math.atan2(candidateLidarPoint.z, candidateLidarPoint.x);
                  double rayAngleY = Math.atan2(candidateLidarPoint.y, candidateLidarPoint.x);
                  if (Math.abs(rayAngleX - desiredRayAngleX) < FILTERING_ANGLE && Math.abs(rayAngleY - desiredRayAngleY) < FILTERING_ANGLE)
                  {
                     filteredPoints.add(candidateLidarPoint);
                  }
               }
            }
         }
   
         Point3f[] filteredPointArray = new Point3f[filteredPoints.size()];
         filteredPoints.toArray(filteredPointArray);
         return filteredPointArray;
      }
      else
      {
         return new Point3f[0];
      }
   }

   @Override
   public void initialize()
   {
      super.initialize();
      runBlobFilter.set(true);
      coloredCircularBlobDetectorBehaviorService.initialize();
      
      DepthDataStateCommand depthDataStateCommand = new DepthDataStateCommand(LidarState.ENABLE_BEHAVIOR_ONLY);
      depthDataStateCommand.setDestination(PacketDestination.NETWORK_PROCESSOR);
      
      sendPacketToNetworkProcessor(depthDataStateCommand);
      
      TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket("<prosody pitch=\"90Hz\" rate=\"-20%\" volume=\"x-loud\">I am looking for balls.</prosody>");
      textToSpeechPacket.setDestination(PacketDestination.BROADCAST);
      sendPacketToNetworkProcessor(textToSpeechPacket);
   }

   @Override
   public boolean isDone()
   {
      return super.isDone();
   }

   @Override public void pause()
   {
      super.pause();
      runBlobFilter.set(false);
      coloredCircularBlobDetectorBehaviorService.pause();
   }

   @Override public void stop()
   {
      super.stop();
      runBlobFilter.set(false);
      coloredCircularBlobDetectorBehaviorService.stop();
   }

   @Override public void resume()
   {
      super.resume();
      runBlobFilter.set(true);
      coloredCircularBlobDetectorBehaviorService.resume();
   }

   public Point2d getLatestBallPosition()
   {
      return coloredCircularBlobDetectorBehaviorService.getLatestBallPosition2d();
   }

   public int getNumBallsDetected()
   {
      return numBallsDetected;
   }
}
