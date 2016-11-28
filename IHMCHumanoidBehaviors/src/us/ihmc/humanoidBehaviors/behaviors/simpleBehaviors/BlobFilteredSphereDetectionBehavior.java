package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3f;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.ColoredCircularBlobDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.ihmcPerception.vision.shapes.HSVRange;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class BlobFilteredSphereDetectionBehavior extends SphereDetectionBehavior
{
   private final ReferenceFrame headFrame;
   private static final double FILTERING_ANGLE = Math.toRadians(5.0);
   private static final double FILTERING_MIN_DISTANCE = 0.1;
   private static final double FILTERING_MAX_DISTANCE = 7.0;

   // http://www.bostondynamics.com/img/MultiSense_SL.pdf
   private static final double MUTLISENSE_HORIZONTAL_FOV = Math.toRadians(80.0);
   private static final double MULTISENSE_VERTICAL_FOV = Math.toRadians(45.0);
   
   private final ColoredCircularBlobDetectorBehaviorService coloredCircularBlobDetectorBehaviorService;
   
   public BlobFilteredSphereDetectionBehavior(CommunicationBridge behaviorCommunicationBridge, HumanoidReferenceFrames referenceFrames,
         FullHumanoidRobotModel fullRobotModel)
   {
      super(behaviorCommunicationBridge, referenceFrames);

      attachNetworkListeningQueue(pointCloudQueue, PointCloudWorldPacket.class);

      coloredCircularBlobDetectorBehaviorService = new ColoredCircularBlobDetectorBehaviorService(behaviorCommunicationBridge);

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
      if (pointCloudQueue.isNewPacketAvailable())
      {
         PointCloudWorldPacket latestPointCloudWorldPacket = pointCloudQueue.getLatestPacket();
         Point3f[] fullPointCloud = latestPointCloudWorldPacket.getDecayingWorldScan();

         Point3f[] filteredPointCloud = filterPointsNearBall(fullPointCloud);

         
         findBallsAndSaveResult(filteredPointCloud);
      }
   }

   private Point3f[] filterPointsNearBall(Point3f[] fullPointCloud)
   {
      if (fullPointCloud.length == 0)
      {
         DepthDataStateCommand enableBehaviorLidar = new DepthDataStateCommand(LidarState.ENABLE_BEHAVIOR_ONLY);
         enableBehaviorLidar.setDestination(PacketDestination.SENSOR_MANAGER);
         sendPacket(enableBehaviorLidar);
         
         ThreadTools.sleep(100);
         
         return new Point3f[0];
      }
      

      List<Point3f> filteredPoints = new ArrayList<Point3f>();
      RigidBodyTransform worldToCameraTransform = headFrame.getTransformToWorldFrame();
      worldToCameraTransform.invert();

      BufferedImage latestCameraImage = coloredCircularBlobDetectorBehaviorService.getLatestUnmodifiedCameraImage();
      if (latestCameraImage != null)
      {
         int cameraPixelWidth = latestCameraImage.getWidth();
         int cameraPixelHeight = latestCameraImage.getHeight();
         
         synchronized (coloredCircularBlobDetectorBehaviorService.getBallListConch())
         {
            for (Point2d latestBallPosition2d : coloredCircularBlobDetectorBehaviorService.getLatestBallPositionSet())
            {
               double ballCenterX = latestBallPosition2d.getX() - latestCameraImage.getMinX();
               double ballCenterY = latestBallPosition2d.getY() - latestCameraImage.getMinY();

               double desiredRayAngleX = MULTISENSE_VERTICAL_FOV * (-ballCenterY / cameraPixelHeight + 0.5);
               double desiredRayAngleY = MUTLISENSE_HORIZONTAL_FOV * (ballCenterX / cameraPixelWidth - 0.5);

               Point3f candidateLidarPoint = new Point3f();

               for (int i = 0; i < fullPointCloud.length; i++)
               {

                  candidateLidarPoint.set(fullPointCloud[i]);
                  worldToCameraTransform.transform(candidateLidarPoint);

                  if (candidateLidarPoint.getX() > FILTERING_MIN_DISTANCE && candidateLidarPoint.getX() < FILTERING_MAX_DISTANCE)
                  {
                     // rayAngle axes are in terms of the buffered image (y-down), temp pnt axes are in terms of camera frame (z-up)
                     double rayAngleX = Math.atan2(candidateLidarPoint.getZ(), candidateLidarPoint.getX());
                     double rayAngleY = Math.atan2(candidateLidarPoint.getY(), candidateLidarPoint.getX());
                     if (Math.abs(rayAngleX - desiredRayAngleX) < FILTERING_ANGLE && Math.abs(rayAngleY - desiredRayAngleY) < FILTERING_ANGLE)
                     {
                        filteredPoints.add(fullPointCloud[i]);
                     }
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
         PrintTools.debug(this, "LatestCameraImage is null. Returning empty");
         return new Point3f[0];
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      coloredCircularBlobDetectorBehaviorService.run();
      
      DepthDataStateCommand depthDataStateCommand = new DepthDataStateCommand(LidarState.ENABLE_BEHAVIOR_ONLY);
      depthDataStateCommand.setDestination(PacketDestination.SENSOR_MANAGER);
      
      sendPacket(depthDataStateCommand);
      
      TextToSpeechPacket textToSpeechPacket = new TextToSpeechPacket("<prosody pitch=\"90Hz\" rate=\"-20%\" volume=\"x-loud\">I am looking for balls.</prosody>");
      textToSpeechPacket.setDestination(PacketDestination.TEXT_TO_SPEECH);
      sendPacket(textToSpeechPacket);
   }

   @Override
   public boolean isDone()
   {
      return super.isDone();
   }

   @Override
   public void onBehaviorPaused()
   {
      coloredCircularBlobDetectorBehaviorService.pause();
   }

   @Override
   public void onBehaviorAborted()
   {
      coloredCircularBlobDetectorBehaviorService.pause();
   }

   @Override
   public void onBehaviorResumed()
   {
      coloredCircularBlobDetectorBehaviorService.run();
   }

   public Point2d getLatestBallPosition()
   {
      return coloredCircularBlobDetectorBehaviorService.getLatestBallPosition2d();
   }

   public int getNumBallsDetected()
   {
      return coloredCircularBlobDetectorBehaviorService.getLatestBallPositionSet().size();
   }
}
