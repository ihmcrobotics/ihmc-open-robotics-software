package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.PointCloudWorldPacket;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.ColoredCircularBlobDetectorBehaviorService;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.ihmcPerception.vision.shapes.HSVRange;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.Ros2Node;

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

   public BlobFilteredSphereDetectionBehavior(String robotName, Ros2Node ros2Node, HumanoidReferenceFrames referenceFrames,
                                              FullHumanoidRobotModel fullRobotModel)
   {
      super(robotName, ros2Node, referenceFrames);

      createSubscriber(PointCloudWorldPacket.class, ROS2Tools.getDefaultTopicNameGenerator(), pointCloudQueue::put); // FIXME That stream is no more

      coloredCircularBlobDetectorBehaviorService = new ColoredCircularBlobDetectorBehaviorService(robotName, ros2Node);

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
         Point3D32[] fullPointCloud = HumanoidMessageTools.getDecayingWorldScan(latestPointCloudWorldPacket);

         Point3D32[] filteredPointCloud = filterPointsNearBall(fullPointCloud);

         findBallsAndSaveResult(filteredPointCloud);
      }
   }

   private Point3D32[] filterPointsNearBall(Point3D32[] fullPointCloud)
   {
      if (fullPointCloud.length == 0)
      { // FIXME
         //         DepthDataStateCommand enableBehaviorLidar = new DepthDataStateCommand(LidarState.ENABLE_BEHAVIOR_ONLY);
         //         enableBehaviorLidar.setDestination(PacketDestination.SENSOR_MANAGER);
         //         sendPacket(enableBehaviorLidar);

         ThreadTools.sleep(100);

         return new Point3D32[0];
      }

      List<Point3D32> filteredPoints = new ArrayList<Point3D32>();
      RigidBodyTransform worldToCameraTransform = headFrame.getTransformToWorldFrame();
      worldToCameraTransform.invert();

      BufferedImage latestCameraImage = coloredCircularBlobDetectorBehaviorService.getLatestUnmodifiedCameraImage();
      if (latestCameraImage != null)
      {
         int cameraPixelWidth = latestCameraImage.getWidth();
         int cameraPixelHeight = latestCameraImage.getHeight();

         synchronized (coloredCircularBlobDetectorBehaviorService.getBallListConch())
         {
            for (Point2D latestBallPosition2d : coloredCircularBlobDetectorBehaviorService.getLatestBallPositionSet())
            {
               double ballCenterX = latestBallPosition2d.getX() - latestCameraImage.getMinX();
               double ballCenterY = latestBallPosition2d.getY() - latestCameraImage.getMinY();

               double desiredRayAngleX = MULTISENSE_VERTICAL_FOV * (-ballCenterY / cameraPixelHeight + 0.5);
               double desiredRayAngleY = MUTLISENSE_HORIZONTAL_FOV * (ballCenterX / cameraPixelWidth - 0.5);

               Point3D32 candidateLidarPoint = new Point3D32();

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

         Point3D32[] filteredPointArray = new Point3D32[filteredPoints.size()];
         filteredPoints.toArray(filteredPointArray);

         return filteredPointArray;
      }
      else
      {
         PrintTools.debug(this, "LatestCameraImage is null. Returning empty");
         return new Point3D32[0];
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      coloredCircularBlobDetectorBehaviorService.run();

      // FIXME
      //      DepthDataStateCommand depthDataStateCommand = new DepthDataStateCommand(LidarState.ENABLE_BEHAVIOR_ONLY);
      //      depthDataStateCommand.setDestination(PacketDestination.SENSOR_MANAGER);
      //      sendPacket(depthDataStateCommand);

      publishTextToSpeech("<prosody pitch=\"90Hz\" rate=\"-20%\" volume=\"x-loud\">I am looking for balls.</prosody>");
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

   public Point2D getLatestBallPosition()
   {
      return coloredCircularBlobDetectorBehaviorService.getLatestBallPosition2d();
   }

   public int getNumBallsDetected()
   {
      return coloredCircularBlobDetectorBehaviorService.getLatestBallPositionSet().size();
   }
}
