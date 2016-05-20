package us.ihmc.humanoidBehaviors.behaviors;

import boofcv.struct.calib.IntrinsicParameters;
import bubo.clouds.FactoryPointCloudShape;
import bubo.clouds.detect.CloudShapeTypes;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.clouds.detect.wrapper.ConfigMultiShapeRansac;
import bubo.clouds.detect.wrapper.ConfigSurfaceNormals;
import georegression.struct.point.Point3D_F64;
import georegression.struct.shapes.Sphere3D_F64;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.producers.CompressedVideoDataClient;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.VideoStreamer;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.thread.ThreadTools;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class FollowBallBehavior extends BehaviorInterface implements VideoStreamer
{
   private FootstepDataListMessage outgoingFootstepDataList;
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue;
   private final ConcurrentListeningQueue<WalkingStatusMessage> walkingStatusQueue;
   private FootstepStatus latestFootstepStatus;

   private final SDFFullHumanoidRobotModel fullRobotModel;

   private final double defaultSwingTime;
   private final double defaultTranferTime;

   private BufferedImage lastBufferedImage;
   private final CompressedVideoDataClient videoDataClient;
   private boolean detectBall = false;
   private BallDetectionThread computerVisionThread = new BallDetectionThread();
   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<>();

   private final ReferenceFrame headFrame;
   private static final double FILTERING_ANGLE = Math.toRadians(3.0);
   private static final double FILTERING_DISTANCE = 0.08;

   // http://www.bostondynamics.com/img/MultiSense_SL.pdf
   private static final double HORIZONTAL_FOV = Math.toRadians(80.0);
   private static final double VERTICAL_FOV = Math.toRadians(45.0);

   // sphere detection parameters
   private static final int SPHERE_DECECTION_ITERATIONS = 7;
   private static final double SPHERE_DETECTION_ANGLE_TOLERANCE = 0.914327;
   private static final double SPHERE_DETECTION_RANSAC_DISTANCE_THRESHOLD = 0.0872604;
   private static final int SPHERE_DETECTION_MIN_PTS = 30;
   private static final int SPHERE_DETECTION_NUM_NEIGHBORS = 41;
   private static final double SPHERE_DETECTION_MAX_NEIGHBOR_DIST = 0.098158;

   private final ConcurrentListeningQueue<PointCloudWorldPacket> pointCloudQueue = new ConcurrentListeningQueue<PointCloudWorldPacket>();

   private final Point3d latestBallPosition = new Point3d();

   public FollowBallBehavior(BehaviorCommunicationBridge communicationBridge, WalkingControllerParameters walkingControllerParameters,
         SDFFullHumanoidRobotModel fullRobotModel)
   {
      super(communicationBridge);

      footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>();
      attachControllerListeningQueue(footstepStatusQueue, FootstepStatus.class);
      walkingStatusQueue = new ConcurrentListeningQueue<>();
      attachControllerListeningQueue(walkingStatusQueue, WalkingStatusMessage.class);

      defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      defaultTranferTime = walkingControllerParameters.getDefaultTransferTime();

      videoDataClient = CompressedVideoDataFactory.createCompressedVideoDataClient(this);
      computerVisionThread.start();
      communicationBridge.attachGlobalListener(getNetworkProcessorGlobalObjectConsumer());
      attachNetworkProcessorListeningQueue(videoPacketQueue, VideoPacket.class);
      attachNetworkProcessorListeningQueue(pointCloudQueue, PointCloudWorldPacket.class);

      this.fullRobotModel = fullRobotModel;
      this.headFrame = fullRobotModel.getHead().getBodyFixedFrame();
   }

   private class BallDetectionThread extends Thread
   {
      @Override public void run()
      {
         while (true)
         {
            if (detectBall)
            {
               if (videoPacketQueue.isNewPacketAvailable())
               {
                  VideoPacket packet = videoPacketQueue.poll();
                  while (videoPacketQueue.isNewPacketAvailable())
                  {
                     packet = videoPacketQueue.poll();
                  }

                  videoDataClient.consumeObject(packet.getData(), packet.getPosition(), packet.getOrientation(), packet.getIntrinsicParameters());
               }
               else
               {
                  ThreadTools.sleep(10);
               }
            }
         }
      }
   }

   @Override public void updateImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, IntrinsicParameters intrinsicParamaters)
   {
      this.lastBufferedImage = bufferedImage;
      // set ball center from image
   }

   @Override public void doControl()
   {
      if (checkForNewFootstepStatusPacket())
      {
         generateAndSendPathToBall();
      }
      else
      {
         ThreadTools.sleep(10);
      }
   }

   private boolean checkForNewFootstepStatusPacket()
   {
      if (footstepStatusQueue.isNewPacketAvailable())
      {
         latestFootstepStatus = footstepStatusQueue.poll();
         return true;
      }
      else
      {
         return false;
      }
   }

   private void generateAndSendPathToBall()
   {
      Point3f[] fullPointCloud = null;

      while (pointCloudQueue.isNewPacketAvailable())
         fullPointCloud = pointCloudQueue.poll().getDecayingWorldScan();

      if (fullPointCloud != null)
      {
         fullRobotModel.getHead();
         List<Point3f> filteredPointCloud = filterPointsNearBall(fullPointCloud, headFrame);
         List<Sphere3D_F64> detectedSpheres = detectBalls(filteredPointCloud);
      }
   }

   private static List<Point3f> filterPointsNearBall(Point3f[] fullPointCloud, ReferenceFrame cameraFrame)
   {
      List<Point3f> filteredPoints = new ArrayList<Point3f>();
      RigidBodyTransform worldToCameraTransform = cameraFrame.getTransformToWorldFrame();
      Point3f tempPoint = new Point3f();
      double desiredRayAngleX = 0.0; // TODO determine ray from 2d point
      double desiredRayAngleY = 0.0; // TODO determine ray from 2d point

      for (int i = 0; i < fullPointCloud.length; i++)
      {
         tempPoint.set(fullPointCloud[i]);
         worldToCameraTransform.transform(tempPoint);

         if (tempPoint.z > FILTERING_DISTANCE)
         {
            double rayAngleX = Math.atan2(tempPoint.y, tempPoint.z);
            double rayAngleY = Math.atan2(tempPoint.x, tempPoint.z);
            if (Math.abs(rayAngleX - desiredRayAngleX) < FILTERING_ANGLE && Math.abs(rayAngleY - desiredRayAngleY) < FILTERING_ANGLE)
            {
               filteredPoints.add(fullPointCloud[i]);
            }
         }
      }

      return filteredPoints;
   }

   private static ArrayList<Sphere3D_F64> detectBalls(List<Point3f> pointCloud)
   {
      ArrayList<Sphere3D_F64> foundBalls = new ArrayList<Sphere3D_F64>();
      ArrayList<Point3D_F64> pointsNearBy = new ArrayList<Point3D_F64>();

      for (Point3f tmpPoint : pointCloud)
      {
         pointsNearBy.add(new Point3D_F64(tmpPoint.x, tmpPoint.y, tmpPoint.z));
      }

      // find plane
      ConfigMultiShapeRansac configRansac = ConfigMultiShapeRansac
            .createDefault(SPHERE_DECECTION_ITERATIONS, SPHERE_DETECTION_ANGLE_TOLERANCE, SPHERE_DETECTION_RANSAC_DISTANCE_THRESHOLD, CloudShapeTypes.SPHERE);
      configRansac.minimumPoints = SPHERE_DETECTION_MIN_PTS;
      PointCloudShapeFinder findSpheres = FactoryPointCloudShape
            .ransacSingleAll(new ConfigSurfaceNormals(SPHERE_DETECTION_NUM_NEIGHBORS, SPHERE_DETECTION_MAX_NEIGHBOR_DIST), configRansac);

      // sort large to small
      List<PointCloudShapeFinder.Shape> spheres = findSpheres.getFound();
      Collections.sort(spheres, new Comparator<PointCloudShapeFinder.Shape>()
      {
         @Override public int compare(PointCloudShapeFinder.Shape o1, PointCloudShapeFinder.Shape o2)
         {
            return Integer.compare(o1.points.size(), o2.points.size());
         }
      });

      if (spheres.size() > 0)
      {
//         ballsFound = spheres.size();
//         smallestRadius = ((Sphere3D_F64) spheres.get(0).getParameters()).getRadius();
      }

      for (PointCloudShapeFinder.Shape sphere : spheres)
      {
//         Sphere3D_F64 sphereParams = (Sphere3D_F64) sphere.getParameters();
//         PrintTools.debug(DEBUG, this, "sphere radius" + sphereParams.getRadius() + " center " + sphereParams.getCenter());

//         if ((sphereParams.getRadius() < BALL_RADIUS + 0.025f) && (sphereParams.getRadius() > BALL_RADIUS - 0.025f))// soccer ball -
//         {
//            foundBalls.add(sphereParams);
//            PrintTools.debug(DEBUG, this, "------Found Soccer Ball radius" + sphereParams.getRadius() + " center " + sphereParams.getCenter());
//
//            RigidBodyTransform t = new RigidBodyTransform();
//            t.setTranslation(sphereParams.getCenter().x, sphereParams.getCenter().y, sphereParams.getCenter().z);
//         }

      }

      return foundBalls;
   }

   @Override public void initialize()
   {
      detectBall = true;
   }

   @Override public void pause()
   {
      sendPacketToController(new PauseWalkingMessage(true));
      detectBall = false;
   }

   @Override public void stop()
   {
      sendPacketToController(new PauseWalkingMessage(true));
      detectBall = false;
   }

   @Override public void resume()
   {
      sendPacketToController(new PauseWalkingMessage(false));
      detectBall = true;
   }

   @Override public boolean isDone()
   {
      return false;
   }

   @Override protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
   }

   @Override protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
   }

   @Override public void enableActions()
   {
   }

   @Override public void doPostBehaviorCleanup()
   {
   }

   @Override public boolean hasInputBeenSet()
   {
      return true;
   }
}
