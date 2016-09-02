package us.ihmc.humanoidBehaviors.behaviors;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import javax.vecmath.Point3f;

import georegression.struct.point.Point3D_F64;
import georegression.struct.shapes.Sphere3D_F64;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.ColoredCircularBlobDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.DetectedObjectPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.ihmcPerception.vision.HSVValue;
import us.ihmc.ihmcPerception.vision.shapes.HSVRange;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.bubo.clouds.FactoryPointCloudShape;
import us.ihmc.sensorProcessing.bubo.clouds.detect.CloudShapeTypes;
import us.ihmc.sensorProcessing.bubo.clouds.detect.PointCloudShapeFinder;
import us.ihmc.sensorProcessing.bubo.clouds.detect.wrapper.ConfigMultiShapeRansac;
import us.ihmc.sensorProcessing.bubo.clouds.detect.wrapper.ConfigSurfaceNormals;

public class FollowBallBehavior extends AbstractBehavior
{
   private static final boolean DEBUG = true;

   private FootstepDataListMessage outgoingFootstepDataList;
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue;
   private final ConcurrentListeningQueue<WalkingStatusMessage> walkingStatusQueue;
   private FootstepStatus latestFootstepStatus;

   private final SDFFullHumanoidRobotModel fullRobotModel;

   private final double swingTime = 0.6;
   private final double tranferTime = 0.5;

   private final ColoredCircularBlobDetectorBehaviorService coloredCircularBlobDetectorBehaviorService;
   
   private BufferedImage lastBufferedImage;
   
   private final ReferenceFrame headFrame;
   private static final double FILTERING_ANGLE = Math.toRadians(5.0);
   private static final double FILTERING_MIN_DISTANCE = 0.1;
   private static final double FILTERING_MAX_DISTANCE = 7.0;

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

   private static final double MIN_BALL_RADIUS = 0.05;
   private static final double MAX_BALL_RADIUS = 0.15;

   private final ConcurrentListeningQueue<PointCloudWorldPacket> pointCloudQueue = new ConcurrentListeningQueue<PointCloudWorldPacket>();

   private final PointCloudShapeFinder pointCloudSphereFinder;

   public FollowBallBehavior(BehaviorCommunicationBridge behaviorCommunicationBridge, SDFFullHumanoidRobotModel fullRobotModel)
   {
      super(behaviorCommunicationBridge);

      footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>();
      attachControllerListeningQueue(footstepStatusQueue, FootstepStatus.class);
      walkingStatusQueue = new ConcurrentListeningQueue<>();
      attachControllerListeningQueue(walkingStatusQueue, WalkingStatusMessage.class);

      coloredCircularBlobDetectorBehaviorService = new ColoredCircularBlobDetectorBehaviorService(this);
      HSVRange greenRange = new HSVRange(new HSVValue(78, 100, 100), new HSVValue(83, 255, 255));
      coloredCircularBlobDetectorBehaviorService.addHSVRange(greenRange);
      
      behaviorCommunicationBridge.attachGlobalListener(getNetworkProcessorGlobalObjectConsumer());
      attachNetworkProcessorListeningQueue(pointCloudQueue, PointCloudWorldPacket.class);

      this.fullRobotModel = fullRobotModel;
      this.headFrame = fullRobotModel.getHead().getBodyFixedFrame();

      ConfigMultiShapeRansac configRansac = ConfigMultiShapeRansac
            .createDefault(SPHERE_DECECTION_ITERATIONS, SPHERE_DETECTION_ANGLE_TOLERANCE, SPHERE_DETECTION_RANSAC_DISTANCE_THRESHOLD, CloudShapeTypes.SPHERE);
      configRansac.minimumPoints = SPHERE_DETECTION_MIN_PTS;
      pointCloudSphereFinder = FactoryPointCloudShape
            .ransacSingleAll(new ConfigSurfaceNormals(SPHERE_DETECTION_NUM_NEIGHBORS, SPHERE_DETECTION_MAX_NEIGHBOR_DIST), configRansac);
   }

   @Override
   public void doControl()
   {
      generateAndSendPathToBall();

      //      if (checkForNewFootstepStatusPacket())
//      {
//         generateAndSendPathToBall();
//      }
//      else
//      {
//         ThreadTools.sleep(10);
//      }
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

   private static int counter = 0;
   private static int id = 0;

   private void generateAndSendPathToBall()
   {
      Point3f[] fullPointCloud = null;

      while (pointCloudQueue.isNewPacketAvailable())
         fullPointCloud = pointCloudQueue.poll().getDecayingWorldScan();

      if (fullPointCloud != null)
      {
         fullRobotModel.getHead();
         List<Point3f> filteredPointCloud = filterPointsNearBall(fullPointCloud);

         if (DEBUG)
            System.out.println("starting sphere detection");

         long startTime = System.currentTimeMillis();
         List<Sphere3D_F64> detectedSpheres = detectSpheres(filteredPointCloud, pointCloudSphereFinder);
         long stopTime = System.currentTimeMillis();
         long detectionTime = stopTime - startTime;

         if(DEBUG)
         {
            System.out.println("sphere detection found: " + detectedSpheres.size() + " spheres");
            System.out.println("\t detection time: " + detectionTime);

            for (int i = 0; i < detectedSpheres.size(); i++)
               System.out.println("\t center: " + detectedSpheres.get(i).getCenter());
         }

         for (Sphere3D_F64 ball : detectedSpheres)
         {
            RigidBodyTransform ballTransform = new RigidBodyTransform();
            ballTransform.setTranslation(ball.getCenter().x, ball.getCenter().y, ball.getCenter().z);
            sendPacketToNetworkProcessor(new DetectedObjectPacket(ballTransform, id++));
         }
      }
   }

   private List<Point3f> filterPointsNearBall(Point3f[] fullPointCloud)
   {
      List<Point3f> filteredPoints = new ArrayList<Point3f>();
      RigidBodyTransform worldToCameraTransform = headFrame.getTransformToWorldFrame();
      worldToCameraTransform.invert();

      int cameraPixelWidth = lastBufferedImage.getWidth();
      int cameraPixelHeight = lastBufferedImage.getHeight();

      double ballCenterX = coloredCircularBlobDetectorBehaviorService.getLatestBallPosition2d().getX() - lastBufferedImage.getMinX();
      double ballCenterY = coloredCircularBlobDetectorBehaviorService.getLatestBallPosition2d().getY() - lastBufferedImage.getMinY();

      double desiredRayAngleX = VERTICAL_FOV * (- ballCenterY / cameraPixelHeight + 0.5);
      double desiredRayAngleY = HORIZONTAL_FOV * (ballCenterX / cameraPixelWidth - 0.5);

      Point3f tempPoint = new Point3f();

      for (int i = 0; i < fullPointCloud.length; i++)
      {
         tempPoint.set(fullPointCloud[i]);
         worldToCameraTransform.transform(tempPoint);

         if (tempPoint.getX() > FILTERING_MIN_DISTANCE && tempPoint.getX() < FILTERING_MAX_DISTANCE)
         {
            // rayAngle axes are in terms of the buffered image (y-down), temp pnt axes are in terms of camera frame (z-up)
            double rayAngleX = Math.atan2(tempPoint.getZ(), tempPoint.getX());
            double rayAngleY = Math.atan2(tempPoint.getY(), tempPoint.getX());
            if (Math.abs(rayAngleX - desiredRayAngleX) < FILTERING_ANGLE && Math.abs(rayAngleY - desiredRayAngleY) < FILTERING_ANGLE)
            {
               filteredPoints.add(fullPointCloud[i]);
            }
         }
      }

      return filteredPoints;
   }

   private static ArrayList<Sphere3D_F64> detectSpheres(List<Point3f> pointCloud, PointCloudShapeFinder pointCloudSphereFinder)
   {
      ArrayList<Sphere3D_F64> foundBalls = new ArrayList<Sphere3D_F64>();
      ArrayList<Point3D_F64> pointsNearBy = new ArrayList<Point3D_F64>();

      for (int i = 0; i < pointCloud.size(); i++)
      {
         Point3f tmpPoint = pointCloud.get(i);
         pointsNearBy.add(new Point3D_F64(tmpPoint.getX(), tmpPoint.getY(), tmpPoint.getZ()));
      }

      pointCloudSphereFinder.process(pointsNearBy, null);

      // sort large to small
      List<PointCloudShapeFinder.Shape> spheres = pointCloudSphereFinder.getFound();
      Collections.sort(spheres, new Comparator<PointCloudShapeFinder.Shape>()
      {
         @Override public int compare(PointCloudShapeFinder.Shape o1, PointCloudShapeFinder.Shape o2)
         {
            return Integer.compare(o1.points.size(), o2.points.size());
         }
      });

      for (int i = 0; i < spheres.size(); i++)
      {
         PointCloudShapeFinder.Shape sphere = spheres.get(i);
         Sphere3D_F64 sphereParams = sphere.getParameters();

         if ((sphereParams.getRadius() < MAX_BALL_RADIUS) && (sphereParams.getRadius() > MIN_BALL_RADIUS))
            foundBalls.add(sphereParams);
      }

      return foundBalls;
   }

   @Override
   public void initialize()
   {
      coloredCircularBlobDetectorBehaviorService.initialize();
   }

   @Override
   public void pause()
   {
      sendPacketToController(new PauseWalkingMessage(true));
      coloredCircularBlobDetectorBehaviorService.pause();
   }

   @Override
   public void stop()
   {
      sendPacketToController(new PauseWalkingMessage(true));
      coloredCircularBlobDetectorBehaviorService.stop();
   }

   @Override
   public void resume()
   {
      sendPacketToController(new PauseWalkingMessage(false));
      coloredCircularBlobDetectorBehaviorService.resume();
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
   }

   @Override
   public void enableActions()
   {
   }

   @Override
   public void doPostBehaviorCleanup()
   {
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return true;
   }
}
