package us.ihmc.perception;

import us.ihmc.bytedeco.mapsenseWrapper.VisualOdometry;
import us.ihmc.bytedeco.slamWrapper.SlamWrapper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.elements.CameraModel;

import java.util.ArrayList;
import java.util.Arrays;

public class VisualSLAMModule
{
   public static final int MAX_LANDMARKS = 300;

   private final VisualOdometry.VisualOdometryExternal visualOdometryExternal;
   private final SlamWrapper.FactorGraphExternal factorGraphExternal;

   RigidBodyTransform worldToSensorTransform = new RigidBodyTransform();

   private int frameIndex = 0;
   private boolean initialized = false;

   public VisualSLAMModule()
   {
      BytedecoTools.loadMapsenseLibraries();
      visualOdometryExternal = new VisualOdometry.VisualOdometryExternal();

      BytedecoTools.loadGTSAMLibraries();
      factorGraphExternal = new SlamWrapper.FactorGraphExternal();

      float[] poseInitial = new float[]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      LogTools.info("Inserting Prior Pose Factor: x{}", 0);
      factorGraphExternal.addPriorPoseFactor(0, poseInitial);
      factorGraphExternal.setPoseInitialValue(0, poseInitial);
   }


   public void update(ImageMat leftImage, ImageMat rightImage)
   {
      visualOdometryExternal.updateStereo(leftImage.getData(), rightImage.getData(), leftImage.getRows(), leftImage.getCols());

      LogTools.info("Extracting Keyframe External");
      double[] relativePoseTransformAsArray = new double[16];
      int[] keyframeID = new int[]{-1};
      visualOdometryExternal.getExternalKeyframe(relativePoseTransformAsArray, keyframeID);
      LogTools.info("Relative Pose: x{} -> {}", keyframeID[0], Arrays.toString(relativePoseTransformAsArray));

      LogTools.info("Inserting Odometry Factor: x{}", keyframeID[0] + 1);
      factorGraphExternal.addOdometryFactorExtended(relativePoseTransformAsArray, keyframeID[0] + 1);

      LogTools.info("Extracting Landmarks External");
      /* Extract keyframe and landmark measurements from visual odometry module. */
      float[] landmarks = new float[5 * MAX_LANDMARKS];
      int[] landmarkIDs = new int[MAX_LANDMARKS];
      int totalLandmarks = visualOdometryExternal.getExternalLandmarks(landmarks, landmarkIDs, MAX_LANDMARKS);
      LogTools.info("Total Landmarks: {}", totalLandmarks);
      LogTools.info("Landmarks: {}", Arrays.toString(landmarks));
      LogTools.info("LandmarkIDs: {}", Arrays.toString(landmarkIDs));

      LogTools.info("Transforming Point to World Frame");
      /* Update sensorToWorldTransform based on last sensor pose and most recent odometry */
      RigidBodyTransform odometryTransform = new RigidBodyTransform();
      odometryTransform.set(relativePoseTransformAsArray);
      worldToSensorTransform.multiply(odometryTransform);
      LogTools.info("Odometry: {}", odometryTransform);
      LogTools.info("Sensor Transform: {}", worldToSensorTransform);

      /* Compute and insert keyframe pose in world frame as initial value */
      float[] poseValue = new float[16];
      worldToSensorTransform.get(poseValue);
      LogTools.info("World to Sensor Transform: {}", worldToSensorTransform);
      LogTools.info("Setting Initial Pose Value: x{} -> ", keyframeID[0] + 1, Arrays.toString(poseValue));
      factorGraphExternal.setPoseInitialValueExtended(keyframeID[0] + 1, poseValue);

      /* Insert landmarks and initial estimates into the factor graph. */
      int totalValidLandmarks = 0;
      for(int i = 0; i<totalLandmarks; i++)
      {
         if(landmarkIDs[i] != -1)
         {
            /* Insert generic projection factor for each landmark measurement on left camera. */
            LogTools.info("Inserting Projection Factor: x{} -> p{}", keyframeID[0] + 1, landmarkIDs[i]);
            factorGraphExternal.addGenericProjectionFactor(new float[]{landmarks[i*5], landmarks[i*5 + 1]}, landmarkIDs[i], keyframeID[0] + 1);

            /* Compute the 3D point initial value in world frame. */
            Point3D pointInWorld = new Point3D(landmarks[i*5 + 2], landmarks[i*5 + 3], landmarks[i*5 + 4]);
            worldToSensorTransform.transform(pointInWorld);

            LogTools.info("Setting landmark initial value.");
            factorGraphExternal.setPointLandmarkInitialValue(landmarkIDs[i], new float[]{pointInWorld.getX32(), pointInWorld.getY32(), pointInWorld.getZ32()});

            totalValidLandmarks++;
         }
      }
      LogTools.info("Total Valid Landmarks: {}", totalValidLandmarks);

      LogTools.info("Optimizing Factor Graph");
      // TODO: Try Incremental SAM instead of batch optimizer.
      factorGraphExternal.optimize();
      //factorGraphExternal.printResults();

      frameIndex++;
   }

   public FramePose3D getSensorPose(int index)
   {
      double[] poses = new double[16];
      int[] indices = new int[]{index};
      factorGraphExternal.getResultPoses(poses, indices, 1);

      LogTools.info("Array: {}", Arrays.toString(poses));

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.set(poses);

      FramePose3D pose = new FramePose3D();
      pose.set(transform);
      return pose;
   }

   public void visualSLAMUpdate()
   {
      float[] angles = new float[]{1.5708f, 2.35619f, 3.14159f, -2.35619f, -1.5708f, -0.785398f, -1.83697e-16f, 0.785398f};

      double radius = 30.0;
      int i = 0;
      double theta = 0.0;
      Point3D up = new Point3D(0, 0, 1);
      Point3D target = new Point3D(0, 0, 0);

      ArrayList<Point3D> points = new ArrayList<>();
      points.add(new Point3D(10.0,10.0,10.0));
      points.add(new Point3D(-10.0,10.0,10.0));
      points.add(new Point3D(-10.0,-10.0,10.0));
      points.add(new Point3D(10.0,-10.0,10.0));
      points.add(new Point3D(10.0,10.0,-10.0));
      points.add(new Point3D(-10.0,10.0,-10.0));
      points.add(new Point3D(-10.0,-10.0,-10.0));
      points.add(new Point3D(10.0,-10.0,-10.0));

      for (; i < 8; ++i, theta += 2 * Math.PI / 8)
      {
         Pose3D pose = new Pose3D();

         CameraModel camera = new CameraModel(50.0f,50.0f,50.0f,50.0f, pose);

         Point3D position = new Point3D(radius * Math.cos(theta), radius * Math.sin(theta), 0.0);

         Point2D measurement = camera.project(points.get(i));

         LogTools.info("Projection: {}", measurement);


      }
   }

   public void render()
   {

   }

   public static void main(String[] args)
   {
      VisualSLAMModule vslam = new VisualSLAMModule();

      String LEFT_CAMERA_NAME = "image_0";
      String RIGHT_CAMERA_NAME = "image_1";

      String DATASET_PATH = "/home/quantum/Workspace/Data/Datasets/sequences/00/";

      String fileName = String.format("%1$6s", 0).replace(' ', '0') + ".png";
      String leftImageName = DATASET_PATH + LEFT_CAMERA_NAME + "/" + fileName;
      String rightImageName = DATASET_PATH + RIGHT_CAMERA_NAME + "/" + fileName;

      ImageMat currentImageLeft = ImageTools.loadAsImageMat(leftImageName);
      ImageMat currentImageRight = ImageTools.loadAsImageMat(rightImageName);

      vslam.update(currentImageLeft, currentImageRight);
   }

}
