package us.ihmc.perception;

import org.apache.commons.lang.ArrayUtils;
import us.ihmc.perception.visualOdometry.VisualOdometry;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.elements.CameraModel;
import us.ihmc.perception.elements.Keyframe;
import us.ihmc.perception.elements.Landmark;
import us.ihmc.perception.slamWrapper.SlamWrapper;

import java.util.*;

public class VisualSLAMModule
{
   public static final int MAX_LANDMARKS = 400;
   public static final int NUMBER_OF_FEATURES = 400;
   public static final int MIN_NUM_FEATURES = 350;

   private final VisualOdometry.VisualOdometryExternal visualOdometryExternal;
   private final SlamWrapper.FactorGraphExternal factorGraphExternal;

   RigidBodyTransform worldToSensorTransform = new RigidBodyTransform();

   private int frameIndex = 0;
   private boolean initialized = false;

   private double[] landmarkPoints;
   private int[] idInts;
   private double[] latestSensorPose = new double[16];
   private HashMap<Integer, Keyframe> keyframes = new HashMap<>();
   private HashMap<Integer, Landmark> landmarks = new HashMap<>();

   public VisualSLAMModule()
   {
      visualOdometryExternal = new VisualOdometry.VisualOdometryExternal(NUMBER_OF_FEATURES, MIN_NUM_FEATURES);

      factorGraphExternal = new SlamWrapper.FactorGraphExternal();

      double[] poseInitial = new double[] {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      //LogTools.info("Inserting Prior Pose Factor: x{}", 0);
      factorGraphExternal.addPriorPoseFactor(0, poseInitial);
      factorGraphExternal.setPoseInitialValue(0, poseInitial);

      keyframes.put(0, new Keyframe(0));
   }

   /*
    *  - Calls the updateStereo() on VisualOdometryExternal to generate visual keypoint measurements and odometry estimate.
    *  - Inserts the landmark measurements and odometry as constraints in the factor graph.
    *  - Performs factor graph optimization and extracts results from the GTSAM wrapper FactorGraphExternal.
    * */
   public boolean update(ImageMat leftImage, ImageMat rightImage)
   {
      /*
       *  Compute both relative pose in last key-frame, and 2D-3D keypoint landmark measurements in current camera frame.
       */
      LogTools.info("Update Stereo(Dims: {}, {})", leftImage.getRows(), leftImage.getCols());
      boolean initialized = visualOdometryExternal.updateStereo(leftImage.getData(),
                                                                rightImage.getData(),
                                                                leftImage.getRows(),
                                                                leftImage.getCols(),
                                                                latestSensorPose,
                                                                idInts,
                                                                landmarkPoints,
                                                                idInts.length);

      LogTools.info("Extracting Keyframe External");
      double[] relativePoseTransformAsArray = new double[16];
      int[] keyframeID = new int[] {-1};
      visualOdometryExternal.getExternalKeyframe(relativePoseTransformAsArray, keyframeID);
      //LogTools.info("Relative Pose: x{} -> x{} = [{}]", keyframeID[0], keyframeID[0] + 1, Arrays.toString(relativePoseTransformAsArray));

      if (!keyframes.containsKey(keyframeID[0]))
      {
         keyframes.put(keyframeID[0], new Keyframe(keyframeID[0]));
      }

      if (initialized)
      {
         //LogTools.info("Inserting Odometry Factor: x{}", keyframeID[0]);
         factorGraphExternal.addOdometryFactorSE3(keyframeID[0], relativePoseTransformAsArray);

         //LogTools.info("Compute transform world-to-sensor.");
         /* Update sensorToWorldTransform based on last sensor pose and most recent odometry */
         RigidBodyTransform odometryTransform = new RigidBodyTransform();
         odometryTransform.set(relativePoseTransformAsArray);
         worldToSensorTransform.multiply(odometryTransform);
         LogTools.info("Odometry: {}", odometryTransform);
         //LogTools.info("Sensor Transform: {}", worldToSensorTransform);

         /* Compute and insert keyframe pose in world frame as initial value */
         double[] poseValue = new double[16];
         worldToSensorTransform.get(poseValue);
         //LogTools.info("World to Sensor Transform: {}", worldToSensorTransform);
         LogTools.info("Setting Initial Pose Value: x{} = [{}]", keyframeID[0], Arrays.toString(poseValue));
         factorGraphExternal.setPoseInitialValueSE3(keyframeID[0], poseValue);
      }

      //LogTools.info("Extracting Landmarks External");
      /* Extract keyframe and landmark measurements from visual odometry module. */
      float[] landmarkFloats = new float[5 * MAX_LANDMARKS];
      int[] landmarkIDs = new int[MAX_LANDMARKS];
      int totalLandmarks = visualOdometryExternal.getExternalLandmarks(landmarkFloats, landmarkIDs, MAX_LANDMARKS);
      LogTools.info("Total Landmarks: {}", totalLandmarks);
      //LogTools.info("Landmarks: {}", Arrays.toString(landmarks));
      //LogTools.info("LandmarkIDs: {}", Arrays.toString(landmarkIDs));


      /* Insert landmarks and initial estimates into the factor graph. */
      int totalValidLandmarks = 0;
      for (int i = 0; i < totalLandmarks; i++)
      {
         if (landmarkIDs[i] != -1) // Only matched keypoints to be inserted
         {
            if (!landmarks.containsKey(landmarkIDs[i]))
            {
               landmarks.put(landmarkIDs[i], new Landmark(landmarkIDs[i]));
            }

            keyframes.get(keyframeID[0]).addLandmark(landmarkIDs[i]);
            landmarks.get(landmarkIDs[i]).addKeyframe(keyframeID[0]);

            LogTools.info("Keyframe: {}, Landmark: {}, LandmarkMeasurements: {}",
                          keyframeID[0],
                          landmarkIDs[i],
                          landmarks.get(landmarkIDs[i]).getKeyframeCount());
            //LogTools.info("Inserting Landmark: {}, Count: {}", landmarkIDs[i], landmarks.get(landmarkIDs[i]).getKeyframeCount());

            /* Insert generic projection factor for each landmark measurement on left camera. */
            //LogTools.info("Inserting Projection Factor: x{} -> p{}", keyframeID[0], landmarkIDs[i]);
            factorGraphExternal.addGenericProjectionFactor(new float[] {landmarkFloats[i * 5], landmarkFloats[i * 5 + 1]}, landmarkIDs[i], keyframeID[0]);

            /* Compute the 3D point initial value in world frame. */
            Point3D pointInWorld = new Point3D(landmarkFloats[i * 5 + 2], landmarkFloats[i * 5 + 3], landmarkFloats[i * 5 + 4]);
            worldToSensorTransform.transform(pointInWorld);

            //LogTools.info("Setting landmark initial value.");
            factorGraphExternal.setPointLandmarkInitialValue(landmarkIDs[i], new float[] {pointInWorld.getX32(), pointInWorld.getY32(), pointInWorld.getZ32()});

            totalValidLandmarks++;
         }
      }
      LogTools.info("Total Valid Landmarks: {}", totalValidLandmarks);

      LogTools.info("Optimizing Factor Graph");
      // TODO: Try Incremental SAM instead of batch optimizer.
      if (initialized)
      {
         factorGraphExternal.optimize();

         factorGraphExternal.getPoseById(keyframeID[0], latestSensorPose);
         getLandmarkPoints(landmarks.keySet());

         //factorGraphExternal.optimizeISAM2((byte) 4);
         //factorGraphExternal.optimize();
      }

      LogTools.info("Obtaining Results");
      //factorGraphExternal.printResults();

      frameIndex++;

      LogTools.info("Visual SLAM Update End");
      return initialized;
   }

   public void clearISAM2()
   {
      //factorGraphExternal.clearISAM2();
   }

   public FramePose3D getSensorPose(int index)
   {
      double[] poses = new double[16];
      int[] indices = new int[] {index};
      factorGraphExternal.getResultPoses(poses, indices, 1);

      ////LogTools.info("Array: {}", Arrays.toString(poses));

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.set(poses);

      FramePose3D pose = new FramePose3D();
      pose.set(transform);
      return pose;
   }

   public ArrayList<Point3D> getLandmarkPoints(Set<Integer> ids)
   {
      Integer[] idIntObjects = (Integer[]) ids.toArray();
      idInts = ArrayUtils.toPrimitive(idIntObjects);
      landmarkPoints = new double[idInts.length * 3];
      factorGraphExternal.getResultLandmarks(landmarkPoints, idInts, idInts.length);

      ArrayList<Point3D> points = new ArrayList<>();
      for (int i = 0; i < idInts.length; i++)
      {
         points.add(new Point3D(landmarkPoints[i * 3], landmarkPoints[i * 3 + 1], landmarkPoints[i * 3 + 2]));
      }
      return points;
   }

   public void visualSLAMUpdate()
   {
      float[] angles = new float[] {1.5708f, 2.35619f, 3.14159f, -2.35619f, -1.5708f, -0.785398f, -1.83697e-16f, 0.785398f};

      double radius = 30.0;
      int i = 0;
      double theta = 0.0;
      Point3D up = new Point3D(0, 0, 1);
      Point3D target = new Point3D(0, 0, 0);

      ArrayList<Point3D> points = new ArrayList<>();
      points.add(new Point3D(10.0, 10.0, 10.0));
      points.add(new Point3D(-10.0, 10.0, 10.0));
      points.add(new Point3D(-10.0, -10.0, 10.0));
      points.add(new Point3D(10.0, -10.0, 10.0));
      points.add(new Point3D(10.0, 10.0, -10.0));
      points.add(new Point3D(-10.0, 10.0, -10.0));
      points.add(new Point3D(-10.0, -10.0, -10.0));
      points.add(new Point3D(10.0, -10.0, -10.0));

      for (; i < 8; ++i, theta += 2 * Math.PI / 8)
      {
         Pose3D pose = new Pose3D();

         CameraModel camera = new CameraModel(50.0f, 50.0f, 50.0f, 50.0f, pose);

         Point3D position = new Point3D(radius * Math.cos(theta), radius * Math.sin(theta), 0.0);

         Point2D measurement = camera.project(points.get(i));

         //LogTools.info("Projection: {}", measurement);

      }
   }

   public void render()
   {

   }

   public Set<Integer> getLandmarkKeys()
   {
      return landmarks.keySet();
   }

   //public static void main(String[] args)
   //{
   //   VisualSLAMModule vslam = new VisualSLAMModule();
   //
   //   String LEFT_CAMERA_NAME = "image_0";
   //   String RIGHT_CAMERA_NAME = "image_1";
   //
   //   String DATASET_PATH = "/home/quantum/Workspace/Data/Datasets/sequences/00/";
   //
   //   String fileName = String.format("%1$6s", 0).replace(' ', '0') + ".png";
   //   String leftImageName = DATASET_PATH + LEFT_CAMERA_NAME + "/" + fileName;
   //   String rightImageName = DATASET_PATH + RIGHT_CAMERA_NAME + "/" + fileName;
   //
   //   ImageMat currentImageLeft = ImageTools.loadAsImageMat(leftImageName);
   //   ImageMat currentImageRight = ImageTools.loadAsImageMat(rightImageName);
   //
   //   vslam.update(currentImageLeft, currentImageRight);
   //}
}
