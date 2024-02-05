package us.ihmc.perception.semantic;

import org.apache.commons.lang.ArrayUtils;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.YOLOv8.YOLOv8Detection;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionResults;
import us.ihmc.perception.YOLOv8.YOLOv8ObjectDetector;
import us.ihmc.perception.slamWrapper.SlamWrapperNativeLibrary;
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
import us.ihmc.perception.visualOdometry.VisualOdometryNativeLibrary;
import us.ihmc.tools.IHMCCommonPaths;

import java.util.*;

public class SemanticSLAMModule
{
   private static String WEIGHTS_FILE_PATH = IHMCCommonPaths.WEIGHTS_DIRECTORY.resolve("yolov8n-seg_736x1280.onnx").toString();
   private static final float CONFIDENCE_THRESHOLD = 0.5f;
   private static final float NMS_THRESHOLD = 0.1f;
   private static final float MASK_THRESHOLD = 0.0f;

   public static final int MAX_LANDMARKS = 400;
   public static final int NUMBER_OF_FEATURES = 400;
   public static final int MIN_NUM_FEATURES = 350;

   private final double[] latestSensorPose = new double[16];
   private double[] landmarkPoints = new double[0];
   private int[] idInts = new int[0];

   private boolean initialized = false;
   private int frameIndex = 0;

   private final RigidBodyTransform worldToSensorTransform = new RigidBodyTransform();
   private final RigidBodyTransform optimizedSensorToWorldTransform = new RigidBodyTransform();
   private final YOLOv8ObjectDetector yoloObjectDetector = new YOLOv8ObjectDetector(WEIGHTS_FILE_PATH);
   private final HashMap<Integer, Keyframe> keyframes = new HashMap<>();
   private final HashMap<Integer, Landmark> landmarks = new HashMap<>();
   private final SlamWrapper.FactorGraphExternal factorGraphExternal;


   public SemanticSLAMModule()
   {
      VisualOdometryNativeLibrary.load();
      SlamWrapperNativeLibrary.load();

      factorGraphExternal = new SlamWrapper.FactorGraphExternal();

      double[] poseInitial = new double[] {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

      factorGraphExternal.addPriorPoseFactor(0, poseInitial);
      factorGraphExternal.setPoseInitialValue(0, poseInitial);

      keyframes.put(0, new Keyframe(0));
   }

   public boolean update(Mat leftImage, Mat rightImage)
   {

      long acquisitionTime = System.currentTimeMillis();

      List<YOLOv8Detection> results = yoloObjectDetector.runForDetectionsOnMat(leftImage, CONFIDENCE_THRESHOLD, NMS_THRESHOLD);


      return initialized;
   }



   public void clearISAM2()
   {
      factorGraphExternal.clearISAM2();
   }

   public FramePose3D getSensorPose(int index)
   {
      double[] poses = new double[16];
      int[] indices = new int[] {index};
      factorGraphExternal.getResultPoses(poses, indices, 1);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.set(poses);

      FramePose3D pose = new FramePose3D();
      pose.set(transform);
      return pose;
   }

   public ArrayList<Point3D> getLandmarkPoints(Set<Integer> ids)
   {
      Integer[] idIntObjects = Arrays.stream(ids.toArray()).parallel().map(o -> (Integer) o).toArray(Integer[]::new);
      idInts = ArrayUtils.toPrimitive(idIntObjects);
      landmarkPoints = new double[idInts.length * 3];

      // Extract optimized landmark points from factor graph.
      factorGraphExternal.getResultLandmarks(landmarkPoints, idInts, idInts.length);

      ArrayList<Point3D> points = new ArrayList<>();
      for (int i = 0; i < idInts.length; i++)
      {
         points.add(new Point3D(landmarkPoints[i * 3], landmarkPoints[i * 3 + 1], landmarkPoints[i * 3 + 2]));
      }
      return points;
   }

   public void render()
   {

   }

   public Set<Integer> getLandmarkKeys()
   {
      return landmarks.keySet();
   }
}
