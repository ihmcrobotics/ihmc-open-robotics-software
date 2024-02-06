package us.ihmc.perception.YOLOv8;

import org.ddogleg.struct.Tuple3;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.OpenCLDepthImageSegmenter;
import us.ihmc.perception.OpenCLPointCloudExtractor;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.iterativeClosestPoint.IterativeClosestPointWorker;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.YOLOv8IterativeClosestPointNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;
import us.ihmc.tools.thread.RestartableThread;

import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class YOLOv8IterativeClosestPointManager
{
   private static final float YOLO_CONFIDENCE_THRESHOLD = 0.3f;
   private static final float YOLO_NMS_THRESHOLD = 0.1f;
   private static final float YOLO_SEGMENTATION_THRESHOLD = 0.0f;
   private static final int EROSION_KERNEL_RADIUS = 2;
   private static final double Z_SCORE_THRESHOLD = 1.0;
   private static final int OUTLIER_REJECTION_SAMPLES = 300;
   private static final double DISTANCE_THRESHOLD = 10.0;
   private static final double ICP_WORK_FREQUENCY = 20.0;
   private static final int ICP_ITERATIONS = 2;
   private static final Random RANDOM = new Random(System.nanoTime());

   private final OpenCLManager openCLManager = new OpenCLManager();
   private final OpenCLPointCloudExtractor extractor = new OpenCLPointCloudExtractor(openCLManager);
   private final OpenCLDepthImageSegmenter segmenter = new OpenCLDepthImageSegmenter(openCLManager);

   private final WorkspaceResourceDirectory pointCloudDirectory = new WorkspaceResourceDirectory(YOLOv8DetectableObject.class, "/yoloICPPointClouds/");

   private final SceneGraph sceneGraph;

   private final YOLOv8ObjectDetector yoloDetector = new YOLOv8ObjectDetector();
   private RawImage yoloDetectionImage;
   private RawImage icpEnvironmentDepthImage;
   private long lastYoloImageSequenceNumber = -1;
   private long lastICPImageSequenceNumber = -1;
   private Set<YOLOv8Detection> oldDetections = new HashSet<>();
   private final RestartableThread yoloICPThread;
   private final Lock imageUpdateLock = new ReentrantLock();
   private final Condition newImagesAvailable = imageUpdateLock.newCondition();

   private final Map<YOLOv8Detection, IterativeClosestPointWorker> detectionToWorkerMap = new HashMap<>();
   private final Map<IterativeClosestPointWorker, YOLOv8IterativeClosestPointNode> workerToNodeMap = new HashMap<>();

   public YOLOv8IterativeClosestPointManager(SceneGraph sceneGraph)
   {
      this.sceneGraph = sceneGraph;
      yoloICPThread = new RestartableThread("YOLODetector", this::runYOLODetection);
   }

   public void setDetectionImages(RawImage yoloColorImage, RawImage icpDepthImage)
   {
      yoloColorImage.get();
      icpDepthImage.get();
      imageUpdateLock.lock();
      try
      {
         if (yoloDetectionImage != null)
            yoloDetectionImage.release();
         if (icpEnvironmentDepthImage != null)
            icpEnvironmentDepthImage.release();

         yoloDetectionImage = yoloColorImage;
         icpEnvironmentDepthImage = icpDepthImage;
         newImagesAvailable.signal();
      }
      finally
      {
         imageUpdateLock.unlock();
      }

      yoloColorImage.release();
      icpDepthImage.release();
   }

   private void runYOLODetection()
   {

      imageUpdateLock.lock();
      try
      {
         while ((yoloDetectionImage == null || yoloDetectionImage.isEmpty() || yoloDetectionImage.getSequenceNumber() == lastYoloImageSequenceNumber)
                && (icpEnvironmentDepthImage == null || icpEnvironmentDepthImage.isEmpty()
                    || icpEnvironmentDepthImage.getSequenceNumber() == lastICPImageSequenceNumber))
         {
            newImagesAvailable.await();
         }

         runYOLOAndICP();

         lastYoloImageSequenceNumber = yoloDetectionImage.getSequenceNumber();

      }
      catch (InterruptedException interruptedException)
      {
         interruptedException.printStackTrace();
      }
      finally
      {
         imageUpdateLock.unlock();
      }
   }

   private void runYOLOAndICP()
   {
      YOLOv8DetectionResults yoloResults = yoloDetector.runOnImage(yoloDetectionImage, YOLO_CONFIDENCE_THRESHOLD, YOLO_NMS_THRESHOLD);
      Map<YOLOv8Detection, RawImage> objectMasks = yoloResults.getSegmentationImages(YOLO_SEGMENTATION_THRESHOLD);

      Set<YOLOv8Detection> newDetections = yoloResults.getDetections();
      Map<YOLOv8Detection, YOLOv8Detection> oldNewMatchingDetections = organizeDetections(oldDetections, newDetections);

      for (Map.Entry<YOLOv8Detection, YOLOv8Detection> oldNewPair : oldNewMatchingDetections.entrySet())
      {
         YOLOv8Detection oldDetection = oldNewPair.getKey();
         YOLOv8Detection newDetection = oldNewPair.getValue();

         IterativeClosestPointWorker icpWorker = detectionToWorkerMap.get(oldDetection);
         if (icpWorker == null)
            throw new NullPointerException("Can't find the ICP worker you're looking for.");

         RawImage segmentedDepth = segmenter.removeBackground(icpEnvironmentDepthImage, objectMasks.get(newDetection), EROSION_KERNEL_RADIUS);
         List<Point3DReadOnly> segmentedPointCloud = extractor.extractPointCloud(segmentedDepth);
         icpWorker.setEnvironmentPointCloud(segmentedPointCloud);
         icpWorker.runICP(ICP_ITERATIONS);

         // replace old key with new one
         detectionToWorkerMap.put(newDetection, detectionToWorkerMap.remove(oldDetection));

         segmentedDepth.release();
      }

      for (YOLOv8Detection newDetection : newDetections)
      {
         IterativeClosestPointWorker icpWorker = createICPWorker(newDetection);
         detectionToWorkerMap.put(newDetection, icpWorker);

         RawImage segmentedDepth = segmenter.removeBackground(icpEnvironmentDepthImage, objectMasks.get(newDetection), EROSION_KERNEL_RADIUS);
         List<Point3DReadOnly> segmentedPointCloud = extractor.extractPointCloud(segmentedDepth);
         icpWorker.setEnvironmentPointCloud(segmentedPointCloud);
         icpWorker.runICP(ICP_ITERATIONS);
         // TODO: Worker -> scene node map, add node in scene graph update if a worker not associated with scene node
      }

      for (YOLOv8Detection oldDetection : oldDetections)
      {
         // Remove the old detection and associated stuff TODO: update the scene graph of this removal
         workerToNodeMap.remove(detectionToWorkerMap.remove(oldDetection));
         oldDetections.remove(oldDetection);
      }

      for (RawImage mask : objectMasks.values())
         mask.release();
      yoloResults.destroy();
   }

   /**
    *
    * @param oldDetections Previous YOLOv8 detections. MODIFIED: old detections which were matched to new ones will be removed from the set.
    * @param newDetections New YOLOv8 detections. MODIFIED: new detections which were matched to new ones will be removed from the set.
    * @return Map of closest old and new detections
    */
   private Map<YOLOv8Detection, YOLOv8Detection> organizeDetections(Set<YOLOv8Detection> oldDetections, Set<YOLOv8Detection> newDetections)
   {
      // Priority queue of all possible detection pairs ordered by distance between them
      PriorityQueue<Tuple3<YOLOv8Detection, YOLOv8Detection, Double>> matchingDetections = new PriorityQueue<>(Comparator.comparingDouble(Tuple3::getD2));

      // Add all detections which possibly match to the queue
      for (YOLOv8Detection oldDetection : oldDetections)
      {
         for (YOLOv8Detection newDetection : newDetections)
         {
            double distanceSquared = MathTools.square(oldDetection.x() - newDetection.x())
                                     + MathTools.square(oldDetection.y() - newDetection.y());

            if (oldDetection.objectClass() == newDetection.objectClass() && distanceSquared < DISTANCE_THRESHOLD)
               matchingDetections.add(new Tuple3<>(oldDetection, newDetection, distanceSquared));
         }
      }

      Map<YOLOv8Detection, YOLOv8Detection> oldNewMatchingDetectionMap = new HashMap<>();
      while (!oldDetections.isEmpty() && !newDetections.isEmpty())
      {
         Tuple3<YOLOv8Detection, YOLOv8Detection, Double> bestMatchDetections = matchingDetections.poll();
         if (bestMatchDetections == null)
            break;

         YOLOv8Detection oldMatchDetection = bestMatchDetections.getD0();
         YOLOv8Detection newMatchDetection = bestMatchDetections.getD1();
         if (oldDetections.contains(oldMatchDetection) && newDetections.contains(newMatchDetection))
         {
            oldDetections.remove(oldMatchDetection);
            newDetections.remove(newMatchDetection);
            oldNewMatchingDetectionMap.put(oldMatchDetection, newMatchDetection);
         }
      }

      return oldNewMatchingDetectionMap;
   }

   private IterativeClosestPointWorker createICPWorker (YOLOv8Detection targetObject)
   {
      String pointCloudFileName = targetObject.objectClass().getPointCloudFileName();
      if (pointCloudFileName == null)
         return null;

      WorkspaceResourceFile pointCloudFile = new WorkspaceResourceFile(pointCloudDirectory, pointCloudFileName);
      IterativeClosestPointWorker worker = new IterativeClosestPointWorker(pointCloudFile.getFilesystemFile().toFile(), 1000, new Pose3D(), RANDOM);
      worker.useProvidedTargetPoint(false);
      worker.setSegmentSphereRadius(Double.MAX_VALUE);
      return worker;
   }
}
