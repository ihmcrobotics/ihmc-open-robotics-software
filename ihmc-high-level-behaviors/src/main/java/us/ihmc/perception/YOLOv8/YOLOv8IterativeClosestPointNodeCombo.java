package us.ihmc.perception.YOLOv8;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.OpenCLDepthImageSegmenter;
import us.ihmc.perception.OpenCLPointCloudExtractor;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.filters.TimeBasedDetectionFilter;
import us.ihmc.perception.iterativeClosestPoint.IterativeClosestPointWorker;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.sceneGraph.YOLOv8IterativeClosestPointNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphClearSubtree;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeRemoval;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.List;
import java.util.Random;

public class YOLOv8IterativeClosestPointNodeCombo
{
   private static final int EROSION_KERNEL_RADIUS = 1;
   private static final int ICP_ITERATIONS = 2;
   private static final double Z_SCORE_THRESHOLD = 2.0;
   private static final int OUTLIER_REJECTION_SAMPLES = 256;
   private static final double BASE_DISTANCE_THRESHOLD = 2000.0;

   private final ROS2Helper ros2Helper;
   private final WorkspaceResourceDirectory pointCloudDirectory;

   private final OpenCLPointCloudExtractor extractor;
   private final OpenCLDepthImageSegmenter segmenter;

   private YOLOv8IterativeClosestPointNode node;
   private IterativeClosestPointWorker icpWorker;
   private YOLOv8Detection lastDetection;

   private RigidBodyTransform lastCentroidToWorldTransform = new RigidBodyTransform();
   private final Object transformSynchronizer = new Object();

   private boolean isCreated = false;
   private boolean selfDestruct = false;
   private double distanceThreshold = BASE_DISTANCE_THRESHOLD;
   private final TimeBasedDetectionFilter detectionFilter = new TimeBasedDetectionFilter(2.0, 2);

   public YOLOv8IterativeClosestPointNodeCombo(ROS2Helper ros2Helper, WorkspaceResourceDirectory pointCloudDirectory, OpenCLManager openCLManager)
   {
      this.ros2Helper = ros2Helper;
      this.pointCloudDirectory = pointCloudDirectory;
      extractor = new OpenCLPointCloudExtractor(openCLManager);
      segmenter = new OpenCLDepthImageSegmenter(openCLManager);
   }

   /**
    * Updates the scene node associated with the detection.
    * Creates a new node & ICP worker if they have not been created yet.
    * Should be called in the scene graph update loop.
    *
    * @param sceneGraph the scene graph
    * @param detection detection associated with the node/ICP worker (only used when creating)
    * @return true if the node was normally updated, false if the node was removed from the scene graph.
    */
   public boolean updateSceneGraph(ROS2SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue, YOLOv8Detection detection)
   {
      if (selfDestruct)
      {
         node.setCurrentlyDetected(false);
         modificationQueue.accept(new SceneGraphClearSubtree(node));
         modificationQueue.accept(new SceneGraphNodeRemoval(node, sceneGraph));
         return false;
      }
      else if (!isCreated)
      {
         create(detection, sceneGraph);
         isCreated = true;
      }
      else
      {
         synchronized (transformSynchronizer)
         {
            node.getNodeToParentFrameTransform().set(lastCentroidToWorldTransform);
            node.setCurrentlyDetected(detectionFilter.isDetected());
         }
      }

      return true;
   }

   public boolean destroy()
   {
      distanceThreshold += 400.0;
      if (!detectionFilter.isDetected())
      {
         selfDestruct = true;
         extractor.destroy();
         segmenter.destroy();
         return true;
      }

      return false;
   }

   public void runICP(YOLOv8Detection newDetection, RawImage depthImage, RawImage mask)
   {
      depthImage.get();
      mask.get();

      if (!selfDestruct)
      {
         if (distanceThreshold > BASE_DISTANCE_THRESHOLD)
            distanceThreshold -= 100.0;
         lastDetection = newDetection;
         RawImage segmentedDepth = segmenter.removeBackground(depthImage, mask, EROSION_KERNEL_RADIUS);
         List<Point3DReadOnly> segmentedPointCloud = extractor.extractPointCloud(segmentedDepth);
         icpWorker.setEnvironmentPointCloud(YOLOv8Tools.filterOutliers(segmentedPointCloud, Z_SCORE_THRESHOLD, OUTLIER_REJECTION_SAMPLES));
         if (icpWorker.runICP(ICP_ITERATIONS))
            ros2Helper.publish(PerceptionAPI.ICP_RESULT, icpWorker.getResult());

         synchronized (transformSynchronizer)
         {
            lastCentroidToWorldTransform = new RigidBodyTransform(icpWorker.getResultPose());
         }

         detectionFilter.registerDetection();
         segmentedDepth.release();
      }

      depthImage.release();
      mask.release();
   }

   public YOLOv8Detection getLastDetection()
   {
      return lastDetection;
   }

   public double getDistanceThreshold()
   {
      return distanceThreshold;
   }

   private void create(YOLOv8Detection detection, ROS2SceneGraph sceneGraph)
   {
      lastDetection = detection;

      String pointCloudFileName = detection.objectClass().getPointCloudFileName();
      if (pointCloudFileName == null)
         throw new NullPointerException("We can't run ICP on this object yet because we don't have a model point cloud file.");

      WorkspaceResourceFile pointCloudFile = new WorkspaceResourceFile(pointCloudDirectory, pointCloudFileName);
      icpWorker = new IterativeClosestPointWorker(detection.objectClass().getPrimitiveApproximation(),
                                                  new Vector3D(1.0, 1.0, 1.0),
                                                  new Vector3D(1.0, 1.0, 1.0),
                                                  1000,
                                                  1000,
                                                  new Pose3D(),
                                                  new Random(System.nanoTime()));

      icpWorker.useProvidedTargetPoint(false);
      icpWorker.setSegmentSphereRadius(Double.MAX_VALUE);
      icpWorker.setDetectionShape(PrimitiveRigidBodyShape.CUSTOM, pointCloudFile.getFilesystemFile().toFile());

      long id = sceneGraph.getNextID().getAndIncrement();
      icpWorker.setSceneNodeID(id);
      node = new YOLOv8IterativeClosestPointNode(id, detection.objectClass().toString() + id);
      sceneGraph.modifyTree(modificationQueue -> modificationQueue.accept(new SceneGraphNodeAddition(node, sceneGraph.getRootNode())));

      detectionFilter.registerDetection();
   }
}
