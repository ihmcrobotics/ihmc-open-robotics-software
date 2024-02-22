package us.ihmc.perception.YOLOv8;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.OpenCLDepthImageSegmenter;
import us.ihmc.perception.OpenCLPointCloudExtractor;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.filters.DetectionFilter;
import us.ihmc.perception.iterativeClosestPoint.IterativeClosestPointWorker;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.sceneGraph.YOLOv8IterativeClosestPointNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphClearSubtree;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeRemoval;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.tools.time.FrequencyCalculator;

import java.util.List;
import java.util.Random;

public class YOLOv8IterativeClosestPointNodeCombo
{
   private static final int OUTLIER_REJECTION_SAMPLES = 256;

   private final ROS2Helper ros2Helper;

   private final OpenCLPointCloudExtractor extractor;
   private final OpenCLDepthImageSegmenter segmenter;

   private final long nodeID;
   private YOLOv8IterativeClosestPointNode node;
   private final IterativeClosestPointWorker icpWorker;
   private YOLOv8Detection lastDetection;

   private final RigidBodyTransform lastCentroidToWorldTransform = new RigidBodyTransform();

   private boolean selfDestruct = false;
   private double distanceThreshold;
   private final DetectionFilter detectionFilter;
   private boolean ranICP = false;
   private final FrequencyCalculator detectionFrequencyCalculator = new FrequencyCalculator();

   public YOLOv8IterativeClosestPointNodeCombo(YOLOv8Detection initialDetection,
                                               DetectionFilter detectionFilter,
                                               List<Point3D32> objectPointCloud,
                                               ROS2SceneGraph sceneGraph,
                                               SceneGraphModificationQueue modificationQueue,
                                               ROS2Helper ros2Helper,
                                               OpenCLManager openCLManager)
   {
      this.ros2Helper = ros2Helper;
      this.detectionFilter = detectionFilter;
      this.detectionFilter.setAcceptanceThreshold(0.2f); // TODO: Make this a variable in the node
      lastDetection = initialDetection;
      extractor = new OpenCLPointCloudExtractor(openCLManager);
      segmenter = new OpenCLDepthImageSegmenter(openCLManager);

      icpWorker = new IterativeClosestPointWorker(objectPointCloud,
                                                  1000,
                                                  new Random(System.nanoTime()));

      icpWorker.useProvidedTargetPoint(false);
      icpWorker.setSegmentSphereRadius(Double.POSITIVE_INFINITY);

      nodeID = sceneGraph.getNextID().getAndIncrement();
      icpWorker.setSceneNodeID(nodeID);

      String nodeName = lastDetection.objectClass().toString().substring(0, 1).toUpperCase() + lastDetection.objectClass().toString().substring(1) + " " + nodeID;
      node = new YOLOv8IterativeClosestPointNode(nodeID, nodeName, 1, 2.0, 1, 3000.0, true, distanceThreshold, 0.0);
      modificationQueue.accept(new SceneGraphNodeAddition(node, sceneGraph.getRootNode()));
      sceneGraph.getIDToNodeMap().put(nodeID, node);
      distanceThreshold = node.getBaseDistanceThreshold();
   }

   /**
    * Updates the scene node associated with the detection.
    * Creates a new node & ICP worker if they have not been created yet.
    * Should be called in the scene graph update loop.
    *
    * @param sceneGraph The scene graph
    * @param modificationQueue Modification queue of the scene graph
    * @return true if node was updated or created normally, false if the node was removed from the scene graph.
    */
   public boolean updateSceneGraph(ROS2SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue)
   {
      if (sceneGraph.getIDToNodeMap().containsKey(nodeID))
         node = (YOLOv8IterativeClosestPointNode) sceneGraph.getIDToNodeMap().get(nodeID);
      else
      {
         destroy();
         return false;
      }

      if (selfDestruct)
      {
         node.setCurrentlyDetected(false);
         modificationQueue.accept(new SceneGraphClearSubtree(node));
         modificationQueue.accept(new SceneGraphNodeRemoval(node, sceneGraph));
         return false;
      }

      synchronized (lastCentroidToWorldTransform)
      {
         node.getNodeToParentFrameTransform().set(lastCentroidToWorldTransform);
      }
      node.getNodeFrame().update();

      if (ranICP)
         detectionFilter.registerDetection();
      detectionFilter.update();
      node.setCurrentlyDetected(detectionFilter.isStableDetectionResult() && ranICP);
      node.setMovementDistanceThreshold(distanceThreshold);
      node.setDetectionFrequency(detectionFrequencyCalculator.anyPingsYet() ? detectionFrequencyCalculator.getFrequency() : 0.0);

      return true;
   }

   /**
    * Destroys itself if the detection has expired.
    * Each time this method is called, the distance threshold increases.
    *
    * @return true if detection has expired and destruction is initiated, false if detection has not expired.
    */
   public boolean destroyIfExpired()
   {
      distanceThreshold += node.getBaseDistanceThreshold();
      ranICP = false;

      if (detectionFilter.hasEnoughSamples() && !detectionFilter.isStableDetectionResult())
      {
         destroy();
         return true;
      }

      return false;
   }

   private void destroy()
   {
      selfDestruct = true;

      // ensure extractor and segmenter don't get destroyed while in use
      synchronized (extractor)
      {
         extractor.destroy();
         segmenter.destroy();
      }
   }

   public void runICP(RawImage depthImage, RawImage mask)
   {
      synchronized (extractor)
      {
         if (!selfDestruct)
         {
            depthImage.get();
            mask.get();

            if (node.isRunningICP())
            {
               // decrease distance threshold if it has been increased beyond default threshold
               if (distanceThreshold > node.getBaseDistanceThreshold())
                  distanceThreshold = Math.max(distanceThreshold - node.getBaseDistanceThreshold(), node.getBaseDistanceThreshold());

               // Process images & run ICP
               RawImage segmentedDepth = segmenter.removeBackground(depthImage, mask, node.getMaskErosionKernelRadius()); // segment depth image using the mask
               List<Point3DReadOnly> segmentedPointCloud = extractor.extractPointCloud(segmentedDepth); // extract the point cloud from the depth image
               segmentedPointCloud = YOLOv8Tools.filterOutliers(segmentedPointCloud, node.getOutlierFilterThreshold(), OUTLIER_REJECTION_SAMPLES); // remove outliers
               icpWorker.setEnvironmentPointCloud(segmentedPointCloud); // provide segmented & filtered point cloud to the ICP worker
               if (icpWorker.runICP(node.getICPIterations())) // run ICP & publish results if successful
                  ros2Helper.publish(PerceptionAPI.ICP_RESULT, icpWorker.getResult());

               // Update transform which will be provided to the node
               synchronized (lastCentroidToWorldTransform)
               {
                  lastCentroidToWorldTransform.set(icpWorker.getResultPose());
               }
               segmentedDepth.release();
            }

            ranICP = true;
            detectionFrequencyCalculator.ping();

            depthImage.release();
            mask.release();
         }
      }
   }

   public void setDetection(YOLOv8Detection detection)
   {
      lastDetection = detection;
   }

   public YOLOv8Detection getLastDetection()
   {
      return lastDetection;
   }

   public double getDistanceThreshold()
   {
      return distanceThreshold;
   }
}
