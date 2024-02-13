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
import us.ihmc.perception.filters.DetectionFilter;
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
   private static final int OUTLIER_REJECTION_SAMPLES = 256;

   private final ROS2Helper ros2Helper;
   private final WorkspaceResourceDirectory pointCloudDirectory;

   private final OpenCLPointCloudExtractor extractor;
   private final OpenCLDepthImageSegmenter segmenter;

   private long nodeID = -1L;
   private YOLOv8IterativeClosestPointNode node;
   private IterativeClosestPointWorker icpWorker;
   private YOLOv8Detection lastDetection;

   private RigidBodyTransform lastCentroidToWorldTransform = new RigidBodyTransform();
   private final Object transformSynchronizer = new Object();

   private boolean isCreated = false;
   private boolean selfDestruct = false;
   private boolean detectionIsRunning = true;
   private double distanceThreshold;
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
    * @param sceneGraph The scene graph
    * @param modificationQueue Modification queue of the scene graph
    * @return true if node was updated or created normally, false if the node was removed from the scene graph.
    */
   public boolean updateSceneGraph(ROS2SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue)
   {
      if (sceneGraph.getIDToNodeMap().get(nodeID) != null)
         node = (YOLOv8IterativeClosestPointNode) sceneGraph.getIDToNodeMap().get(nodeID);

      if (selfDestruct && node != null)
      {
         System.out.println("Destroying in updateSceneGraph()");
         System.out.flush();
         node.setCurrentlyDetected(false);
         modificationQueue.accept(new SceneGraphClearSubtree(node));
         modificationQueue.accept(new SceneGraphNodeRemoval(node, sceneGraph));
         return false;
      }
      else if (!isCreated)
      {
         if (create(sceneGraph, modificationQueue))
            isCreated = true;
      }
      else
      {
         synchronized (transformSynchronizer)
         {
            node.getNodeToParentFrameTransform().set(lastCentroidToWorldTransform);
         }
         node.setCurrentlyDetected(detectionFilter.isDetected() && detectionIsRunning);
         node.setMovementDistanceThreshold(distanceThreshold); // FIXME: this doesn't get set properly
      }

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
      distanceThreshold += 400.0;
      detectionIsRunning = false;

      if (!detectionFilter.isDetected())
      {
         System.out.println("Destroying in destroy()");
         System.out.flush();
         selfDestruct = true;
         extractor.destroy();
         segmenter.destroy();
         return true;
      }

      return false;
   }

   public void runICP(RawImage depthImage, RawImage mask)
   {
      depthImage.get();
      mask.get();

      detectionIsRunning = true;

      if (isCreated && !selfDestruct)
      {
         if (node.isRunningICP())
         {
            // Node has received a new detection.
            detectionFilter.registerDetection();

            // decrease distance threshold if it has been increased beyond default threshold
            if (distanceThreshold > node.getBaseDistanceThreshold())
               distanceThreshold -= 100.0;

            // Process images & run ICP
            RawImage segmentedDepth = segmenter.removeBackground(depthImage, mask, node.getMaskErosionKernelRadius()); // segment depth image using the mask
            List<Point3DReadOnly> segmentedPointCloud = extractor.extractPointCloud(segmentedDepth); // extract the point cloud from the depth image
            segmentedPointCloud = YOLOv8Tools.filterOutliers(segmentedPointCloud, node.getOutlierFilterThreshold(), OUTLIER_REJECTION_SAMPLES); // remove outliers
            icpWorker.setEnvironmentPointCloud(segmentedPointCloud); // provide segmented & filtered point cloud to the ICP worker
            if (icpWorker.runICP(node.getICPIterations())) // run ICP & publish results if successful
               ros2Helper.publish(PerceptionAPI.ICP_RESULT, icpWorker.getResult());

            // Update transform which will be provided to the node
            synchronized (transformSynchronizer)
            {
               lastCentroidToWorldTransform = new RigidBodyTransform(icpWorker.getResultPose());
            }
            segmentedDepth.release();
         }
      }

      depthImage.release();
      mask.release();
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

   private boolean create(ROS2SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue)
   {
      // ensure a detection has been provided before creating
      if (lastDetection != null)
      {
         String pointCloudFileName = lastDetection.objectClass().getPointCloudFileName();
         if (pointCloudFileName == null)
            throw new NullPointerException("We can't run ICP on this object yet because we don't have a model point cloud file.");

         WorkspaceResourceFile pointCloudFile = new WorkspaceResourceFile(pointCloudDirectory, pointCloudFileName);
         icpWorker = new IterativeClosestPointWorker(lastDetection.objectClass().getPrimitiveApproximation(),
                                                     new Vector3D(1.0, 1.0, 1.0),
                                                     new Vector3D(1.0, 1.0, 1.0),
                                                     1000,
                                                     1000,
                                                     new Pose3D(),
                                                     new Random(System.nanoTime()));

         icpWorker.useProvidedTargetPoint(false);
         icpWorker.setSegmentSphereRadius(Double.MAX_VALUE);
         icpWorker.setDetectionShape(PrimitiveRigidBodyShape.CUSTOM, pointCloudFile.getFilesystemFile().toFile());

         nodeID = sceneGraph.getNextID().getAndIncrement();
         icpWorker.setSceneNodeID(nodeID);

         String nodeName = lastDetection.objectClass().toString().substring(0, 1).toUpperCase() + lastDetection.objectClass().toString().substring(1) + " " + nodeID;
         node = new YOLOv8IterativeClosestPointNode(nodeID, nodeName, 1, 2.0, 1, 3000.0, true, distanceThreshold);
         modificationQueue.accept(new SceneGraphNodeAddition(node, sceneGraph.getRootNode()));
         sceneGraph.getIDToNodeMap().put(nodeID, node);
         distanceThreshold = node.getBaseDistanceThreshold();

         detectionFilter.registerDetection();

         return true;
      }

      return false;
   }
}
