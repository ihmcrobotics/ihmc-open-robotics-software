package us.ihmc.perception;

import perception_msgs.msg.dds.IterativeClosestPointRequest;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.util.Map;
import java.util.Random;
import java.util.concurrent.ConcurrentHashMap;

public class IterativeClosestPointManager
{
   private static final double ICP_WORK_FREQUENCY = 20.0;
   private static final double EPSILON = 0.001;

   private final ROS2Helper ros2Helper;
   private final SceneGraph sceneGraph;

   private final OpenCLManager openCLManager = new OpenCLManager();
   private final OpenCLPointCloudExtractor pointCloudExtractor = new OpenCLPointCloudExtractor(openCLManager);

   private final Random random = new Random(System.nanoTime());
   private final ConcurrentHashMap<Long, IterativeClosestPointWorker> nodeIDToWorkerMap = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<IterativeClosestPointWorker, Integer> workerToIterationsMap = new ConcurrentHashMap<>();
   private final IHMCROS2Input<IterativeClosestPointRequest> requestMessageSubscription;
   private final RestartableThrottledThread workerThread;

   private RecyclingArrayList<Point3D32> environmentPointCloud;

   public IterativeClosestPointManager(ROS2Helper ros2Helper, SceneGraph sceneGraph)
   {
      this.ros2Helper = ros2Helper;
      this.sceneGraph = sceneGraph;

      requestMessageSubscription = ros2Helper.subscribe(PerceptionAPI.ICP_REQUEST);

      // Each time a request message is received, we update the corresponding worker accordingly
      // TODO: Find out whether messages can be skipped, resulting in updates being missed
      requestMessageSubscription.addCallback(requestMessage ->
      {
         long nodeID = requestMessage.getNodeId();

         if (!nodeIDToWorkerMap.containsKey(nodeID))     // ICP Worker requested (add new)
         {
            addWorker(requestMessage);
         }
         else if (!requestMessage.getRunIcp())           // ICP Worker no longer needed (remove)
         {
            workerToIterationsMap.remove(nodeIDToWorkerMap.get(nodeID));
            nodeIDToWorkerMap.remove(nodeID);
         }
         else                                            // ICP Worker exists (update it)
         {
            IterativeClosestPointWorker worker = nodeIDToWorkerMap.get(nodeID);

            // Check if size changed
            PrimitiveRigidBodyShape shape = PrimitiveRigidBodyShape.fromByte(requestMessage.getShape());
            Vector3D lengths = requestMessage.getLengths();
            Vector3D radii = requestMessage.getRadii();
            int numberOfShapeSamples = requestMessage.getNumberOfShapeSamples();

            boolean sizeChanged = false;
            sizeChanged |= !lengths.epsilonEquals(worker.getLengths(), EPSILON);
            sizeChanged |= !radii.epsilonEquals(worker.getRadii(), EPSILON);
            sizeChanged |= numberOfShapeSamples != worker.getNumberOfShapeSamples();

            // Update worker size if changed
            if (sizeChanged)
            {
               worker.changeSize(lengths, radii, numberOfShapeSamples);
            }

            // Update worker
            worker.setNumberOfCorrespondences(requestMessage.getNumberOfCorrespondences());
            worker.setTargetPoint(requestMessage.getProvidedPose().getPosition());
            worker.useProvidedTargetPoint(requestMessage.getUseProvidedPose());
            worker.setSegmentSphereRadius(requestMessage.getSegmentationRadius());
            workerToIterationsMap.replace(worker, requestMessage.getNumberOfIterations());
         }
      });

      workerThread = new RestartableThrottledThread("ICPWorkers", ICP_WORK_FREQUENCY, this::runWorkers);
   }

   /**
    * Sets the environment point cloud from a 16UC1 depth image,
    * then passes the new point cloud to all the workers
    * @param depthImage 16UC1 image from which the new point cloud is extracted
    */
   public void setEnvironmentPointCloud(RawImage depthImage)
   {
      depthImage.get();

      environmentPointCloud = pointCloudExtractor.extractPointCloud(depthImage);
      nodeIDToWorkerMap.forEachValue(1L, worker -> worker.setEnvironmentPointCloud(environmentPointCloud));

      depthImage.release();
   }

   public void startWorkers()
   {
      workerThread.start();
   }

   public void stopWorkers()
   {
      workerThread.stop();
   }

   public boolean isDemanded()
   {
      return !nodeIDToWorkerMap.isEmpty();
   }

   public void destroy()
   {
      workerThread.blockingStop();

      openCLManager.destroy();
   }

   /**
    * Runs all ICP workers present for two iterations,
    * updates the associated SceneNodes if ICP tracking is enabled
    */
   private void runWorkers()
   {
      for (Map.Entry<Long, IterativeClosestPointWorker> entry : nodeIDToWorkerMap.entrySet())
      {
         long nodeID = entry.getKey();

         if (!sceneGraph.getIDToNodeMap().containsKey(nodeID))
         {
            workerToIterationsMap.remove(nodeIDToWorkerMap.get(nodeID));
            nodeIDToWorkerMap.remove(nodeID);
            LogTools.info("Force removing ICP worker for node ID " + nodeID);
            continue;
         }

         if (entry.getValue().runICP(workerToIterationsMap.get(entry.getValue())))
            ros2Helper.publish(PerceptionAPI.ICP_RESULT, entry.getValue().getResult());

         if (!entry.getValue().isUsingTargetPoint())
         {
            RigidBodyTransform centroidToWorldTransform = new RigidBodyTransform(entry.getValue().getResultPose());
            SceneNode node = sceneGraph.getIDToNodeMap().get(entry.getKey());
            if (node != null) // FIXME: race condition occurs when this is running & node is removed from scene graph through the scene graph UI
            {
               node.getNodeToParentFrameTransform().set(centroidToWorldTransform);
               node.getNodeFrame().update();
            }
         }
      }
   }

   private void addWorker(IterativeClosestPointRequest requestMessage)
   {
      PrimitiveRigidBodyShape shape = PrimitiveRigidBodyShape.fromByte(requestMessage.getShape());

      Vector3D lengths = requestMessage.getLengths();
      Vector3D radii = requestMessage.getRadii();

      IterativeClosestPointWorker worker = new IterativeClosestPointWorker(shape,
                                                                           lengths,
                                                                           radii,
                                                                           requestMessage.getNumberOfShapeSamples(),
                                                                           requestMessage.getNumberOfCorrespondences(),
                                                                           new FramePose3D(requestMessage.getProvidedPose()),
                                                                           random);
      worker.setSceneNodeID(requestMessage.getNodeId());
      nodeIDToWorkerMap.putIfAbsent(requestMessage.getNodeId(), worker);
      workerToIterationsMap.putIfAbsent(worker, requestMessage.getNumberOfIterations());
   }
}
