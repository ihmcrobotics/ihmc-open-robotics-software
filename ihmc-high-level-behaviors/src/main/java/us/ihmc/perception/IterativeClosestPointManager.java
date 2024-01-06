package us.ihmc.perception;

import perception_msgs.msg.dds.IterativeClosestPointRequest;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.util.HashMap;
import java.util.Random;

public class IterativeClosestPointManager
{
   private static final double ICP_WORK_FREQUENCY = 20.0;
   private static final int NUMBER_OF_CORRESPONDENCES = 1000; // TODO extract to request message
   private static final double EPSILON = 0.001;

   private final ROS2Helper ros2Helper;
   private final SceneGraph sceneGraph;

   private final OpenCLManager openCLManager = new OpenCLManager();
   private final OpenCLPointCloudExtractor pointCloudExtractor = new OpenCLPointCloudExtractor(openCLManager);

   private final Random random = new Random(System.nanoTime());
   private final HashMap<Long, IterativeClosestPointWorker> nodeIDToWorkerMap = new HashMap<>();
   private final IHMCROS2Input<IterativeClosestPointRequest> requestMessageSubscription;
   private final RestartableThrottledThread workerThread;

   private RecyclingArrayList<Point3D32> environmentPointCloud;

   public IterativeClosestPointManager(ROS2Helper ros2Helper, SceneGraph sceneGraph)
   {
      this.ros2Helper = ros2Helper;
      this.sceneGraph = sceneGraph;

      requestMessageSubscription = ros2Helper.subscribe(PerceptionAPI.ICP_REQUEST);

      // Each time a request message is received, we update the corresponding worker accordingly
      // TODO: Find out whether messages can be skipped, resulting in updates being misseed
      requestMessageSubscription.addCallback(requestMessage ->
      {
         long nodeID = requestMessage.getNodeId();

         if (!nodeIDToWorkerMap.containsKey(nodeID))     // ICP Worker requested (add new)
         {
            addWorker(requestMessage);
         }
         else if (!requestMessage.getRunIcp())           // ICP Worker no longer needed (remove)
         {
            nodeIDToWorkerMap.remove(nodeID);
         }
         else                                            // ICP Worker exists (update it)
         {
            IterativeClosestPointWorker worker = nodeIDToWorkerMap.get(nodeID);

            // Check if size changed
            PrimitiveRigidBodyShape shape = PrimitiveRigidBodyShape.fromByte(requestMessage.getShape());
            float requestXLength = requestMessage.getXLength();
            float requestYLength = requestMessage.getYLength();
            float requestZLength = requestMessage.getZLength();
            float requestXRadius = requestMessage.getXRadius();
            float requestYRadius = requestMessage.getYRadius();
            float requestZRadius = requestMessage.getZRadius();

            boolean sizeChanged = false;
            sizeChanged |= !MathTools.epsilonEquals(worker.getXLength(), requestXLength, EPSILON);
            sizeChanged |= !MathTools.epsilonEquals(worker.getYLength(), requestYLength, EPSILON);
            sizeChanged |= !MathTools.epsilonEquals(worker.getZLength(), requestZLength, EPSILON);
            sizeChanged |= !MathTools.epsilonEquals(worker.getXRadius(), requestXRadius, EPSILON);
            sizeChanged |= !MathTools.epsilonEquals(worker.getYRadius(), requestYRadius, EPSILON);
            sizeChanged |= !MathTools.epsilonEquals(worker.getZRadius(), requestZRadius, EPSILON);

            // Update worker size if changed
            if (sizeChanged)
            {
               int newNumberOfPoints = approximateNumberOfPoints(shape,
                                                                 requestXLength,
                                                                 requestYLength,
                                                                 requestZLength,
                                                                 requestXRadius,
                                                                 requestYRadius,
                                                                 requestZRadius);
               worker.changeSize(requestXLength, requestYLength, requestZLength, requestXRadius, requestYRadius, requestZRadius, newNumberOfPoints);
            }

            // Update worker
            worker.setNumberOfCorrespondences(NUMBER_OF_CORRESPONDENCES);             // TODO extract to request message
            worker.setTargetPoint(requestMessage.getProvidedPose().getPosition());
            worker.useProvidedTargetPoint(requestMessage.getUseProvidedPose());
            worker.setSegmentSphereRadius(requestMessage.getUseProvidedPose() ? 0.3 : 0.2);
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
      for (IterativeClosestPointWorker worker : nodeIDToWorkerMap.values())
      {
         worker.setEnvironmentPointCloud(environmentPointCloud);
      }

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
      for (long id : nodeIDToWorkerMap.keySet())
      {
         IterativeClosestPointWorker worker = nodeIDToWorkerMap.get(id);
         if (worker.runICP(2))
            worker.publishResults();

         // If ICP isn't using the provided target pose, it'll update the SceneNode to the ICP worker's centroid
         if (!worker.isUsingTargetPoint())
         {
            SceneNode node = sceneGraph.getIDToNodeMap().get(id);
            RigidBodyTransform centroidToWorldTransform = new RigidBodyTransform(worker.getResultPose());
            node.getNodeToParentFrameTransform().set(centroidToWorldTransform);
            node.getNodeFrame().update();
         }
      }
   }

   private void addWorker(IterativeClosestPointRequest requestMessage)
   {
      PrimitiveRigidBodyShape shape = PrimitiveRigidBodyShape.fromByte(requestMessage.getShape());

      float xLength = requestMessage.getXLength();
      float yLength = requestMessage.getYLength();
      float zLength = requestMessage.getZLength();
      float xRadius = requestMessage.getXRadius();
      float yRadius = requestMessage.getYRadius();
      float zRadius = requestMessage.getZRadius();

      int numberOfPoints = approximateNumberOfPoints(shape, xLength, yLength, zLength, xRadius, yRadius, zRadius);
      IterativeClosestPointWorker worker = new IterativeClosestPointWorker(shape,
                                                                           xLength,
                                                                           yLength,
                                                                           zLength,
                                                                           xRadius,
                                                                           yRadius,
                                                                           zRadius,
                                                                           numberOfPoints,
                                                                           NUMBER_OF_CORRESPONDENCES,
                                                                           new FramePose3D(requestMessage.getProvidedPose()),
                                                                           ros2Helper,
                                                                           random);
      worker.setSceneNodeID(requestMessage.getNodeId());
      nodeIDToWorkerMap.putIfAbsent(requestMessage.getNodeId(), worker);
   }

   // FIXME: Maybe just allow the user to select number of points instead of trying to calculate it using the dimensions
   private int approximateNumberOfPoints(PrimitiveRigidBodyShape shape, float xLength, float yLength, float zLength, float xRadius, float yRadius, float zRadius)
   {
      double surfaceArea;
      switch (shape)
      {
         case BOX -> surfaceArea = 2.0 * (xLength * yLength + xLength * zLength + yLength * zLength);
         case PRISM -> surfaceArea = (xLength * zLength) + (2.0 * Math.sqrt(xLength * xLength + zLength * zLength));
         case CYLINDER -> surfaceArea = 2.0 * Math.PI * xRadius * (zLength + xRadius);
         case CONE -> surfaceArea = Math.PI * xRadius * zLength;
         default -> surfaceArea = 0.5;
      }

      return (int) (surfaceArea * 2000.0);
   }
}
