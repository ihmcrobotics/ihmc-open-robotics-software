package us.ihmc.perception;

import perception_msgs.msg.dds.IterativeClosestPointRequest;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.util.HashMap;
import java.util.Random;

public class IterativeClosestPointManager
{
   private static final double ICP_WORK_FREQUENCY = 20.0;
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
      requestMessageSubscription.addCallback(requestMessage ->
      {
         System.out.println("Reveived message");
         long nodeID = requestMessage.getNodeId();

         if (!nodeIDToWorkerMap.containsKey(nodeID))     // ICP Worker requested (add new)
         {
            System.out.println("Adding worker " + nodeID);
            addWorker(requestMessage);
         }
         else if (!requestMessage.getRunIcp())           // ICP Worker no longer needed (remove)
         {
            System.out.println("Removing worker " + nodeID);
            nodeIDToWorkerMap.remove(nodeID);
         }
         else                                            // ICP Worker exists (ensure correct size)
         {
            System.out.println("Updating worker " + nodeID);
            IterativeClosestPointWorker worker = nodeIDToWorkerMap.get(nodeID);

            PrimitiveRigidBodyShape shape = PrimitiveRigidBodyShape.fromByte(requestMessage.getShape());

            float requestWidth = requestMessage.getYLength();
            float requestHeight = requestMessage.getZLength();
            float requestDepth = requestMessage.getXLength();
            float requestLength = requestMessage.getZLength();
            float requestRadius = requestMessage.getXRadius();

            boolean sizeChanged = false;
            switch (shape)
            {
               case BOX ->
               {
                  sizeChanged |= !MathTools.epsilonEquals(worker.getWidth(), requestWidth, EPSILON);
                  sizeChanged |= !MathTools.epsilonEquals(worker.getHeight(), requestHeight, EPSILON);
                  sizeChanged |= !MathTools.epsilonEquals(worker.getDepth(), requestDepth, EPSILON);
               }
               case CONE, CYLINDER ->
               {
                  sizeChanged |= !MathTools.epsilonEquals(worker.getLength(), requestLength, EPSILON);
                  sizeChanged |= !MathTools.epsilonEquals(worker.getRadius(), requestRadius, EPSILON);
               }
            }

            if (sizeChanged)
            {
               int newNumberOfPoints = approximateNumberOfPoints(shape, requestWidth, requestHeight, requestDepth, requestLength, requestRadius);
               worker.changeSize(requestWidth, requestHeight, requestDepth, requestLength, requestRadius, newNumberOfPoints);
            }

            worker.setTargetPoint(requestMessage.getProvidedPose().getPosition());
            worker.useProvidedTargetPoint(requestMessage.getUseProvidedPose());
            worker.setSegmentSphereRadius(requestMessage.getUseProvidedPose() ? 0.3 : 0.2);
         }
      });

      workerThread = new RestartableThrottledThread("ICPWorkers", ICP_WORK_FREQUENCY, this::runWorkers);
   }

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

   private void runWorkers()
   {
      for (long id : nodeIDToWorkerMap.keySet())
      {
         IterativeClosestPointWorker worker = nodeIDToWorkerMap.get(id);
         worker.runICP(2);

         if (!worker.isUsingTargetPoint())
         {
            System.out.println("Updating node using ICP");
            SceneNode node = sceneGraph.getIDToNodeMap().get(id);
            RigidBodyTransform centroidToWorldTransform = new RigidBodyTransform(worker.getOrientation(), worker.getCentroid());
            node.getNodeToParentFrameTransform().set(centroidToWorldTransform);
            node.getNodeFrame().update();
         }
      }
   }

   private void addWorker(IterativeClosestPointRequest requestMessage)
   {
      PrimitiveRigidBodyShape shape = PrimitiveRigidBodyShape.fromByte(requestMessage.getShape());

      float width = 0.0f;
      float height = 0.0f;
      float depth = 0.0f;
      float length = 0.0f;
      float radius = 0.0f;

      switch (shape)
      {
         case BOX ->
         {
            width = requestMessage.getYLength();
            height = requestMessage.getZLength();
            depth = requestMessage.getXLength();
         }
         case CONE, CYLINDER ->
         {
            length = requestMessage.getZLength();
            radius = requestMessage.getXRadius();
         }
         default -> throw new RuntimeException("Unsupperted Shape"); // FIXME: Probably do something else here
      }

      int numberOfPoints = approximateNumberOfPoints(shape, width, height, depth, length, radius);
      IterativeClosestPointWorker worker = new IterativeClosestPointWorker(shape, width, height, depth, length, radius, numberOfPoints, ros2Helper, random);
      worker.setSceneNodeID(requestMessage.getNodeId());
      nodeIDToWorkerMap.putIfAbsent(requestMessage.getNodeId(), worker);
   }

   private int approximateNumberOfPoints(PrimitiveRigidBodyShape shape, float width, float height, float depth, float length, float radius)
   {
      int numberOfPoints;
      switch (shape)
      {
         case BOX ->
         {
            return (int) (5000.0 * (width * height + width * depth + height * depth));
         }
         case CONE, CYLINDER ->
         {
            return (int) (5000.0 * (Math.PI * radius * length)); // dumb way of figuring out number of points
         }
         default ->
         {
            return 1000;
         }
      }
   }
}
