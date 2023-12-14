package us.ihmc.perception;

import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.util.HashMap;
import java.util.Random;

// TODO: Add subscriber to ICP Request Message, finish class
public class IterativeClosestPointManager
{
   private static final double ICP_WORK_FREQUENCY = 20.0;

   private final SceneGraph sceneGraph;

   private final OpenCLManager openCLManager = new OpenCLManager();
   private final Random random = new Random(System.nanoTime());
   private final HashMap<ROS2DemandGraphNode, IterativeClosestPointWorker> sceneNodeToWorkerMap = new HashMap<>();

   private final RestartableThrottledThread workerThread;

   public IterativeClosestPointManager(SceneGraph sceneGraph)
   {
      this.sceneGraph = sceneGraph;
      workerThread = new RestartableThrottledThread("ICPWorkers", ICP_WORK_FREQUENCY, this::runWorkers);
   }

   private void runWorkers()
   {
      for (ROS2DemandGraphNode demandNode : sceneNodeToWorkerMap.keySet())
      {
//         if (demandNode.isDemanded())
//            sceneNodeToWorkerMap.get(demandNode).runICP();
      }
   }

   public void addWorker(ROS2DemandGraphNode workerDemandNode)
   {

   }
}
