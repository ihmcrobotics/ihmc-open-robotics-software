package us.ihmc.perception.sceneGraph.ros2;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.SceneGraph;

import java.util.function.Supplier;

public class ROS2SceneGraphUpdateThread extends RepeatingTaskThread
{
   private final ROS2SceneGraph sceneGraph;
   private final DetectionManager detectionManager;
   private final Supplier<ReferenceFrame> robotPelvisFrameSupplier;

   public ROS2SceneGraphUpdateThread(ROS2SceneGraph sceneGraph, DetectionManager detectionManager, Supplier<ReferenceFrame> robotPelvisFrameSupplier)
   {
      super(ROS2SceneGraphUpdateThread.class.getSimpleName());
      setFrequencyLimit(SceneGraph.UPDATE_FREQUENCY);
      this.sceneGraph = sceneGraph;
      this.detectionManager = detectionManager;
      this.robotPelvisFrameSupplier = robotPelvisFrameSupplier;
   }

   @Override
   protected void runTask()
   {
      sceneGraph.updateSubscription();

      // TODO: Add ArUco

      sceneGraph.updateDetections(detectionManager);

      // TODO: Add planar regions

      ReferenceFrame pelvisFrame = robotPelvisFrameSupplier.get();
      sceneGraph.updateOnRobotOnly(pelvisFrame);
      sceneGraph.updatePublication();

      // TODO: add behavior tree?
   }
}
