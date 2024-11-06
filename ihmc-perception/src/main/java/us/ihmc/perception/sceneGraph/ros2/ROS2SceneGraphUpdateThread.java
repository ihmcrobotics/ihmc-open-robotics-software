package us.ihmc.perception.sceneGraph.ros2;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.function.Supplier;

public class ROS2SceneGraphUpdateThread extends RepeatingTaskThread
{
   private final ROS2SceneGraph sceneGraph;
   private final DetectionManager detectionManager;
   private final Supplier<ReferenceFrame> robotPelvisFrameSupplier;
   private final TypedNotification<FramePlanarRegionsList> planarRegionsNotification;

   public ROS2SceneGraphUpdateThread(ROS2SceneGraph sceneGraph,
                                     DetectionManager detectionManager,
                                     Supplier<ReferenceFrame> robotPelvisFrameSupplier,
                                     TypedNotification<FramePlanarRegionsList> planarRegionsNotification)
   {
      super(ROS2SceneGraphUpdateThread.class.getSimpleName());
      setFrequencyLimit(SceneGraph.UPDATE_FREQUENCY);
      this.sceneGraph = sceneGraph;
      this.detectionManager = detectionManager;
      this.robotPelvisFrameSupplier = robotPelvisFrameSupplier;
      this.planarRegionsNotification = planarRegionsNotification;
   }

   @Override
   protected void runTask()
   {
      sceneGraph.updateSubscription();

      // TODO: Add ArUco

      sceneGraph.updateDetections(detectionManager);

      // Update planar regions for door nodes
      if (planarRegionsNotification.poll())
      {
         FramePlanarRegionsList framePlanarRegions = planarRegionsNotification.read();
         PlanarRegionsList planarRegionsInWorldFrame = framePlanarRegions.getPlanarRegionsList();
         planarRegionsInWorldFrame.applyTransform(framePlanarRegions.getSensorToWorldFrameTransform());

         for (SceneNode sceneNode : sceneGraph.getSceneNodesByID())
         {
            if (sceneNode instanceof DoorNode doorNode)
               doorNode.getDoorPanel().filterAndSetPlanarRegionFromPlanarRegionsList(planarRegionsInWorldFrame);
         }
      }

      ReferenceFrame pelvisFrame = robotPelvisFrameSupplier.get();
      sceneGraph.updateOnRobotOnly(pelvisFrame);
      sceneGraph.updatePublication();

      // TODO: add behavior tree?
   }
}
