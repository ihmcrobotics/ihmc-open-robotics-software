package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.SimulatedDepthCamera;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.ros2.Ros2NodeInterface;

import java.util.function.Supplier;

public class Head360Simulator implements Supplier<PlanarRegionsList>
{
   private final PointCloudPolygonizer polygonizer;
   private volatile PlanarRegionsList map;

   private RemoteSyncedRobotModel syncedRobot;
   private MovingReferenceFrame neckFrame;
   private SimulatedDepthCamera simulatedDepthCamera;

   public Head360Simulator(PlanarRegionsList map, DRCRobotModel robotModel, Ros2NodeInterface ros2Node)
   {
      this.map = map;

      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);
      neckFrame = syncedRobot.getReferenceFrames().getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);
      simulatedDepthCamera = new SimulatedDepthCamera(Double.NaN, Double.NaN, Double.POSITIVE_INFINITY, neckFrame);
      polygonizer = new PointCloudPolygonizer();
   }

   @Override
   public PlanarRegionsList get()
   {
      syncedRobot.update();

      if (syncedRobot.hasReceivedFirstMessage())
      {
         return polygonizer.polygonize(simulatedDepthCamera.computeRegionPointMapFrame(map));
      }
      else
      {
         // blank result
         return new PlanarRegionsList();
      }
   }
}
