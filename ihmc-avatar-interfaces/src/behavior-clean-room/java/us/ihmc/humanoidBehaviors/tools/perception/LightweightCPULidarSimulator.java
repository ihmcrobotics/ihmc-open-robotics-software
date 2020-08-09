package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.SimulatedLidar;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2NodeInterface;

import java.util.function.Supplier;

public class LightweightCPULidarSimulator implements Supplier<PlanarRegionsList>
{
   private volatile PlanarRegionsList map;

   private RemoteSyncedRobotModel syncedRobot;
   private MovingReferenceFrame neckFrame;
   private SimulatedLidar simulatedLidar;

   public LightweightCPULidarSimulator(PlanarRegionsList map, DRCRobotModel robotModel, Ros2NodeInterface ros2Node)
   {
      // start thread for point gathering - high update rate
      // thread for polygonizing - low update rate


   }

   @Override
   public PlanarRegionsList get()
   {
      // volatile or lock on access

      return null;
   }
}
