package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.SimulatedDepthCamera;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.function.Supplier;

public class MultisenseHeadStereoSimulator implements Supplier<PlanarRegionsList>
{
   private volatile PlanarRegionsList map;

   private RemoteSyncedRobotModel syncedRobot;
   private MovingReferenceFrame neckFrame;
   private SimulatedDepthCamera simulatedDepthCamera;

   public MultisenseHeadStereoSimulator(PlanarRegionsList map,
                                        DRCRobotModel robotModel,
                                        ROS2NodeInterface ros2Node,
                                        double range,
                                        int sphereScanSize)
   {
      this.map = map;

      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);
      neckFrame = syncedRobot.getReferenceFrames().getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);
      double verticalFOV = 80.0;
      double horizontalFOV = 80.0;
      simulatedDepthCamera = new SimulatedDepthCamera(verticalFOV, horizontalFOV, range, sphereScanSize, neckFrame);
   }

   public PlanarRegionsList computeRegions()
   {
      syncedRobot.update();

      if (syncedRobot.hasReceivedFirstMessage())
      {
         return simulatedDepthCamera.computeAndPolygonize(map);
      }
      else
      {
         // blank result
         return new PlanarRegionsList();
      }
   }
   @Override
   public PlanarRegionsList get()
   {
      return computeRegions();
   }

   public void setMap(PlanarRegionsList map)
   {
      this.map = map;
   }

}
