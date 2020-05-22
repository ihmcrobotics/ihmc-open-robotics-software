package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.SimulatedDepthCamera;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.ros2.Ros2NodeInterface;

import java.util.function.Supplier;

public class Head360Simulator implements Supplier<PlanarRegionsList>
{
   private volatile PlanarRegionsList map;

   private RemoteSyncedHumanoidRobotState remoteSyncedHumanoidRobotState;
   private MovingReferenceFrame neckFrame;
   private SimulatedDepthCamera simulatedDepthCamera;

   public Head360Simulator(PlanarRegionsList map, DRCRobotModel robotModel, Ros2NodeInterface ros2Node)
   {
      this.map = map;

      remoteSyncedHumanoidRobotState = new RemoteSyncedHumanoidRobotState(robotModel, ros2Node);
      neckFrame = remoteSyncedHumanoidRobotState.getHumanoidRobotState().getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);
      simulatedDepthCamera = new SimulatedDepthCamera(neckFrame);
   }

   @Override
   public PlanarRegionsList get()
   {
      remoteSyncedHumanoidRobotState.pollHumanoidRobotState();

      if (remoteSyncedHumanoidRobotState.hasReceivedFirstMessage())
      {
         return simulatedDepthCamera.filterMapToVisible(map);
      }
      else
      {
         // blank result
         return new PlanarRegionsList();
      }
   }
}
