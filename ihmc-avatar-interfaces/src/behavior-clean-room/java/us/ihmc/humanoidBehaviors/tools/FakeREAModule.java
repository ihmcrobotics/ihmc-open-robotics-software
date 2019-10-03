package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomPlanarRegionHandler;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.ArrayList;
import java.util.HashMap;

public class FakeREAModule
{
   private volatile PlanarRegionsList map;

   private final IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionPublisher;
   private RemoteSyncedHumanoidRobotState remoteSyncedHumanoidRobotState;

   private final HashMap<Integer, PlanarRegion> additionalPlanarRegions = new HashMap<>();
   private final PausablePeriodicThread thread;
   private MovingReferenceFrame neckFrame;
   private FakeREAVirtualCameraFOV virtualCameraFOV;

   public FakeREAModule(PlanarRegionsList map)
   {
      this(map, null);
   }

   public FakeREAModule(PlanarRegionsList map, DRCRobotModel robotModel)
   {
      this.map = map;

      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA.getNodeName());

      planarRegionPublisher = new IHMCROS2Publisher<>(ros2Node, PlanarRegionsListMessage.class, null, ROS2Tools.REA);

      if (robotModel != null)
      {
         remoteSyncedHumanoidRobotState = new RemoteSyncedHumanoidRobotState(robotModel, ros2Node);
         neckFrame = remoteSyncedHumanoidRobotState.getHumanoidRobotState().getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);
         virtualCameraFOV = new FakeREAVirtualCameraFOV(Math.toRadians(30.0), Math.toRadians(70.0), neckFrame);
      }

      new ROS2Callback<>(ros2Node,
                         PlanarRegionsListMessage.class,
                         null,
                         ROS2Tools.REA.qualifyMore(ROS2Tools.REA_CUSTOM_REGION_QUALIFIER),
                         this::acceptAdditionalRegionList);

      thread = new PausablePeriodicThread(this::process, 0.5, getClass().getSimpleName());
   }

   public void start()
   {
      thread.start();
   }

   public void stop()
   {
      thread.stop();
   }

   public void setMap(PlanarRegionsList map)
   {
      this.map = map;
   }

   private void process()
   {
      remoteSyncedHumanoidRobotState.pollHumanoidRobotState();
      ArrayList<PlanarRegion> combinedRegionsList = new ArrayList<>();
      if (remoteSyncedHumanoidRobotState != null)
      {
         if (remoteSyncedHumanoidRobotState.hasReceivedFirstMessage())
         {
            combinedRegionsList.addAll(virtualCameraFOV.filterMapToVisible(map).getPlanarRegionsAsList());
         }
         else
         {
            // blank result
         }
      }
      else
      {
         combinedRegionsList.addAll(map.getPlanarRegionsAsList());
      }

      synchronized (this)
      {
         combinedRegionsList.addAll(additionalPlanarRegions.values());
         PlanarRegionsList combinedRegions = new PlanarRegionsList(combinedRegionsList);
         PlanarRegionsListMessage message = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(combinedRegions);
         planarRegionPublisher.publish(message);
      }
   }

   private void acceptAdditionalRegionList(PlanarRegionsListMessage message)
   {
      PlanarRegionsList newRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);

      synchronized (this)
      {
         for (PlanarRegion region : newRegions.getPlanarRegionsAsList())
         {
            if (region.getRegionId() == PlanarRegion.NO_REGION_ID)
            {
               continue;
            }
            else if (region.isEmpty())
            {
               additionalPlanarRegions.remove(region.getRegionId());
            }
            else
            {
               CustomPlanarRegionHandler.performConvexDecompositionIfNeeded(region);
               additionalPlanarRegions.put(region.getRegionId(), region);
            }
         }
      }
   }
}
