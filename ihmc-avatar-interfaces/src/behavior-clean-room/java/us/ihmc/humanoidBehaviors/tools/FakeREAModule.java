package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomPlanarRegionHandler;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;

public class FakeREAModule
{
   private final PlanarRegionsList constantPlanarRegions;

   private final IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionPublisher;

   private final HashMap<Integer, PlanarRegion> customPlanarRegions = new HashMap<>();

   public FakeREAModule(PlanarRegionsList constantPlanarRegions)
   {
      this.constantPlanarRegions = constantPlanarRegions;

      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, LIDARBasedREAModule.ROS2_ID.getNodeName());

      planarRegionPublisher = new IHMCROS2Publisher<>(ros2Node, PlanarRegionsListMessage.class, null, LIDARBasedREAModule.ROS2_ID);

      new ROS2Callback<>(ros2Node,
                         PlanarRegionsListMessage.class,
                         null,
                         LIDARBasedREAModule.ROS2_ID.qualifyMore(LIDARBasedREAModule.CUSTOM_REGION_QUALIFIER),
                         this::acceptCustomRegion);

      ExceptionHandlingThreadScheduler scheduler = new ExceptionHandlingThreadScheduler(getClass().getSimpleName());
      scheduler.schedule(this::process, 500, TimeUnit.MILLISECONDS);
   }

   private void process()
   {
      ArrayList<PlanarRegion> combinedRegionsList = new ArrayList<>(constantPlanarRegions.getPlanarRegionsAsList());

      synchronized (this)
      {
         combinedRegionsList.addAll(customPlanarRegions.values());
         PlanarRegionsList combinedRegions = new PlanarRegionsList(combinedRegionsList);
         PlanarRegionsListMessage message = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(combinedRegions);
         planarRegionPublisher.publish(message);
      }
   }

   private void acceptCustomRegion(PlanarRegionsListMessage message)
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
               customPlanarRegions.remove(region.getRegionId());
            }
            else
            {
               CustomPlanarRegionHandler.performConvexDecompositionIfNeeded(region);
               customPlanarRegions.put(region.getRegionId(), region);
            }
         }
      }
   }
}
