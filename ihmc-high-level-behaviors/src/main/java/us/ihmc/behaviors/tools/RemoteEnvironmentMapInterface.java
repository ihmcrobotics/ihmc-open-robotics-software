package us.ihmc.behaviors.tools;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomPlanarRegionHandler;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;

public class RemoteEnvironmentMapInterface
{
   private volatile PlanarRegionsList latestCombinedRegionsList = new PlanarRegionsList();

   private volatile PlanarRegionsList realsenseSLAMRegions = new PlanarRegionsList();
   private final HashMap<Integer, PlanarRegion> supportRegions = new HashMap<>();

   private final Stopwatch stopwatch = new Stopwatch();

   private final ArrayList<Consumer<PlanarRegionsList>> callbacks = new ArrayList<>();

   public RemoteEnvironmentMapInterface(ROS2NodeInterface ros2Node)
   {
      new ROS2Callback<>(ros2Node, PlanarRegionsListMessage.class, PerceptionAPI.REALSENSE_SLAM_MODULE.withOutput(), this::acceptRealsenseSLAMRegions);

      // used to be "/ihmc/rea/custom_region/input/planar_regions_list"
      new ROS2Callback<>(ros2Node, PlanarRegionsListMessage.class, PerceptionAPI.REA_SUPPORT_REGIONS.withOutput(), this::acceptAdditionalRegionList);
   }

   public synchronized PlanarRegionsList getLatestCombinedRegionsList()
   {
      return latestCombinedRegionsList;
   }

   public void addPlanarRegionsListCallback(Consumer<PlanarRegionsList> planarRegionsListConsumer)
   {
      callbacks.add(planarRegionsListConsumer);
   }

   private void acceptRealsenseSLAMRegions(PlanarRegionsListMessage message)
   {
      stopwatch.start();

      synchronized (this)
      {
         realsenseSLAMRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);

         combineRegions();
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
               supportRegions.remove(region.getRegionId());
            }
            else
            {
               CustomPlanarRegionHandler.performConvexDecompositionIfNeeded(region);
               supportRegions.put(region.getRegionId(), region);
            }
         }

         combineRegions();
      }
   }

   private void combineRegions()
   {
      ArrayList<PlanarRegion> combinedRegionsList = new ArrayList<>();
      combinedRegionsList.addAll(realsenseSLAMRegions.getPlanarRegionsAsList());
      combinedRegionsList.addAll(supportRegions.values());
      latestCombinedRegionsList = new PlanarRegionsList(combinedRegionsList);

      for (Consumer<PlanarRegionsList> callback : callbacks)
      {
         callback.accept(latestCombinedRegionsList);
      }
   }

   /**
    * NaN is no planar regions yet received.
    * @return time since planar regions last updated
    */
   public double timeSinceLastUpdate()
   {
      return stopwatch.lapElapsed();
   }

   public boolean getPlanarRegionsListExpired(double expirationDuration)
   {
      return Double.isNaN(timeSinceLastUpdate()) || timeSinceLastUpdate() > expirationDuration;
   }
}
