package us.ihmc.humanoidBehaviors.ui.graphics.live;

import java.time.LocalDateTime;
import java.util.concurrent.ExecutorService;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

public class JavaFXLivePlanarRegionsGraphic extends PlanarRegionsGraphic
{
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);

   private final ExecutorService executorService = ThreadTools.newSingleThreadExecutor(getClass().getSimpleName());

   private boolean acceptNewRegions = true;
   private volatile PlanarRegionsList latestPlanarRegionsList = new PlanarRegionsList(); // prevent NPEs

   public JavaFXLivePlanarRegionsGraphic(ROS2Node ros2Node, boolean initializeToFlatGround)
   {
      this(ros2Node, ROS2Tools.LIDAR_REA_REGIONS, initializeToFlatGround);
   }

   public JavaFXLivePlanarRegionsGraphic(ROS2Node ros2Node, ROS2Topic<PlanarRegionsListMessage> topic, boolean initializeToFlatGround)
   {
      super(initializeToFlatGround);

      new ROS2Callback<>(ros2Node, PlanarRegionsListMessage.class, topic, this::acceptPlanarRegions);
      animationTimer.start();
   }

   public JavaFXLivePlanarRegionsGraphic(boolean initializeToFlatGround)
   {
      super(initializeToFlatGround);
      animationTimer.start();
   }

   public synchronized void acceptPlanarRegions(PlanarRegionsListMessage incomingData)
   {
      if (acceptNewRegions)
      {
         synchronized (this) // just here for clear method
         {
            executorService.submit(() -> convertAndGenerateMesh(incomingData));
         }
      }
   }

   public synchronized void acceptPlanarRegions(PlanarRegionsList incomingData)
   {
      if (acceptNewRegions)
      {
         synchronized (this) // just here for clear method
         {
            executorService.submit(() ->
            {
               this.latestPlanarRegionsList = incomingData;
               LogTools.trace("Received regions from behavior: {}: {}", LocalDateTime.now(), incomingData.hashCode());
               generateMeshes(incomingData);
            });
         }
      }
   }

   private synchronized void convertAndGenerateMesh(PlanarRegionsListMessage incomingData)
   {
      PlanarRegionsList latestPlanarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(incomingData);
      this.latestPlanarRegionsList = latestPlanarRegionsList;
      LogTools.debug("Generating mesh for sequenceId: {}", incomingData.getSequenceId());
      generateMeshes(latestPlanarRegionsList); // important not to execute this in either ROS2 or JavaFX threads
   }

   private void handle(long now)
   {
      super.update();
   }

   public void setEnabled(boolean enabled)
   {
      acceptNewRegions = enabled;
   }

   public synchronized void clear()
   {
      synchronized (this) // to avoid collision with ROS 2 thread
      {
         if (latestPlanarRegionsList != null)
            latestPlanarRegionsList.clear();
         executorService.submit(() -> generateMeshes(latestPlanarRegionsList));
      }
   }

   public synchronized void setAcceptNewRegions(boolean acceptNewRegions)
   {
      this.acceptNewRegions = acceptNewRegions;
   }

   public synchronized PlanarRegionsList getLatestPlanarRegionsList()
   {
      return latestPlanarRegionsList.copy();
   }

   public void destroy()
   {
      executorService.shutdownNow();
   }
}
