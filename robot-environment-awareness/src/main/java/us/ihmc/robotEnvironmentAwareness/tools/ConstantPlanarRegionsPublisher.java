package us.ihmc.robotEnvironmentAwareness.tools;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.publisherTopicNameGenerator;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberTopicNameGenerator;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationNodeData;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.RegionFeaturesProvider;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

import javax.swing.*;
import java.io.File;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class ConstantPlanarRegionsPublisher
{
   private final PlanarRegionsList planarRegionsList;
   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName());
   private final REAPlanarRegionPublicNetworkProvider publisher;
   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;

   public ConstantPlanarRegionsPublisher(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
      this.publisher = new REAPlanarRegionPublicNetworkProvider(new ConstantPlanarRegionProvider(), ros2Node, publisherTopicNameGenerator, subscriberTopicNameGenerator);
   }

   public void start()
   {
      if(scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(() -> publisher.update(true), 0, 200, TimeUnit.MILLISECONDS);
      }
   }

   public void stop()
   {
      if (scheduled != null)
      {
         scheduled.cancel(true);
         scheduled = null;
      }

      if (executorService != null)
      {
         executorService.shutdownNow();
         executorService = null;
      }
   }

   private class ConstantPlanarRegionProvider implements RegionFeaturesProvider
   {
      @Override
      public List<PlanarRegionSegmentationNodeData> getSegmentationNodeData()
      {
         return null;
      }

      @Override
      public PlanarRegionsList getPlanarRegionsList()
      {
         return planarRegionsList;
      }

      @Override
      public int getNumberOfPlaneIntersections()
      {
         return 0;
      }

      @Override
      public LineSegment3D getIntersection(int index)
      {
         return null;
      }
   }

   // select header.txt inside the planar regions directory
   public static void main(String[] args)
   {
      JFileChooser fileChooser = new JFileChooser(new File("."));
      int returnVal = fileChooser.showOpenDialog(new JFrame());

      if (returnVal == JFileChooser.APPROVE_OPTION)
      {
         File file = fileChooser.getSelectedFile().getParentFile();
         PlanarRegionsList planarRegionsList = PlanarRegionFileTools.importPlanarRegionData(file);
         new ConstantPlanarRegionsPublisher(planarRegionsList).start();
      }
   }
}
