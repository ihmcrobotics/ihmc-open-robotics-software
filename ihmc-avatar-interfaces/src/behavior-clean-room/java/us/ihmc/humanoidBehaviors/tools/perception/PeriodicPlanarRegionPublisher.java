package us.ihmc.humanoidBehaviors.tools.perception;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2NodeInterface;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.function.Supplier;

public class PeriodicPlanarRegionPublisher
{
   private final Supplier<PlanarRegionsList> planarRegionsListSupplier;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> publisher;
   private final PausablePeriodicThread thread;

   public PeriodicPlanarRegionPublisher(Ros2NodeInterface ros2Node,
                                        ROS2Topic<PlanarRegionsListMessage> topic,
                                        double period,
                                        Supplier<PlanarRegionsList> planarRegionsListSupplier)
   {
      this.planarRegionsListSupplier = planarRegionsListSupplier;

      publisher = ROS2Tools.createPublisher(ros2Node, topic);
      thread = new PausablePeriodicThread(getClass().getSimpleName(), period, this::publish);
   }

   private void publish()
   {
      PlanarRegionsListMessage message = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsListSupplier.get());
      publisher.publish(message);
   }

   public void start()
   {
      thread.start();
   }

   public void stop()
   {
      thread.stop();
   }
}
