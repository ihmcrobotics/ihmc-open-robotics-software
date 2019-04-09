package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.FlatGroundPlanarRegionParametersMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import org.apache.poi.ss.formula.functions.T;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2ModuleIdentifier;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.tools.thread.ExceptionPrintingThreadScheduler;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.TimeUnit;

/**
 * Supply flat ground regions to REA. TODO: Add reset functionality and/or sizing and positioning
 */
public class FlatGroundPlanarRegionPublisher
{
   public static final ROS2ModuleIdentifier ROS2_ID = new ROS2ModuleIdentifier("flat_ground_planar_region_publisher",
                                                                               ROS2Tools.FLAT_GROUND_REGION_PUBLISHER);
   private final ExceptionPrintingThreadScheduler scheduler;
   private final PlanarRegionsListMessage flatRegionMessage;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> publisher;

   public FlatGroundPlanarRegionPublisher()
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2_ID.getNodeName());

      flatRegionMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(new PlanarRegionsList(createFlatGroundRegion()));

      scheduler = new ExceptionPrintingThreadScheduler(getClass().getSimpleName());

      new ROS2Callback<>(ros2Node, FlatGroundPlanarRegionParametersMessage.class, null, ROS2_ID, this::acceptParameters);

      publisher = new IHMCROS2Publisher<>(ros2Node,
                                          PlanarRegionsListMessage.class,
                                          null,
                                          LIDARBasedREAModule.ROS2_ID.qualifyMore(LIDARBasedREAModule.CUSTOM_REGION_QUALIFIER));

      scheduler.schedule(this::publish, 500, TimeUnit.MILLISECONDS);
   }

   private void acceptParameters(FlatGroundPlanarRegionParametersMessage message)
   {
      if (message.getEnable())
      {
         scheduler.schedule(this::publish, 500, TimeUnit.MILLISECONDS);
      }
      else
      {
         scheduler.shutdown();
      }
   }

   private void publish()
   {
      publisher.publish(flatRegionMessage);
   }

   private PlanarRegion createFlatGroundRegion()
   {
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();  // start with a flat ground region
      convexPolygon.addVertex(10.0, 10.0);
      convexPolygon.addVertex(-10.0, 10.0);
      convexPolygon.addVertex(-10.0, -10.0);
      convexPolygon.addVertex(10.0, -10.0);
      convexPolygon.update();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      transformToWorld.setTranslationZ(-0.0001);
      PlanarRegion planarRegion = new PlanarRegion(transformToWorld, convexPolygon);
      planarRegion.setRegionId(1780);
      return planarRegion;
   }
}
