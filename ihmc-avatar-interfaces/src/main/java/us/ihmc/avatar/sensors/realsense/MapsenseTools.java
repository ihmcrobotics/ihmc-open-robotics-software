package us.ihmc.avatar.sensors.realsense;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;

import java.util.function.Consumer;

public class MapsenseTools
{
   private static final RigidBodyTransform zForwardXRightToZUpXForward = new RigidBodyTransform();
   static
   {
      zForwardXRightToZUpXForward.appendPitchRotation(Math.PI / 2.0);
      zForwardXRightToZUpXForward.appendYawRotation(-Math.PI / 2.0);
   }

   public static DelayFixedPlanarRegionsSubscription subscribeToPlanarRegionsWithDelayCompensation(ROS2Node ros2Node,
                                                                                                   DRCRobotModel robotModel,
                                                                                                   Consumer<Pair<Long, PlanarRegionsList>> callback)
   {
      return subscribeToPlanarRegionsWithDelayCompensation(ros2Node, robotModel, "/mapsense/planar_regions", callback);
   }

   public static DelayFixedPlanarRegionsSubscription subscribeToPlanarRegionsWithDelayCompensation(ROS2Node ros2Node,
                                                                                                   DRCRobotModel robotModel,
                                                                                                   String topic,
                                                                                                   Consumer<Pair<Long, PlanarRegionsList>> callback)
   {
      return new DelayFixedPlanarRegionsSubscription(ros2Node, robotModel, topic, callback);
   }

   public static RigidBodyTransformReadOnly getTransformFromCameraToWorld()
   {
      return zForwardXRightToZUpXForward;
   }
}
