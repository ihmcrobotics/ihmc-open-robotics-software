package us.ihmc.avatar.sensors.realsense;

import map_sense.RawGPUPlanarRegionList;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.util.function.Consumer;

public class MapsenseTools
{
   private static final RigidBodyTransform zForwardXRightToZUpXForward = new RigidBodyTransform();
   static
   {
      zForwardXRightToZUpXForward.appendPitchRotation(Math.PI / 2.0);
      zForwardXRightToZUpXForward.appendYawRotation(-Math.PI / 2.0);
   }

   public static DelayFixedPlanarRegionsSubscription subscribeToPlanarRegionsWithDelayCompensation(ROS2NodeInterface ros2Node,
                                                                                                   DRCRobotModel robotModel,
                                                                                                   Consumer<Pair<Long, PlanarRegionsList>> callback)
   {
      return subscribeToPlanarRegionsWithDelayCompensation(ros2Node, robotModel, RosTools.MAPSENSE_REGIONS, callback);
   }

   public static DelayFixedPlanarRegionsSubscription subscribeToPlanarRegionsWithDelayCompensation(ROS2NodeInterface ros2Node,
                                                                                                   DRCRobotModel robotModel,
                                                                                                   String topic,
                                                                                                   Consumer<Pair<Long, PlanarRegionsList>> callback)
   {
      return new DelayFixedPlanarRegionsSubscription(ros2Node, robotModel, topic, callback);
   }

   public static AbstractRosTopicSubscriber<RawGPUPlanarRegionList> createROS1Callback(RosNodeInterface ros1Node, Consumer<RawGPUPlanarRegionList> callback)
   {
      return createROS1Callback(RosTools.MAPSENSE_REGIONS, ros1Node, callback);
   }

   public static AbstractRosTopicSubscriber<RawGPUPlanarRegionList> createROS1Callback(String topic,
                                                                                       RosNodeInterface ros1Node,
                                                                                       Consumer<RawGPUPlanarRegionList> callback)
   {
      AbstractRosTopicSubscriber<RawGPUPlanarRegionList> subscriber = new AbstractRosTopicSubscriber<RawGPUPlanarRegionList>(RawGPUPlanarRegionList._TYPE)
      {
         @Override
         public void onNewMessage(RawGPUPlanarRegionList rawGPUPlanarRegionList)
         {
            callback.accept(rawGPUPlanarRegionList);
         }
      };
      ros1Node.attachSubscriber(topic, subscriber);
      return subscriber;
   }

   public static RigidBodyTransformReadOnly getTransformFromCameraToWorld()
   {
      return zForwardXRightToZUpXForward;
   }
}
