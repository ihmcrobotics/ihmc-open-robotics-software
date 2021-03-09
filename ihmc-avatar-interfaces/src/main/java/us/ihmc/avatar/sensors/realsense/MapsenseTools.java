package us.ihmc.avatar.sensors.realsense;

import map_sense.RawGPUPlanarRegionList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.utilities.ros.RosMainNode;
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

   public static AbstractRosTopicSubscriber<RawGPUPlanarRegionList> createROS1Callback(RosMainNode ros1Node, Consumer<RawGPUPlanarRegionList> callback)
   {
      return createROS1Callback(RosTools.MAPSENSE_REGIONS, ros1Node, callback);
   }

   public static AbstractRosTopicSubscriber<RawGPUPlanarRegionList> createROS1Callback(String topic,
                                                                                       RosMainNode ros1Node,
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
