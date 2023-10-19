package us.ihmc.behaviors.tools.perception;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudMessageTools;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.function.Supplier;

public class PeriodicPointCloudPublisher
{
   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> publisher;
   private final PausablePeriodicThread thread;
   private final RecyclingArrayList<Point3D32> points;
   private final Supplier<? extends Pose3DReadOnly> poseSupplier;

   public PeriodicPointCloudPublisher(ROS2NodeInterface ros2Node,
                                      ROS2Topic<StereoVisionPointCloudMessage> topic,
                                      double period,
                                      RecyclingArrayList<Point3D32> points,
                                      Supplier<? extends Pose3DReadOnly> poseSupplier)
   {
      this.points = points;
      this.poseSupplier = poseSupplier;

      publisher = ROS2Tools.createPublisher(ros2Node, topic);
      thread = new PausablePeriodicThread(getClass().getSimpleName(), period, this::publish);
   }

   private void publish()
   {
      StereoVisionPointCloudMessage message = PointCloudMessageTools.toStereoVisionPointCloudMessage(points, poseSupplier.get());
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
