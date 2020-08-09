package us.ihmc.humanoidBehaviors.tools.perception;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.List;
import java.util.function.Supplier;

public class PeriodicPointCloudPublisher
{
   private final Supplier<List<Point3DReadOnly>> pointCloudSupplier;
   private final Supplier<Pose3DReadOnly> sensorPoseProvider;
   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> publisher;
   private final PausablePeriodicThread thread;

   public PeriodicPointCloudPublisher(Ros2Node ros2Node,
                                      ROS2Topic<StereoVisionPointCloudMessage> topic,
                                      double period,
                                      Supplier<List<Point3DReadOnly>> pointCloudSupplier,
                                      Supplier<Pose3DReadOnly> sensorPoseProvider)
   {
      this.pointCloudSupplier = pointCloudSupplier;
      this.sensorPoseProvider = sensorPoseProvider;

      publisher = ROS2Tools.createPublisher(ros2Node, topic);
      thread = new PausablePeriodicThread(getClass().getSimpleName(), period, this::publish);
   }

   private void publish()
   {
      List<Point3DReadOnly> pointCloud = pointCloudSupplier.get();

      if (pointCloud.isEmpty())
         return;

      Pose3DReadOnly sensorPose = sensorPoseProvider.get();

      Point3D[] pointArray = new Point3D[pointCloud.size()];
      int[] colors = new int[pointCloud.size()];
      for (int i = 0; i < pointCloud.size(); i++)
      {
         Point3DReadOnly point3DReadOnly = pointCloud.get(i);
         pointArray[i] = new Point3D(point3DReadOnly);
         colors[i] = 0;
      }

      StereoVisionPointCloudMessage message = PointCloudCompression.compressPointCloud(System.nanoTime(),
                                                                                       pointArray,
                                                                                       colors,
                                                                                       pointArray.length,
                                                                                       0.005,
                                                                                       null);

      message.getSensorPosition().set(sensorPose.getPosition());
      message.getSensorOrientation().set(sensorPose.getOrientation());

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
