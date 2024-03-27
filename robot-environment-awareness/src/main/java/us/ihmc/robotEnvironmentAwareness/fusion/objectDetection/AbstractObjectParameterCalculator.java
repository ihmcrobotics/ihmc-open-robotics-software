package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.msg.dds.RegionOfInterest;
import us.ihmc.commons.MathTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.StereoPointCloudCompression;
import us.ihmc.communication.packets.StereoPointCloudCompression.PointCoordinateConsumer;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2TopicNameTools;

public abstract class AbstractObjectParameterCalculator<T extends Packet<?>>
{
   private static final CameraPinholeBrown intrinsicParameters = PointCloudProjectionHelper.multisenseOnCartIntrinsicParameters;
   protected final List<Point3DBasics> pointCloudToCalculate;
   protected final RegionOfInterest objectROI = new RegionOfInterest();

   private final Class<T> messageType;
   private final ROS2PublisherBasics<T> packetPublisher;
   protected final AtomicReference<T> newPacket = new AtomicReference<>(null);

   public AbstractObjectParameterCalculator(ROS2Node ros2Node, Class<T> messageType)
   {
      this.messageType = messageType;
      pointCloudToCalculate = new ArrayList<Point3DBasics>();
      packetPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, messageType, ROS2Tools.IHMC_ROOT);
      newPacket.set(ROS2TopicNameTools.newMessageInstance(messageType));
   }

   public void trimPointCloudInROI(StereoVisionPointCloudMessage pointCloudMessage, RegionOfInterest roi)
   {
      StereoPointCloudCompression.decompressPointCloud(pointCloudMessage, new PointCoordinateConsumer()
      {
         @Override
         public void accept(double x, double y, double z)
         {
            Point3D point = new Point3D(x, y, z);
            Point2D projectedPixel = new Point2D();
            PointCloudProjectionHelper.projectMultisensePointCloudOnImage(point, projectedPixel, intrinsicParameters);

            if (MathTools.intervalContains(projectedPixel.getX(), roi.getXOffset(), roi.getXOffset() + roi.getWidth()))
            {
               if (MathTools.intervalContains(projectedPixel.getY(), roi.getYOffset(), roi.getYOffset() + roi.getHeight()))
               {
                  pointCloudToCalculate.add(point);
               }
            }
         }
      });
      objectROI.set(roi);
      LogTools.info("total number of points in roi " + pointCloudToCalculate.size());
   }

   public void initialize()
   {
      newPacket.set(ROS2TopicNameTools.newMessageInstance(messageType));
      pointCloudToCalculate.clear();
   }

   public void publish()
   {
      packetPublisher.publish(newPacket.get());
   }

   /**
    * Some object might need multiple ROIs to calculate out the parameters of the required packet.
    */
   public abstract void calculate(RegionOfInterest... additionalROIs);
}