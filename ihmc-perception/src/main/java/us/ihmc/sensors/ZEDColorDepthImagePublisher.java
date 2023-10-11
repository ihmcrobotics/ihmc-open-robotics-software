package us.ihmc.sensors;

import org.bytedeco.javacpp.BytePointer;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.cuda.CUDAImageEncoder;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

import java.util.function.Supplier;

public class ZEDColorDepthImagePublisher
{
   private final ZEDModelData zedModelData;

   private final Supplier<ReferenceFrame> sensorFrameSupplier;
   private final FramePose3D leftCameraFramePose = new FramePose3D();
   private final SideDependentList<FramePose3D> cameraPosesInDepthFrame = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   private final SideDependentList<IHMCROS2Publisher<ImageMessage>> ros2ColorImagePublishers;
   private final IHMCROS2Publisher<ImageMessage> ros2DepthImagePublisher;

   private final CUDAImageEncoder imageEncoder = new CUDAImageEncoder();

   private long lastDepthSequenceNumber = -1L;
   private final SideDependentList<Long> lastColorSequenceNumbers = new SideDependentList<>(-1L, -1L);

   public ZEDColorDepthImagePublisher(ZEDModelData zedModelData,
                                      SideDependentList<ROS2Topic<ImageMessage>> colorTopics,
                                      ROS2Topic<ImageMessage> depthTopic,
                                      Supplier<ReferenceFrame> sensorFrameSupplier)
   {
      this.zedModelData = zedModelData;
      this.sensorFrameSupplier = sensorFrameSupplier;
      cameraPosesInDepthFrame.get(RobotSide.RIGHT).getPosition().subY(2.0 * zedModelData.getCenterToCameraDistance());

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "zed2_node");
      ros2ColorImagePublishers = new SideDependentList<>(ROS2Tools.createPublisher(ros2Node, colorTopics.get(RobotSide.LEFT), ROS2QosProfile.BEST_EFFORT()),
                                                         ROS2Tools.createPublisher(ros2Node, colorTopics.get(RobotSide.RIGHT), ROS2QosProfile.BEST_EFFORT()));
      ros2DepthImagePublisher = ROS2Tools.createPublisher(ros2Node, depthTopic, ROS2QosProfile.BEST_EFFORT());
   }

   public void publishDepthImage(RawImage gpuDepthImage16UC1)
   {
      // Perform safety checks
      if (gpuDepthImage16UC1 != null &&
          !gpuDepthImage16UC1.isEmpty() &&
          gpuDepthImage16UC1.getSequenceNumber() != lastDepthSequenceNumber)
      {
         // Encode depth image to png
         BytePointer depthPNGPointer = new BytePointer();
         OpenCVTools.compressImagePNG(gpuDepthImage16UC1.getCpuImageMatrix(), depthPNGPointer);

         // Get present position of sensor
         leftCameraFramePose.setToZero(sensorFrameSupplier.get());
         leftCameraFramePose.getPosition().addY(zedModelData.getCenterToCameraDistance());
         leftCameraFramePose.changeFrame(ReferenceFrame.getWorldFrame());

         // Publish image
         ImageMessage depthImageMessage = new ImageMessage();
         ImageMessageDataPacker imageMessageDataPacker = new ImageMessageDataPacker(depthPNGPointer.limit());
         imageMessageDataPacker.pack(depthImageMessage, depthPNGPointer);
         MessageTools.toMessage(gpuDepthImage16UC1.getAcquisitionTime(), depthImageMessage.getAcquisitionTime());
         depthImageMessage.setFocalLengthXPixels(gpuDepthImage16UC1.getFocalLengthX());
         depthImageMessage.setFocalLengthYPixels(gpuDepthImage16UC1.getFocalLengthY());
         depthImageMessage.setPrincipalPointXPixels(gpuDepthImage16UC1.getPrincipalPointX());
         depthImageMessage.setPrincipalPointYPixels(gpuDepthImage16UC1.getPrincipalPointY());
         depthImageMessage.setImageWidth(gpuDepthImage16UC1.getImageWidth());
         depthImageMessage.setImageHeight(gpuDepthImage16UC1.getImageHeight());
         depthImageMessage.getPosition().set(leftCameraFramePose.getPosition());
         depthImageMessage.getOrientation().set(leftCameraFramePose.getOrientation());
         depthImageMessage.setSequenceNumber(gpuDepthImage16UC1.getSequenceNumber());
         depthImageMessage.setDepthDiscretization(gpuDepthImage16UC1.getDepthDiscretization());
         CameraModel.PINHOLE.packMessageFormat(depthImageMessage);
         ImageMessageFormat.DEPTH_PNG_16UC1.packMessageFormat(depthImageMessage);

         ros2DepthImagePublisher.publish(depthImageMessage);

         lastDepthSequenceNumber = gpuDepthImage16UC1.getSequenceNumber();

         // Close GpuMat
         depthPNGPointer.close();
         gpuDepthImage16UC1.destroy();
      }
   }

   public void publishColorImage(RawImage gpuColorImageBGR, RobotSide side)
   {
      // Perform safety checks
      if (gpuColorImageBGR != null &&
          !gpuColorImageBGR.isEmpty() &&
          gpuColorImageBGR.getSequenceNumber() != lastColorSequenceNumbers.get(side))
      {
         // Compress image
         BytePointer colorJPEGPointer = new BytePointer((long) gpuColorImageBGR.getImageHeight() * gpuColorImageBGR.getImageWidth());
         imageEncoder.encodeBGR(gpuColorImageBGR.getGpuImageMatrix().data(),
                                colorJPEGPointer,
                                gpuColorImageBGR.getImageWidth(),
                                gpuColorImageBGR.getImageHeight(),
                                gpuColorImageBGR.getGpuImageMatrix().step());

         // Publish compressed image
         ImageMessage colorImageMessage = new ImageMessage();
         ImageMessageDataPacker imageMessageDataPacker = new ImageMessageDataPacker(colorJPEGPointer.limit());
         imageMessageDataPacker.pack(colorImageMessage, colorJPEGPointer);
         MessageTools.toMessage(gpuColorImageBGR.getAcquisitionTime(), colorImageMessage.getAcquisitionTime());
         colorImageMessage.setFocalLengthXPixels(gpuColorImageBGR.getFocalLengthX());
         colorImageMessage.setFocalLengthYPixels(gpuColorImageBGR.getFocalLengthY());
         colorImageMessage.setPrincipalPointXPixels(gpuColorImageBGR.getPrincipalPointX());
         colorImageMessage.setPrincipalPointYPixels(gpuColorImageBGR.getPrincipalPointY());
         colorImageMessage.setImageWidth(gpuColorImageBGR.getImageWidth());
         colorImageMessage.setImageHeight(gpuColorImageBGR.getImageHeight());
         colorImageMessage.getPosition().set(cameraPosesInDepthFrame.get(side).getPosition());
         colorImageMessage.getOrientation().set(cameraPosesInDepthFrame.get(side).getOrientation());
         colorImageMessage.setSequenceNumber(gpuColorImageBGR.getSequenceNumber());
         colorImageMessage.setDepthDiscretization(gpuColorImageBGR.getDepthDiscretization());
         CameraModel.PINHOLE.packMessageFormat(colorImageMessage);
         ImageMessageFormat.COLOR_JPEG_BGR8.packMessageFormat(colorImageMessage);
         ros2ColorImagePublishers.get(side).publish(colorImageMessage);

         lastColorSequenceNumbers.put(side, gpuColorImageBGR.getSequenceNumber());

         // Close stuff
         colorJPEGPointer.close();
         gpuColorImageBGR.destroy();
      }
   }
}
