package us.ihmc.avatar.colorVision;

import org.bytedeco.javacpp.BytePointer;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.message.Time;
import sensor_msgs.Image;
import std_msgs.msg.dds.Float64;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.spinnaker.BytedecoBlackfly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.time.FrequencyCalculator;
import us.ihmc.utilities.ros.ROS1Helper;
import us.ihmc.utilities.ros.publisher.RosImagePublisher;

public class DualBlackflyCamera
{
   private String serialNumber;
   private BytedecoBlackfly blackfly;
   private BytePointer imageData;
   private RobotSide side;
   private ROS1Helper ros1Helper;
   private String ros1Topic;
   private ROS2Helper ros2Helper;
   private RosImagePublisher rosImagePublisher;
   private int numberOfBytesInFrame;
   private int imageWidth;
   private int imageHeight;
   private final FrequencyCalculator ros1ImagePublishRateCalculator = new FrequencyCalculator();

   public DualBlackflyCamera(String serialNumber)
   {
      this.serialNumber = serialNumber;
   }

   public void create(BytedecoBlackfly blackfly, RobotSide side, ROS1Helper ros1Helper, String ros1Topic, ROS2Helper ros2Helper)
   {
      this.blackfly = blackfly;
      this.side = side;
      this.ros1Helper = ros1Helper;
      this.ros1Topic = ros1Topic;
      this.ros2Helper = ros2Helper;
      blackfly.initialize();
   }

   public void update()
   {
      if (blackfly.readFrameData())
      {
         double dataAquisitionTime = Conversions.nanosecondsToSeconds(System.nanoTime());

         if (rosImagePublisher == null)
         {
            imageWidth = blackfly.getWidth();
            imageHeight = blackfly.getHeight();
            LogTools.info("Blackfly {} resolution detected: {}x{}", serialNumber, imageWidth, imageHeight);
            numberOfBytesInFrame = imageWidth * imageHeight * 4;
            imageData = new BytePointer((long) numberOfBytesInFrame);

            LogTools.info("Publishing ROS 1 color: {}", ros1Topic);
            rosImagePublisher = new RosImagePublisher();
            ros1Helper.attachPublisher(ros1Topic, rosImagePublisher);
         }

         blackfly.getImageData(imageData);

         if (rosImagePublisher.isConnected())
         {
            ChannelBuffer channelBuffer = ChannelBuffers.wrappedBuffer(imageData.asByteBuffer());

            int bytesPerValue = 4;
            Image message = rosImagePublisher.createMessage(imageWidth, imageHeight, bytesPerValue, "8UC4", channelBuffer);
            message.getHeader().setStamp(new Time(dataAquisitionTime));

            rosImagePublisher.publish(message);
         }
         ros1ImagePublishRateCalculator.ping();
         Float64 float64Message = new Float64();
         float64Message.setData(ros1ImagePublishRateCalculator.getFrequency());
         ros2Helper.publish(DualBlackflyComms.PUBLISH_RATE.get(side), float64Message);
      }
   }

   public void destroy()
   {
      blackfly.destroy();
   }

   public String getSerialNumber()
   {
      return serialNumber;
   }

   public BytedecoBlackfly getBytedecoBlackfly()
   {
      return blackfly;
   }

   public BytePointer getImageData()
   {
      return imageData;
   }
}
