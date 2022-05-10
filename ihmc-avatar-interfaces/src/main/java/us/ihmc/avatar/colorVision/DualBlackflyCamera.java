package us.ihmc.avatar.colorVision;

import org.bytedeco.javacpp.BytePointer;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.Time;
import sensor_msgs.Image;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.perception.spinnaker.BytedecoBlackfly;
import us.ihmc.utilities.ros.ROS1Helper;
import us.ihmc.utilities.ros.publisher.RosImagePublisher;

public class DualBlackflyCamera
{
   private String serialNumber;
   private BytedecoBlackfly blackfly;
   private BytePointer imageData;
   private String ros1Topic;
   private RosImagePublisher rosImagePublisher;
   private ChannelBuffer ros1ChannelBuffer;
   private int numberOfBytesInFrame;
   private int imageWidth;
   private int imageHeight;

   public DualBlackflyCamera(String serialNumber)
   {
      this.serialNumber = serialNumber;
   }

   public void create(BytedecoBlackfly blackfly, ROS1Helper ros1Helper, String ros1Topic)
   {
      this.blackfly = blackfly;
      this.ros1Topic = ros1Topic;
      blackfly.initialize();

      imageWidth = blackfly.getWidth();
      imageHeight = blackfly.getHeight();
      numberOfBytesInFrame = imageWidth * imageHeight * 4;
      imageData = new BytePointer((long) numberOfBytesInFrame);

      LogTools.info("Publishing ROS 1 color: {}", ros1Topic);
      rosImagePublisher = new RosImagePublisher();
      ros1Helper.attachPublisher(ros1Topic, rosImagePublisher);
      ros1ChannelBuffer = rosImagePublisher.getChannelBufferFactory().getBuffer(numberOfBytesInFrame);
   }

   public void update()
   {
      blackfly.getImageData(imageData);

      double dataAquisitionTime = Conversions.nanosecondsToSeconds(System.nanoTime());

      if (rosImagePublisher.isConnected())
      {
         ros1ChannelBuffer.clear();

         for (int i = 0; i < numberOfBytesInFrame; i++)
         {
            ros1ChannelBuffer.writeByte(imageData.get(i));
         }

         ros1ChannelBuffer.readerIndex(0);
         ros1ChannelBuffer.writerIndex(numberOfBytesInFrame);

         int bytesPerValue = 4;
         Image message = rosImagePublisher.createMessage(imageWidth, imageHeight, bytesPerValue, "8UC4", ros1ChannelBuffer);
         message.getHeader().setStamp(new Time(dataAquisitionTime));

         rosImagePublisher.publish(message);
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
