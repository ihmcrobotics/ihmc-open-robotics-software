package us.ihmc.avatar.stereoVision.net.packet;

import us.ihmc.avatar.stereoVision.net.udp.StereoVisionUDPPacketUtil;
import us.ihmc.robotics.robotSide.RobotSide;

import java.nio.ByteBuffer;

public class StereoVisionImageFragmentPacket extends StereoVisionPacket
{
   public static final int MAX_FRAGMENT_DATA_LENGTH = StereoVisionUDPPacketUtil.STEREO_VISION_MAX_PACKET_DATA_LENGTH - 128;

   private RobotSide robotSide;
   private int imageWidth;
   private int imageHeight;
   private int imageDataLength;
   private int frameNumber;
   private int fragmentNumber;
   private int fragmentDataLength;
   private byte[] data;

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public void setImageWidth(int imageWidth)
   {
      this.imageWidth = imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public void setImageHeight(int imageHeight)
   {
      this.imageHeight = imageHeight;
   }

   public int getImageDataLength()
   {
      return imageDataLength;
   }

   public void setImageDataLength(int imageDataLength)
   {
      this.imageDataLength = imageDataLength;
   }

   public int getFrameNumber()
   {
      return frameNumber;
   }

   public void setFrameNumber(int frameNumber)
   {
      this.frameNumber = frameNumber;
   }

   public int getFragmentNumber()
   {
      return fragmentNumber;
   }

   public void setFragmentNumber(int fragmentNumber)
   {
      this.fragmentNumber = fragmentNumber;
   }

   public int getFragmentDataLength()
   {
      return fragmentDataLength;
   }

   public void setFragmentDataLength(int fragmentDataLength)
   {
      this.fragmentDataLength = fragmentDataLength;
   }

   public byte[] getData()
   {
      return data;
   }

   public void setData(byte[] data)
   {
      this.data = data;
   }

   @Override
   public int serialize(ByteBuffer buffer)
   {
      int length = 0;
      buffer.put(robotSide.toByte());
      length += 1;
      buffer.putInt(imageWidth);
      length += 4;
      buffer.putInt(imageHeight);
      length += 4;
      buffer.putInt(imageDataLength);
      length += 4;
      buffer.putInt(frameNumber);
      length += 4;
      buffer.putInt(fragmentNumber);
      length += 4;
      buffer.putInt(fragmentDataLength);
      length += 4;
      for (int i = 0; i < data.length; i++)
         buffer.put(data[i]);
      length += data.length;
      return length;
   }

   @Override
   public void deserialize(ByteBuffer buffer)
   {
      robotSide = RobotSide.fromByte(buffer.get());
      imageWidth = buffer.getInt();
      imageHeight = buffer.getInt();
      imageDataLength = buffer.getInt();
      frameNumber = buffer.getInt();
      fragmentNumber = buffer.getInt();
      fragmentDataLength = buffer.getInt();
      data = new byte[fragmentDataLength];
      for (int i = 0; i < fragmentDataLength; i++)
         data[i] = buffer.get();
   }

   @Override
   public byte getPacketID()
   {
      return StereoVisionPacket.PACKET_ID_IMAGE_FRAGMENT;
   }
}
