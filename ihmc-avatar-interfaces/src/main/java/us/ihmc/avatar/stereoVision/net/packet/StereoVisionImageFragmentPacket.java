package us.ihmc.avatar.stereoVision.net.packet;

import us.ihmc.robotics.robotSide.RobotSide;

import java.nio.ByteBuffer;

public class StereoVisionImageFragmentPacket extends StereoVisionPacket
{
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
   public void serialize(ByteBuffer buffer)
   {
      buffer.put(robotSide.toByte());
      buffer.putInt(imageWidth);
      buffer.putInt(imageHeight);
      buffer.putInt(imageDataLength);
      buffer.putInt(frameNumber);
      buffer.putInt(fragmentNumber);
      buffer.putInt(fragmentDataLength);
      buffer.put(data);
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
}
