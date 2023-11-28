package us.ihmc.avatar.colorVision;

import us.ihmc.robotics.robotSide.RobotSide;

import javax.annotation.Nullable;
import java.nio.ByteBuffer;

public class DualBlackflyFragmentedUDPImage
{
   private ByteBuffer buffer;
   @Nullable
   private byte[] completeImageData;

   private RobotSide robotSide;
   private int width;
   private int height;
   private int frameNumber;
   private int fragment;
   private boolean complete;

   public DualBlackflyFragmentedUDPImage(int imageBytes)
   {
      buffer = ByteBuffer.allocate(imageBytes);
   }

   private DualBlackflyFragmentedUDPImage()
   {

   }

   public void setRobotSide(RobotSide robotSide)
   {
      if (complete)
      {
         throw new RuntimeException("complete() has been called");
      }

      this.robotSide = robotSide;
   }

   public void setWidth(int width)
   {
      if (complete)
      {
         throw new RuntimeException("complete() has been called");
      }

      this.width = width;
   }

   public int getWidth()
   {
      return width;
   }

   public void setHeight(int height)
   {
      if (complete)
      {
         throw new RuntimeException("complete() has been called");
      }

      this.height = height;
   }

   public void setFrameNumber(int frameNumber)
   {
      if (complete)
      {
         throw new RuntimeException("complete() has been called");
      }

      this.frameNumber = frameNumber;
   }

   public int getFrameNumber()
   {
      return frameNumber;
   }

   public int getHeight()
   {
      return height;
   }

   public int getFragment()
   {
      return fragment;
   }

   public void nextFragment(int fragment, int fragmentDataLength, byte[] fragmentData)
   {
      if (complete)
      {
         throw new RuntimeException("complete() has been called");
      }

      for (int i = 0; i < fragmentDataLength; i++)
      {
         buffer.put(fragmentData[i]);
      }

      this.fragment = fragment;
   }

   public void complete()
   {
      complete = true;
      completeImageData = new byte[buffer.limit()];
      for (int i = 0; i < buffer.limit(); i++)
      {
         completeImageData[i] = buffer.get(i);
      }
   }

   public boolean isComplete()
   {
      return complete;
   }

   public void reset()
   {
      complete = false;
      buffer.rewind();
   }

   @Nullable
   public byte[] getCompleteImageData()
   {
      return completeImageData;
   }

   public DualBlackflyFragmentedUDPImage clone()
   {
      if (!complete)
      {
         throw new RuntimeException("complete() has not been called");
      }
      DualBlackflyFragmentedUDPImage dualBlackflyFragmentedUDPImage = new DualBlackflyFragmentedUDPImage();
      dualBlackflyFragmentedUDPImage.completeImageData = completeImageData;
      dualBlackflyFragmentedUDPImage.width = width;
      dualBlackflyFragmentedUDPImage.height = height;
      dualBlackflyFragmentedUDPImage.frameNumber = frameNumber;
      dualBlackflyFragmentedUDPImage.fragment = fragment;
      dualBlackflyFragmentedUDPImage.complete = complete;
      return dualBlackflyFragmentedUDPImage;
   }
}
