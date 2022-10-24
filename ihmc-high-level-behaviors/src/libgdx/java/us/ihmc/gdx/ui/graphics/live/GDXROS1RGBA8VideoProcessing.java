package us.ihmc.gdx.ui.graphics.live;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import us.ihmc.utilities.ros.RosTools;

import java.nio.ByteBuffer;

public class GDXROS1RGBA8VideoProcessing implements GDXROS1VideoProcessor
{
   private ChannelBuffer nettyChannelBuffer;

   @Override
   public void prepare(int imageWidth, int imageHeight, ChannelBuffer nettyChannelBuffer)
   {
      this.nettyChannelBuffer = nettyChannelBuffer;
   }

   @Override
   public void synchronizedPackPanelImage(Mat panelImageToPack, BytePointer rgba8888BytePointerToPack)
   {
      ByteBuffer inputByteBuffer = RosTools.sliceNettyBuffer(nettyChannelBuffer);
      BytePointer rgba8888BytePointer = rgba8888BytePointerToPack;
      for (int i = 0; i < inputByteBuffer.limit(); i++)
      {
         rgba8888BytePointer.put(i, inputByteBuffer.get());
      }
   }
}
