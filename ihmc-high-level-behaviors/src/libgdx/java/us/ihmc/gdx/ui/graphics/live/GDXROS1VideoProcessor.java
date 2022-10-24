package us.ihmc.gdx.ui.graphics.live;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;

public interface GDXROS1VideoProcessor
{
   public void prepare(int imageWidth, int imageHeight, ChannelBuffer nettyChannelBuffer);

   public void synchronizedPackPanelImage(Mat panelImageToPack, BytePointer rgba8888BytePointerToPack);
}
