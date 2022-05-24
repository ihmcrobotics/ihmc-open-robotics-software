package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.Image;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.utilities.ros.RosTools;

import java.nio.ByteBuffer;

public class GDXROS1RGBA8VideoVisualizerData
{
   private Pixmap pixmap;
   private BytePointer rgba8888BytePointer;
   private Mat rgba8Mat;
   private boolean needNewTexture = false;
   private Texture texture;

   public void updateOnMessageProcessingThread(Image image)
   {
      ChannelBuffer nettyChannelBuffer = image.getData();

      int imageWidth = image.getWidth();
      int imageHeight = image.getHeight();

      if (rgba8Mat == null || pixmap.getWidth() != imageWidth || pixmap.getHeight() != imageHeight)
      {
         if (pixmap != null)
         {
            pixmap.dispose();
         }

         pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
         rgba8888BytePointer = new BytePointer(pixmap.getPixels());
         rgba8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, rgba8888BytePointer);
         needNewTexture = true;
      }

      ByteBuffer inputByteBuffer = RosTools.sliceNettyBuffer(nettyChannelBuffer);
      for (int i = 0; i < inputByteBuffer.limit(); i++)
      {
         rgba8888BytePointer.put(i, inputByteBuffer.get());
      }
   }

   public void updateOnUIThread(ImGuiVideoPanel videoPanel)
   {
      if (rgba8Mat != null)
      {
         if (texture == null || needNewTexture)
         {
            needNewTexture = false;
            if (texture != null)
            {
               texture.dispose();
            }

            texture = new Texture(new PixmapTextureData(pixmap, null, false, false));
            videoPanel.setTexture(texture);
         }

         texture.draw(pixmap, 0, 0);
      }
   }
}
