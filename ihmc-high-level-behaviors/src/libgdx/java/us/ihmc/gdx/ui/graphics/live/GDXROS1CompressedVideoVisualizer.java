package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import imgui.internal.ImGui;
import sensor_msgs.CompressedImage;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoWindow;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1Visualizer;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;

public class GDXROS1CompressedVideoVisualizer extends ImGuiGDXROS1Visualizer
{
   private AbstractRosTopicSubscriber<CompressedImage> subscriber;
   private final String topic;
   private Pixmap pixmap;
   private Texture texture;
   private ImGuiVideoWindow window;
   private volatile CompressedImage image;
   private float lowestValueSeen = -1.0f;
   private float highestValueSeen = -1.0f;

   private long receivedCount = 0;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("", 1000, 230, 20);

   public GDXROS1CompressedVideoVisualizer(String title, String topic)
   {
      super(title);
      this.topic = topic;
   }

   public void subscribe(RosNodeInterface ros1Node)
   {
      subscriber = new AbstractRosTopicSubscriber<CompressedImage>(sensor_msgs.CompressedImage._TYPE)
      {
         @Override
         public void onNewMessage(CompressedImage image)
         {
            GDXROS1CompressedVideoVisualizer.this.image = image;
            ++receivedCount;
         }
      };
      ros1Node.attachSubscriber(topic, subscriber);
   }

   @Override
   public void unsubscribe(RosNodeInterface ros1Node)
   {
      ros1Node.removeSubscriber(subscriber);
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic);
      receivedPlot.render(receivedCount);
   }

   @Override
   public void renderGraphics()
   {
      if (isActive())
      {
         CompressedImage image = this.image; // store the latest one here

         if (image != null)
         {
            boolean is16BitDepth = image.getFormat().contains("16UC1"); // TODO: Support depth image
            boolean is8BitRGB = image.getFormat().contains("rgb8");
            boolean isBGR8 = image.getFormat().contains("bgr8");

            BufferedImage bufferedImage = RosTools.bufferedImageFromRosMessageJpeg(image);
            byte[] data = ((DataBufferByte) bufferedImage.getRaster().getDataBuffer()).getData();
            int width = bufferedImage.getWidth();
            int height = bufferedImage.getHeight();

            if (texture == null || texture.getWidth() < width || texture.getHeight() < height)
            {
               if (texture != null)
               {
                  texture.dispose();
                  pixmap.dispose();
               }

               pixmap = new Pixmap(width, height, Pixmap.Format.RGBA8888);
               texture = new Texture(new PixmapTextureData(pixmap, null, false, false));

               window = new ImGuiVideoWindow(ImGuiTools.uniqueLabel(this, topic), texture, false);
            }


            if (is8BitRGB)
            {
               int zeroedIndex = 0;
               for (int y = 0; y < height; y++)
               {
                  for (int x = 0; x < width; x++)
                  {
                     int rgbaColor = data[zeroedIndex] << 24 | data[zeroedIndex + 1] << 16 | data[zeroedIndex + 2] << 8 | 255;
                     int color = bufferedImage.getRGB(x, y);
                     zeroedIndex += 3;
                     pixmap.drawPixel(x, y, (color << 8) | 255);
                  }
               }
            }
            else if (isBGR8)
            {
               for (int y = 0; y < height; y++)
               {
                  for (int x = 0; x < width; x++)
                  {
                     int i = (y * width + x) * 3;
                     int b = Byte.toUnsignedInt(data[i + 0]);
                     int g = Byte.toUnsignedInt(data[i + 1]);
                     int r = Byte.toUnsignedInt(data[i + 2]);
                     int a = 255;
                     int rgb8888 = (r << 24) | (g << 16) | (b << 8) | a;
                     pixmap.drawPixel(x, y, rgb8888);
                  }
               }
            }
            texture.draw(pixmap, 0, 0);

            window.render();
         }
      }
   }
}
