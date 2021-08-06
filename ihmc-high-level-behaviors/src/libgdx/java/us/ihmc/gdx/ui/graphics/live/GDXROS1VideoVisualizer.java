package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import imgui.internal.ImGui;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1Visualizer;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;

public class GDXROS1VideoVisualizer extends ImGuiGDXROS1Visualizer
{
   private final boolean isCompressed;
   private AbstractRosTopicSubscriber<Image> subscriber;
   private AbstractRosTopicSubscriber<CompressedImage> compressedSubscriber;
   private final String topic;
   private Pixmap pixmap;
   private Texture texture;
   private final ImGuiVideoPanel videoPanel;
   private volatile Image image;
   private volatile CompressedImage compressedImage;
   private float lowestValueSeen = -1.0f;
   private float highestValueSeen = -1.0f;

   private long receivedCount = 0;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("", 1000, 230, 20);

   public GDXROS1VideoVisualizer(String title, String topic)
   {
      super(title);
      this.topic = topic;
      isCompressed = topic.endsWith("compressed");
      videoPanel = new ImGuiVideoPanel(ImGuiTools.uniqueLabel(this, topic), false);
   }

   @Override
   public void subscribe(RosNodeInterface ros1Node)
   {
      if (isCompressed)
      {
         compressedSubscriber = new AbstractRosTopicSubscriber<CompressedImage>(sensor_msgs.CompressedImage._TYPE)
         {
            @Override
            public void onNewMessage(CompressedImage image)
            {
               GDXROS1VideoVisualizer.this.compressedImage = image;
               ++receivedCount;
            }
         };
         ros1Node.attachSubscriber(topic, compressedSubscriber);
      }
      else
      {
         subscriber = new AbstractRosTopicSubscriber<Image>(sensor_msgs.Image._TYPE)
         {
            @Override
            public void onNewMessage(Image image)
            {
               GDXROS1VideoVisualizer.this.image = image;
               ++receivedCount;
            }
         };
         ros1Node.attachSubscriber(topic, subscriber);
      }
   }

   @Override
   public void unsubscribe(RosNodeInterface ros1Node)
   {
      if (isCompressed)
      {
         ros1Node.removeSubscriber(compressedSubscriber);
      }
      else
      {
         ros1Node.removeSubscriber(subscriber);
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic);
      receivedPlot.render(receivedCount);
   }

   @Override
   public void update()
   {
      super.update();
      if (isActive())
      {
         // store the latest ones here
         Image image = this.image;
         CompressedImage compressedImage = this.compressedImage;

         if (isCompressed && compressedImage != null)
         {
            boolean is16BitDepth = compressedImage.getFormat().contains("16UC1"); // TODO: Support depth image
            boolean is8BitRGB = compressedImage.getFormat().contains("rgb8");
            boolean isBGR8 = compressedImage.getFormat().contains("bgr8");

            BufferedImage bufferedImage = RosTools.bufferedImageFromRosMessageJpeg(compressedImage);
            byte[] data = ((DataBufferByte) bufferedImage.getRaster().getDataBuffer()).getData();
            int width = bufferedImage.getWidth();
            int height = bufferedImage.getHeight();

            ensureTextureReady(width, height);

            if (is8BitRGB)
            {
               int zeroedIndex = 0;
               for (int y = 0; y < height; y++)
               {
                  for (int x = 0; x < width; x++)
                  {
//                     int rgbaColor = data[zeroedIndex] << 24 | data[zeroedIndex + 1] << 16 | data[zeroedIndex + 2] << 8 | 255;
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
         }
         else if (image != null)
         {
            ensureTextureReady(image.getWidth(), image.getHeight());

            boolean is16BitDepth = image.getEncoding().equals("16UC1");
            boolean is8BitRGB = image.getEncoding().equals("rgb8");
            if (is8BitRGB)
            {
               ChannelBuffer data = image.getData();
               int zeroedIndex = 0;
               for (int y = 0; y < image.getHeight(); y++)
               {
                  for (int x = 0; x < image.getWidth(); x++)
                  {
                     int r = Byte.toUnsignedInt(data.getByte(zeroedIndex + 0));
                     int g = Byte.toUnsignedInt(data.getByte(zeroedIndex + 1));
                     int b = Byte.toUnsignedInt(data.getByte(zeroedIndex + 2));
                     int a = 255;
                     zeroedIndex += 3;
                     int rgb8888 = (r << 24) | (g << 16) | (b << 8) | a;
                     pixmap.drawPixel(x, y, rgb8888);
                  }
               }
               texture.draw(pixmap, 0, 0);
            }
            else if (is16BitDepth)
            {
               ChannelBuffer data = image.getData();
               byte[] array = data.array();
               int dataIndex = data.arrayOffset();
               for (int y = 0; y < image.getHeight(); y++)
               {
                  for (int x = 0; x < image.getWidth(); x++)
                  {
                     int bigByte = array[dataIndex];
                     dataIndex++;
                     int smallByte = array[dataIndex];
                     dataIndex++;

                     float value = (float) (bigByte & 0xFF | smallByte << 8);

                     if (highestValueSeen < 0 || value > highestValueSeen)
                        highestValueSeen = value;
                     if (lowestValueSeen < 0 || value < lowestValueSeen)
                        lowestValueSeen = value;

                     float colorRange = highestValueSeen - lowestValueSeen;
                     float grayscale = (value - lowestValueSeen) / colorRange;

                     pixmap.drawPixel(x, y, Color.rgba8888(grayscale, grayscale, grayscale, 1.0f));
                  }
               }
               texture.draw(pixmap, 0, 0);
            }
         }
      }
   }

   private void ensureTextureReady(int width, int height)
   {
      if (texture == null || texture.getWidth() < width || texture.getHeight() < height)
      {
         if (texture != null)
         {
            texture.dispose();
            pixmap.dispose();
         }

         pixmap = new Pixmap(width, height, Pixmap.Format.RGBA8888);
         texture = new Texture(new PixmapTextureData(pixmap, null, false, false));

         videoPanel.setTexture(texture);
      }
   }

   @Override
   public ImGuiVideoPanel getPanel()
   {
      return videoPanel;
   }
}
