package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import imgui.internal.ImGui;
import jxl.format.RGB;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import us.ihmc.codecs.generated.RGBPicture;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.util.ByteBufferProvider;
import us.ihmc.codecs.yuv.JPEGDecoder;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoWindow;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.nio.ByteBuffer;

public class GDXROS1CompressedVideoVisualizer
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

   private boolean enabled = false;

   private final ByteBufferProvider byteBufferProvider = new ByteBufferProvider();
   private final JPEGDecoder jpegDecoder = new JPEGDecoder();
   private YUVPicture yuvPicture;
   private RGBPicture rgbPicture;

   public GDXROS1CompressedVideoVisualizer(String topic)
   {
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

   public void unsubscribe(RosNodeInterface ros1Node)
   {
      ros1Node.removeSubscriber(subscriber);
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public void renderWidgets()
   {
      ImGui.text(topic);
      receivedPlot.render(receivedCount);
   }

   public void renderVideo()
   {
      if (enabled)
      {
         CompressedImage image = this.image; // store the latest one here

         if (image != null)
         {
            ByteBuffer byteBuffer = byteBufferProvider.getOrCreateBuffer(image.getData().array().length);
            byteBuffer.put(image.getData().array());
            byteBuffer.flip();
            yuvPicture = jpegDecoder.decode(byteBuffer);
            rgbPicture = yuvPicture.toRGB();

            ByteBuffer dstBuffer = byteBufferProvider.getOrCreateBuffer(rgbPicture.getWidth() * rgbPicture.getHeight() * 3);
            dstBuffer.put(10, (byte) 31);
            rgbPicture.get(dstBuffer);


            if (texture == null || texture.getWidth() < rgbPicture.getWidth() || texture.getHeight() < rgbPicture.getHeight())
            {
               if (texture != null)
               {
                  texture.dispose();
                  pixmap.dispose();
               }

               pixmap = new Pixmap(rgbPicture.getWidth(), rgbPicture.getHeight(), Pixmap.Format.RGBA8888);
               texture = new Texture(new PixmapTextureData(pixmap, null, false, false));

               window = new ImGuiVideoWindow(ImGuiTools.uniqueLabel(this, topic), texture, false);
            }

            boolean is16BitDepth = image.getFormat().contains("16UC1");
            boolean is8BitRGB = image.getFormat().contains("rgb8");
            if (is16BitDepth || is8BitRGB)
            {
               byte[] array = dstBuffer.array();
               int dataIndex = dstBuffer.arrayOffset();
               int zeroedIndex = 0;
               for (int y = 0; y < rgbPicture.getHeight(); y++)
               {
                  for (int x = 0; x < rgbPicture.getWidth(); x++)
                  {
                     if (is16BitDepth)
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
                     else
                     {
                        int unsignedMedium = dstBuffer.getInt(zeroedIndex);
                        zeroedIndex += 3;
                        pixmap.drawPixel(x, y, (unsignedMedium) | 255);
                     }
                  }
               }
               texture.draw(pixmap, 0, 0);
            }

            window.render();
         }
      }
   }
}
