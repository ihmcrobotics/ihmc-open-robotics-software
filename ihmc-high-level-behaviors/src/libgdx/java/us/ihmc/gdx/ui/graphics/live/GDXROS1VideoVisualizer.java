package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import imgui.internal.ImGui;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.Image;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoWindow;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class GDXROS1VideoVisualizer
{
   private AbstractRosTopicSubscriber<Image> subscriber;
   private final String topic;
   private Pixmap pixmap;
   private Texture texture;
   private ImGuiVideoWindow window;
   private volatile Image image;
   private float lowestValueSeen = -1.0f;
   private float highestValueSeen = -1.0f;

   private long receivedCount = 0;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("", 1000, 230, 20);

   private boolean enabled = false;

   public GDXROS1VideoVisualizer(String topic)
   {
      this.topic = topic;
   }

   public void subscribe(RosNodeInterface ros1Node)
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
         Image image = this.image; // store the latest one here

         if (image != null)
         {
            if (texture == null || texture.getWidth() < image.getWidth() || texture.getHeight() < image.getHeight())
            {
               if (texture != null)
               {
                  texture.dispose();
                  pixmap.dispose();
               }

               pixmap = new Pixmap(image.getWidth(), image.getHeight(), Pixmap.Format.RGBA8888);
               texture = new Texture(new PixmapTextureData(pixmap, null, false, false));

               window = new ImGuiVideoWindow(ImGuiTools.uniqueLabel(this, topic), texture, false);
            }

            if (image.getEncoding().equals("16UC1"))
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

            window.render();
         }
      }
   }
}
