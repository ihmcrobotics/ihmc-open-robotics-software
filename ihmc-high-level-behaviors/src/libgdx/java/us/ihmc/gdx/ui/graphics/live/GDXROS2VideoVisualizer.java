package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import controller_msgs.msg.dds.VideoPacket;
import imgui.internal.ImGui;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoWindow;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.awt.image.BufferedImage;

public class GDXROS2VideoVisualizer extends ImGuiGDXVisualizer
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<VideoPacket> topic;
   private final ResettableExceptionHandlingExecutorService threadQueue;
   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private Pixmap pixmap;
   private Texture texture;
   private ImGuiVideoWindow window;

   private long receivedCount = 0;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("", 1000, 230, 20);
   private BufferedImage decompressedImage;

   public GDXROS2VideoVisualizer(String title, ROS2Node ros2Node, ROS2Topic<VideoPacket> topic)
   {
      super(title);
      this.ros2Node = ros2Node;
      this.topic = topic;
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
      new IHMCROS2Callback<>(ros2Node, topic, ROS2QosProfile.BEST_EFFORT(), this::acceptMessage);
   }

   private void acceptMessage(VideoPacket videoPacket)
   {
      ++receivedCount;
      if (isActive())
      {
         threadQueue.clearQueueAndExecute(() ->
         {
            decompressedImage = jpegDecompressor.decompressJPEGDataToBufferedImage(videoPacket.getData().toArray());
         });
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
      receivedPlot.render(receivedCount);
   }

   @Override
   public void renderGraphics()
   {
      super.renderGraphics();
      if (isActive())
      {
         BufferedImage image = decompressedImage; // store the latest one here

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

               window = new ImGuiVideoWindow(ImGuiTools.uniqueLabel(this, topic.getName()), texture, false);
            }

            boolean flipX = false;
            boolean flipY = true;
            for (int x = 0; x < image.getWidth(); x++)
            {
               for (int y = 0; y < image.getHeight(); y++)
               {
                  int placeX = flipX ? image.getWidth()  - 1 - x : x;
                  int placeY = flipY ? image.getHeight() - 1 - y : y;
                  int rgb = image.getRGB(x, y);

                  pixmap.drawPixel(placeX, placeY, rgb);
               }
            }

            texture.draw(pixmap, 0, 0);
            window.render();
         }
      }
   }
}
