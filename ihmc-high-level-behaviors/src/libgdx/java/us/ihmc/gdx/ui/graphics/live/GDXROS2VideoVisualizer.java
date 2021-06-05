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
import java.awt.image.DataBufferByte;

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
//   private Triple<ByteBuffer, Integer, Integer> decompressedImage;
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
//            decompressedImage = jpegDecompressor.decompressJPEGDataToRGBAByteBuffer(videoPacket.getData().toArray());
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

   int i = 0;

   @Override
   public void renderGraphics()
   {
      super.renderGraphics();
      if (isActive())
      {
//         Triple<ByteBuffer, Integer, Integer> image = decompressedImage; // store the latest one here
         BufferedImage image = decompressedImage; // store the latest one here

         if (image != null)
         {
//            IntBuffer intBuffer = image.getLeft().asIntBuffer();
//            intBuffer.limit(intBuffer.capacity());
//            intBuffer.rewind();
//            int width = image.getMiddle();
//            int height = image.getRight();
            int width = image.getWidth();
            int height = image.getHeight();

            if (texture == null || texture.getWidth() < width || texture.getHeight() < height)
            {
               if (texture != null)
               {
                  texture.dispose();
                  pixmap.dispose();
               }

               pixmap = new Pixmap(width, height, Pixmap.Format.RGBA8888);
               texture = new Texture(new PixmapTextureData(pixmap, null, false, false));

               window = new ImGuiVideoWindow(ImGuiTools.uniqueLabel(this, topic.getName()), texture, false);
            }

            byte[] dataBuf2 = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();

            // image is sRGB
            // pack BGR
            for (int y = 0; y < height; y++)
            {
               for (int x = 0; x < width; x++)
               {
                  int i = (y * width + x) * 3;
                  int b = dataBuf2[i + 0] & 0xFF;
                  int g = dataBuf2[i + 1] & 0xFF;
                  int r = dataBuf2[i + 2] & 0xFF;
                  int a = 255;
                  int rgb8888 = (r << 24) | (g << 16) | (b << 8) | a;
                  pixmap.drawPixel(x, y, rgb8888);
               }
            }

            texture.draw(pixmap, 0, 0);
            decompressedImage = null;
         }

         if (window != null)
         {
            window.render();
         }
      }
   }
}
