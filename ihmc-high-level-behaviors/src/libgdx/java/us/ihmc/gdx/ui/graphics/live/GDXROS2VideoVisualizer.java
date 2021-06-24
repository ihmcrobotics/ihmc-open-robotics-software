package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import controller_msgs.msg.dds.DetectedFiducialPacket;
import controller_msgs.msg.dds.VideoPacket;
import imgui.internal.ImGui;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.idl.IDLSequence;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.image.WritableRaster;
import java.nio.ByteBuffer;

public class GDXROS2VideoVisualizer extends ImGuiGDXVisualizer
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<VideoPacket> topic;
   private final ResettableExceptionHandlingExecutorService threadQueue;
   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private Pixmap pixmap;
   private Texture texture;
   private final ImGuiVideoPanel videoPanel;

   private long receivedCount = 0;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("", 1000, 230, 20);
//   private Triple<ByteBuffer, Integer, Integer> decompressedImage;
   private BufferedImage decompressedImage;
   private DetectedFiducialPacket latestDetectedFiducial;
//   private BufferedImage bufferedImage;

   public GDXROS2VideoVisualizer(String title, ROS2Node ros2Node, ROS2Topic<VideoPacket> topic)
   {
      super(title);
      this.ros2Node = ros2Node;
      this.topic = topic;
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
      new IHMCROS2Callback<>(ros2Node, topic, ROS2QosProfile.BEST_EFFORT(), this::acceptMessage);
      videoPanel = new ImGuiVideoPanel(ImGuiTools.uniqueLabel(this, topic.getName()), false);
   }

   public void addFiducialSubscription(ROS2Topic<DetectedFiducialPacket> fiducialTopic)
   {
      new IHMCROS2Callback<>(ros2Node, fiducialTopic, detectedFiducialPacket -> latestDetectedFiducial = detectedFiducialPacket);
   }

   private void acceptMessage(VideoPacket videoPacket)
   {
      ++receivedCount;
      if (isActive())
      {
         threadQueue.clearQueueAndExecute(() ->
         {
//            decompressedImage = jpegDecompressor.decompressJPEGDataToBGR8ByteBuffer(videoPacket.getData().toArray());
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
   public void update()
   {
      super.update();
      if (isActive())
      {
//         Triple<ByteBuffer, Integer, Integer> image = decompressedImage; // store the latest one here
         BufferedImage image = decompressedImage; // store the latest one here

         if (image != null)
         {
//            ByteBuffer buffer = image.getLeft();
//            int width = image.getMiddle();
//            int height = image.getRight();
            WritableRaster raster = image.getRaster();
            byte[] buffer = ((DataBufferByte) raster.getDataBuffer()).getData();
            int width = image.getWidth();
            int height = image.getHeight();

            DetectedFiducialPacket currentFiducial = this.latestDetectedFiducial;
            if (currentFiducial != null)
            {
//               if (bufferedImage == null)
//               {
//                  bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
//               }
//               WritableRaster raster = image.getRaster();
//               DataBufferByte dataBuffer = (DataBufferByte) raster.getDataBuffer();
//               raster.setDataElements(0, 0, width, height, buffer);
//               System.arraycopy(buffer.array(), 0, dataBuffer.getData(), 0, width * height);

               Graphics2D graphics = image.createGraphics();
               graphics.setColor(Color.RED);
               graphics.setStroke(new BasicStroke(5.0f));
               graphics.setFont(new Font("TimesRoman", Font.PLAIN, 20));

               int nPoints = currentFiducial.getBounds().size();
               int[] xPoints = new int[nPoints];
               int[] yPoints = new int[nPoints];
               for (int j = 0; j < currentFiducial.getBounds().size(); j++)
               {
                  xPoints[j] = (int) Math.round(currentFiducial.getBounds().get(j).getX());
                  yPoints[j] = (int) Math.round(currentFiducial.getBounds().get(j).getY());

                  if (j == 0)
                  {
                     graphics.drawString("" + currentFiducial.getFiducialId(), xPoints[j], yPoints[j]);
                  }
               }

               graphics.drawPolygon(xPoints, yPoints, nPoints);

//               raster.getDataElements(0, 0, width, height, buffer);
//               System.arraycopy(dataBuffer.getData(), 0, buffer.array(), 0, width * height);
            }


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

            // unpack BGR
            for (int y = 0; y < height; y++)
            {
               for (int x = 0; x < width; x++)
               {
                  int i = (y * width + x) * 3;
//                  int b = Byte.toUnsignedInt(buffer.get(i + 0));
//                  int g = Byte.toUnsignedInt(buffer.get(i + 1));
//                  int r = Byte.toUnsignedInt(buffer.get(i + 2));
                  int b = Byte.toUnsignedInt(buffer[i + 0]);
                  int g = Byte.toUnsignedInt(buffer[i + 1]);
                  int r = Byte.toUnsignedInt(buffer[i + 2]);
                  int a = 255;
                  int rgb8888 = (r << 24) | (g << 16) | (b << 8) | a;
                  pixmap.drawPixel(x, y, rgb8888);
               }
            }

            texture.draw(pixmap, 0, 0);
            decompressedImage = null;
         }
      }
   }

   @Override
   public ImGuiVideoPanel getPanel()
   {
      return videoPanel;
   }
}
