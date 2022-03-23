package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import controller_msgs.msg.dds.DetectedFiducialPacket;
import controller_msgs.msg.dds.VideoPacket;
import imgui.internal.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class GDXROS2VideoVisualizer extends ImGuiGDXVisualizer
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<VideoPacket> topic;
   private final ResettableExceptionHandlingExecutorService threadQueue;
   private Pixmap pixmap;
   private Texture texture;
   private final ImGuiVideoPanel videoPanel;

   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private DetectedFiducialPacket latestDetectedFiducial;
   private final Mat inputJPEGYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);
   private final Mat bgr8Mat = new Mat(100); // allocate any amount of data, it'll be resized later
   private Mat rgba8Mat;
   private boolean needNewTexture = false;
   private BytePointer rgba8888BytePointer;

   public GDXROS2VideoVisualizer(String title, ROS2Node ros2Node, ROS2Topic<VideoPacket> topic)
   {
      super(title);
      this.ros2Node = ros2Node;
      this.topic = topic;
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
      new IHMCROS2Callback<>(ros2Node, topic, ROS2QosProfile.BEST_EFFORT(), this::acceptMessage);
      boolean flipY = true;
      videoPanel = new ImGuiVideoPanel(ImGuiTools.uniqueLabel(this, topic.getName()), flipY);
   }

   public void addFiducialSubscription(ROS2Topic<DetectedFiducialPacket> fiducialTopic)
   {
      new IHMCROS2Callback<>(ros2Node, fiducialTopic, detectedFiducialPacket -> latestDetectedFiducial = detectedFiducialPacket);
   }

   private void acceptMessage(VideoPacket videoPacket)
   {
      frequencyPlot.recordEvent();
      if (isActive())
      {
         threadQueue.clearQueueAndExecute(() ->
         {
            BytePointer jpegDataBytePointer = new BytePointer(videoPacket.getData().toArray());
            inputJPEGYUVI420Mat.cols(videoPacket.getData().size());
            inputJPEGYUVI420Mat.data(jpegDataBytePointer);

            // Converts image to 3 channel BGR color image.
            // This should handle JPEG encoded YUV I420 and output BGR
            opencv_imgcodecs.imdecode(inputJPEGYUVI420Mat, opencv_imgcodecs.IMREAD_COLOR, bgr8Mat);

            synchronized (this)
            {
               int imageWidth = bgr8Mat.cols();
               int imageHeight = bgr8Mat.rows();
               if (rgba8Mat == null || pixmap.getWidth() < imageWidth || pixmap.getHeight() < imageHeight)
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

               opencv_imgproc.cvtColor(bgr8Mat, rgba8Mat, opencv_imgproc.COLOR_BGR2BGRA);
            }
         });
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
      frequencyPlot.renderImGuiWidgets();
   }

   int i = 0;

   @Override
   public void update()
   {
      super.update();
      if (isActive())
      {

         synchronized (this)
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

////         Triple<ByteBuffer, Integer, Integer> image = decompressedImage; // store the latest one here
//         BufferedImage image = decompressedImage; // store the latest one here
//
//         if (image != null)
//         {
////            ByteBuffer buffer = image.getLeft();
////            int width = image.getMiddle();
////            int height = image.getRight();
//            WritableRaster raster = image.getRaster();
//            byte[] buffer = ((DataBufferByte) raster.getDataBuffer()).getData();
//            int width = image.getWidth();
//            int height = image.getHeight();
//
//            DetectedFiducialPacket currentFiducial = this.latestDetectedFiducial;
//            if (currentFiducial != null)
//            {
////               if (bufferedImage == null)
////               {
////                  bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
////               }
////               WritableRaster raster = image.getRaster();
////               DataBufferByte dataBuffer = (DataBufferByte) raster.getDataBuffer();
////               raster.setDataElements(0, 0, width, height, buffer);
////               System.arraycopy(buffer.array(), 0, dataBuffer.getData(), 0, width * height);
//
//               Graphics2D graphics = image.createGraphics();
//               graphics.setColor(Color.RED);
//               graphics.setStroke(new BasicStroke(5.0f));
//               graphics.setFont(new Font("TimesRoman", Font.PLAIN, 20));
//
//               int nPoints = currentFiducial.getBounds().size();
//               int[] xPoints = new int[nPoints];
//               int[] yPoints = new int[nPoints];
//               for (int j = 0; j < currentFiducial.getBounds().size(); j++)
//               {
//                  xPoints[j] = (int) Math.round(currentFiducial.getBounds().get(j).getX());
//                  yPoints[j] = (int) Math.round(currentFiducial.getBounds().get(j).getY());
//
//                  if (j == 0)
//                  {
//                     graphics.drawString("" + currentFiducial.getFiducialId(), xPoints[j], yPoints[j]);
//                  }
//               }
//
//               graphics.drawPolygon(xPoints, yPoints, nPoints);
//
////               raster.getDataElements(0, 0, width, height, buffer);
////               System.arraycopy(dataBuffer.getData(), 0, buffer.array(), 0, width * height);
//            }
//
//
//            if (texture == null || texture.getWidth() < width || texture.getHeight() < height)
//            {
//               if (texture != null)
//               {
//                  texture.dispose();
//                  pixmap.dispose();
//               }
//
//               pixmap = new Pixmap(width, height, Pixmap.Format.RGBA8888);
//               texture = new Texture(new PixmapTextureData(pixmap, null, false, false));
//
//               videoPanel.setTexture(texture);
//            }
//
//            // unpack BGR
//            for (int y = 0; y < height; y++)
//            {
//               for (int x = 0; x < width; x++)
//               {
//                  int i = (y * width + x) * 3;
////                  int b = Byte.toUnsignedInt(buffer.get(i + 0));
////                  int g = Byte.toUnsignedInt(buffer.get(i + 1));
////                  int r = Byte.toUnsignedInt(buffer.get(i + 2));
//                  int b = Byte.toUnsignedInt(buffer[i + 0]);
//                  int g = Byte.toUnsignedInt(buffer[i + 1]);
//                  int r = Byte.toUnsignedInt(buffer[i + 2]);
//                  int a = 255;
//                  int rgb8888 = (r << 24) | (g << 16) | (b << 8) | a;
//                  pixmap.drawPixel(x, y, rgb8888);
//               }
//            }
//
//            texture.draw(pixmap, 0, 0);
//            decompressedImage = null;
//
//
//
//         }
      }
   }

   @Override
   public ImGuiVideoPanel getPanel()
   {
      return videoPanel;
   }
}
