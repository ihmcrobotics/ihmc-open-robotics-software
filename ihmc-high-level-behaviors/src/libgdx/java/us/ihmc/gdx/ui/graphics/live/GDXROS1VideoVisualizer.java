package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import imgui.internal.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1Visualizer;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.ROSOpenCVImage;
import us.ihmc.perception.ROSOpenCVTools;
import us.ihmc.perception.ImageEncodingTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.nio.Buffer;
import java.nio.ByteBuffer;

public class GDXROS1VideoVisualizer extends ImGuiGDXROS1Visualizer
{
   private final boolean isCompressed;
   private AbstractRosTopicSubscriber<Image> subscriber;
   private AbstractRosTopicSubscriber<CompressedImage> compressedSubscriber;
   private final ResettableExceptionHandlingExecutorService threadQueue;
   private final String topic;
   private Pixmap pixmap;
   private Texture texture;
   private Mat rgba8Mat;
   private Mat input8UC1Mat;
   private BytePointer rgba8888BytePointer;
   private final ImGuiVideoPanel videoPanel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private Mat input16UC1Mat;
   private Mat decodedImageMat;
   private Mat resizedImageMat;
   private boolean needNewTexture = false;

   public GDXROS1VideoVisualizer(String title, String topic)
   {
      super(title);
      this.topic = topic;
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
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
               frequencyPlot.recordEvent();
               if (isActive())
               {
//                  threadQueue.clearQueueAndExecute(() -> processIncomingMessageOnThread(null, image));
               }
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
               frequencyPlot.recordEvent();
               if (isActive())
               {
                  threadQueue.clearQueueAndExecute(() -> processIncomingMessageOnThread(image, null));
               }
            }
         };
         ros1Node.attachSubscriber(topic, subscriber);
      }
   }

   private void processIncomingMessageOnThread(Image image, CompressedImage compressedImage)
   {
      ChannelBuffer nettyChannelBuffer = image == null ? compressedImage.getData() : image.getData();

      int imageWidth = image.getWidth();
      int imageHeight = image.getHeight();
      String encoding = image.getEncoding();
      if (input16UC1Mat == null)
      {
         int cvType = ImageEncodingTools.getCvType(encoding);
         input16UC1Mat = new Mat(imageHeight, imageWidth, cvType);
         input8UC1Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC1);
//         decodedImageMat = new Mat(image.getHeight(), image.getWidth(), opencv_core.CV_8UC4);
//         resizedImageMat = new Mat(image.getHeight(), image.getWidth(), opencv_core.CV_8UC4);
      }

      ROSOpenCVTools.backMatWithNettyBuffer(input16UC1Mat, nettyChannelBuffer);
      BytedecoOpenCVTools.clampTo8BitUnsignedChar(input16UC1Mat, input8UC1Mat, 0.0, 255.0);


//      if (isCompressed && compressedImage != null)
//      {
//         decompressAndDecodeUsingOpenCV(compressedImage);
//         texture.draw(pixmap, 0, 0);
//      }
//      else if (image != null)
//      {
//         ensureTextureReady(image.getWidth(), image.getHeight());
//         decodeUsingOpenCV(image);
//         texture.draw(pixmap, 0, 0);
//      }

      synchronized (this)
      {
//         int imageWidth;
//         int imageHeight;
//         if (format == ROS2VideoFormat.JPEGYUVI420)
//         {
//            imageWidth = bgr8Mat.cols();
//            imageHeight = bgr8Mat.rows();
//         }
//         else
//         {
//            imageWidth = input16UC1Mat.cols();
//            imageHeight = input16UC1Mat.rows();
//         }

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

         opencv_imgproc.cvtColor(input8UC1Mat, rgba8Mat, opencv_imgproc.COLOR_GRAY2RGBA);
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
      frequencyPlot.renderImGuiWidgets();
      ImGui.sameLine();
   }

   private void decodeUsingOpenCV(Image image)
   {
      String encoding = image.getEncoding();
      if (input16UC1Mat == null)
      {
         int cvType = ImageEncodingTools.getCvType(encoding);
         input16UC1Mat = new Mat(image.getHeight(), image.getWidth(), cvType);
         decodedImageMat = new Mat(image.getHeight(), image.getWidth(), opencv_core.CV_8UC4);
         resizedImageMat = new Mat(image.getHeight(), image.getWidth(), opencv_core.CV_8UC4);
      }

      ChannelBuffer nettyChannelBuffer = image.getData();
      ROSOpenCVTools.backMatWithNettyBuffer(input16UC1Mat, nettyChannelBuffer);

//      BytedecoOpenCVTools.clampTo8BitUnsignedChar(inputImageMat, input8UC1Mat, 0.0, 255.0);

      if (ImageEncodingTools.isMono(encoding))
      {
         double min = 0.0;
         double max = ImageEncodingTools.getMaxBitValue(encoding);
         int depthType = -1; // output same type as input
         opencv_core.normalize(input16UC1Mat, input16UC1Mat, min, max, opencv_core.NORM_MINMAX, depthType, opencv_core.noArray());
      }

      int conversionCode = ImageEncodingTools.getColorConversionCode(encoding, ImageEncodingTools.RGBA8).get(0);
      opencv_imgproc.cvtColor(input16UC1Mat, decodedImageMat, conversionCode);

      int sourceDepth = ImageEncodingTools.bitDepth(encoding);
      if (sourceDepth == 16)
      {
         double alpha = 255.0 / 65535.0;
         double beta = 0.0;
         decodedImageMat.convertTo(resizedImageMat, opencv_core.CV_8UC4, alpha, beta);
      }
      else
         decodedImageMat.convertTo(resizedImageMat, opencv_core.CV_8UC4); // TODO: Necessary?

      pixmap.setPixels((ByteBuffer) resizedImageMat.createBuffer());
   }

   private void decompressAndDecodeUsingOpenCV(CompressedImage compressedImage)
   {
      try
      {
         ROSOpenCVImage ROSOpenCVImage = ROSOpenCVTools.toCvCopy(compressedImage, ImageEncodingTools.RGBA8);
         Buffer buffer = ROSOpenCVImage.image.createBuffer();
         ensureTextureReady(ROSOpenCVImage.image.cols(), ROSOpenCVImage.image.rows());
         pixmap.setPixels((ByteBuffer) buffer);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

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
      }
   }

   private void ensureTextureReady(int width, int height)
   {
      if (texture == null || texture.getWidth() < width || texture.getHeight() < height)
      {
         if (texture != null)
         {
            texture.dispose();
//            pixmap.dispose();
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
