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
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1Visualizer;
import us.ihmc.perception.ROSOpenCVImage;
import us.ihmc.perception.ROSOpenCVTools;
import us.ihmc.perception.ImageEncodingTools;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.nio.Buffer;
import java.nio.ByteBuffer;

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
   private long receivedCount = 0;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiPlot receivedPlot = new ImGuiPlot("", 1000, 230, 20);
   private Mat inputImageMat;
   private Mat decodedImageMat;
   private Mat resizedImageMat;

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
      ImGui.sameLine();
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
            decompressAndDecodeUsingOpenCV(compressedImage);
            texture.draw(pixmap, 0, 0);
         }
         else if (image != null)
         {
            ensureTextureReady(image.getWidth(), image.getHeight());
            decodeUsingOpenCV(image);
            texture.draw(pixmap, 0, 0);
         }
      }
   }

   private void decodeUsingOpenCV(Image image)
   {
      if (inputImageMat == null)
      {
         String encoding = image.getEncoding();
         int cvType = ImageEncodingTools.getCvType(encoding);
         inputImageMat = new Mat(image.getHeight(), image.getWidth(), cvType);
         decodedImageMat = new Mat(image.getHeight(), image.getWidth(), opencv_core.CV_8UC4);
         resizedImageMat = new Mat(image.getHeight(), image.getWidth(), opencv_core.CV_8UC4);
      }

      ChannelBuffer nettyImageData = image.getData();
      ByteBuffer dataByteBuffer = nettyImageData.toByteBuffer();
      int arrayOffset = nettyImageData.arrayOffset();
      dataByteBuffer.position(arrayOffset);
      ByteBuffer offsetByteBuffer = dataByteBuffer.slice();

      BytePointer imageDataPointer = new BytePointer(offsetByteBuffer);
      inputImageMat.data(imageDataPointer);

      if (ImageEncodingTools.isMono(image.getEncoding()))
      {
         double min = 0.0;
         double max = ImageEncodingTools.getMaxBitValue(image.getEncoding());
         opencv_core.normalize(inputImageMat, inputImageMat, min, max, opencv_core.NORM_MINMAX, -1, opencv_core.noArray());
      }

      int conversionCode = ImageEncodingTools.getColorConversionCode(image.getEncoding(), ImageEncodingTools.RGBA8).get(0);
      opencv_imgproc.cvtColor(inputImageMat, decodedImageMat, conversionCode);

      int sourceDepth = ImageEncodingTools.bitDepth(image.getEncoding());
      double beta = 0.0;
      if (sourceDepth == 16)
         decodedImageMat.convertTo(resizedImageMat, opencv_core.CV_8UC4, 255. / 65535., beta);
      else
         decodedImageMat.convertTo(resizedImageMat, opencv_core.CV_8UC4);

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
