package us.ihmc.gdx.ui.graphics.live;

import imgui.internal.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.CompressedImage;
import sensor_msgs.Image;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1VisualizerInterface;
import us.ihmc.perception.*;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.nio.Buffer;
import java.nio.ByteBuffer;

public class GDXROS1VideoVisualizer extends GDXOpenCVVideoVisualizer implements ImGuiGDXROS1VisualizerInterface
{
   private final boolean isCompressed;
   private AbstractRosTopicSubscriber<Image> subscriber;
   private AbstractRosTopicSubscriber<CompressedImage> compressedSubscriber;
   private final String topic;
   private Mat input8UC1Mat;
   private Mat input16UC1Mat;
   private Mat decodedImageMat;
   private Mat resizedImageMat;
   private boolean currentlySubscribed = false;
   private final String name;

   public GDXROS1VideoVisualizer(String title, String topic)
   {
      this(title, topic, false);
   }

   public GDXROS1VideoVisualizer(String title, String topic, boolean flipY)
   {
      super(title + " (ROS 1)", topic, flipY);
      name = title;
      this.topic = topic;
      isCompressed = topic.endsWith("compressed");
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
               doReceiveMessageOnThread(() -> processIncomingMessageOnThread(image));
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
               doReceiveMessageOnThread(() -> processIncomingMessageOnThread(image));
            }
         };
         ros1Node.attachSubscriber(topic, subscriber);
      }
   }

   private void processIncomingMessageOnThread(CompressedImage compressedImage)
   {
//      ChannelBuffer nettyChannelBuffer = compressedImage.getData();
      //         decodedImageMat = new Mat(image.getHeight(), image.getWidth(), opencv_core.CV_8UC4);
      //         resizedImageMat = new Mat(image.getHeight(), image.getWidth(), opencv_core.CV_8UC4);

      try
      {
         ROSOpenCVImage ROSOpenCVImage = ROSOpenCVTools.toCvCopy(compressedImage, ImageEncodingTools.RGBA8);
         Buffer buffer = ROSOpenCVImage.image.createBuffer();
         //         ensureTextureReady(ROSOpenCVImage.image.cols(), ROSOpenCVImage.image.rows());
         //         pixmap.setPixels((ByteBuffer) buffer);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   private void processIncomingMessageOnThread(Image image)
   {
      ChannelBuffer nettyChannelBuffer = image.getData();

      int imageWidth = image.getWidth();
      int imageHeight = image.getHeight();
      String encoding = image.getEncoding();
      int cvType = ImageEncodingTools.getCvType(encoding);

      if (cvType == opencv_core.CV_16UC1)
      {
         if (input16UC1Mat == null || input16UC1Mat.rows() != imageHeight || input16UC1Mat.cols() != imageWidth)
         {
            input16UC1Mat = new Mat(imageHeight, imageWidth, cvType);
            input8UC1Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC1);
         }

         ROSOpenCVTools.backMatWithNettyBuffer(input16UC1Mat, nettyChannelBuffer);
         BytedecoOpenCVTools.clampTo8BitUnsignedChar(input16UC1Mat, input8UC1Mat, 0.0, 255.0);
      }

      synchronized (this)
      {
         updateImageDimensions(imageWidth, imageHeight);

         if (cvType == opencv_core.CV_16UC1)
         {
            opencv_imgproc.cvtColor(input8UC1Mat, getRGBA8Mat(), opencv_imgproc.COLOR_GRAY2RGBA);
         }
         else if (cvType == opencv_core.CV_8UC4)
         {
            ByteBuffer inputByteBuffer = RosTools.sliceNettyBuffer(nettyChannelBuffer);
            BytePointer rgba8888BytePointer = getRgba8888BytePointer();
            for (int i = 0; i < inputByteBuffer.limit(); i++)
            {
               rgba8888BytePointer.put(i, inputByteBuffer.get());
            }
         }
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
      getFrequencyPlot().renderImGuiWidgets();
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

//      pixmap.setPixels((ByteBuffer) resizedImageMat.createBuffer());
   }

   public void updateSubscribers(RosNodeInterface ros1Node)
   {
      boolean active = isActive();
      if (active != currentlySubscribed)
      {
         if (active)
         {
            subscribe(ros1Node);
         }
         else
         {
            unsubscribe(ros1Node);
         }
      }
      currentlySubscribed = active;
   }
}
