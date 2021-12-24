package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Size;
import org.jboss.netty.buffer.ChannelBuffer;
import sensor_msgs.Image;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.perception.ImageEncodingTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.ROSOpenCVTools;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

public class GDXGPUPlanarRegionExtraction
{
   private final ImFloat mergeDistanceThreshold = new ImFloat(0.016f);
   private final ImFloat mergeAngularThreshold = new ImFloat(0.82f);
   private final ImFloat filterDisparityThreshold = new ImFloat(2000);
   private final ImInt inputHeight = new ImInt(0);
   private final ImInt inputWidth = new ImInt(0);
   private final ImInt kernelSliderLevel = new ImInt(2);
   private final ImInt filterKernelSize = new ImInt(4);
   private final ImFloat depthFx = new ImFloat(0);
   private final ImFloat depthFy = new ImFloat(0);
   private final ImFloat depthCx = new ImFloat(0);
   private final ImFloat depthCy = new ImFloat(0);
   private final ImBoolean earlyGaussianBlur = new ImBoolean(true);
   private final ImInt gaussianSize = new ImInt(6);
   private final ImInt gaussianSigma = new ImInt(20);
   private final OpenCLManager openCLManager = new OpenCLManager();
   private int numberOfFloatParameters = 16;
   private _cl_mem parametersBufferObject;
   private long parametersBufferSizeInBytes;
   private FloatPointer parametersNativeCPUPointer;
   private Mat inputDepthImageMat;
   private Mat blurredDepthImageMat;
   private Mat blurredDepthImageMatForDisplay;
   private Mat blurredDepthImageMatForDisplayRGBA8888;
   private Mat blurredDepthImageMatForDisplayConvertedRGBA8888;
   private BytePointer blurredDepthForDisplayPixmapBytedecoPointer;
   private final ImGuiVideoPanel blurredDepthPanel;
   private Pixmap blurredDepthPanelPixmap;
   private Texture blurredDepthPanelTexture;
   private int imageWidth;
   private int imageHeight;
   private float lowestValueSeen = -1.0f;
   private float highestValueSeen = -1.0f;
   private BytePointer eyeDepthMetersBytedecoPointer;
   private Size gaussianKernelSize;

   public GDXGPUPlanarRegionExtraction()
   {
      blurredDepthPanel = new ImGuiVideoPanel("Blurred Depth", false);
   }

   public void create(int imageWidth, int imageHeight, ByteBuffer sourceDepthByteBufferOfFloats)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      openCLManager.create();

      parametersBufferSizeInBytes = numberOfFloatParameters * Loader.sizeof(FloatPointer.class);
      parametersBufferObject = openCLManager.createBufferObject(parametersBufferSizeInBytes);
      parametersNativeCPUPointer = new FloatPointer(numberOfFloatParameters);

      inputDepthImageMat = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1);
      blurredDepthImageMat = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1);
      blurredDepthImageMatForDisplay = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1);
//      ByteBuffer displayPixmap = ByteBuffer.allocateDirect(inputHeight * imageWidth * 4);
      gaussianKernelSize = new Size();

      blurredDepthPanelPixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
      blurredDepthForDisplayPixmapBytedecoPointer = new BytePointer(blurredDepthPanelPixmap.getPixels());
      blurredDepthImageMatForDisplayRGBA8888 = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4);
      blurredDepthImageMatForDisplayConvertedRGBA8888 = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4);
//      blurredDepthPanelPixmap.getPixels()
      blurredDepthPanelTexture = new Texture(new PixmapTextureData(blurredDepthPanelPixmap, null, false, false));

      blurredDepthPanel.setTexture(blurredDepthPanelTexture);

      eyeDepthMetersBytedecoPointer = new BytePointer(sourceDepthByteBufferOfFloats);
      inputDepthImageMat.data(eyeDepthMetersBytedecoPointer);
   }

   public void processROS1DepthImage(Image image)
   {
      // See us.ihmc.gdx.ui.graphics.live.GDXROS1VideoVisualizer.decodeUsingOpenCV
      if (inputDepthImageMat == null)
      {
         String encoding = image.getEncoding();
         int cvType = ImageEncodingTools.getCvType(encoding);
         inputDepthImageMat = new Mat(image.getHeight(), image.getWidth(), cvType);
      }

      ROSOpenCVTools.backMatWithNettyBuffer(inputDepthImageMat, image.getData());
   }

   public void blurDepthAndRender(ByteBuffer depthByteBufferOfFloats)
   {
//      depthByteBufferOfFloats.rewind(); // TODO: Necessary?
//      eyeDepthMetersBytedecoPointer.position(depthByteBufferOfFloats.position()); // TODO: Necessary?
//      inputDepthImageMat.data(new BytePointer(depthByteBufferOfFloats));

      int size = gaussianSize.get() * 2 + 1;
      gaussianKernelSize.width(size);
      gaussianKernelSize.height(size);
//      inputDepthImageMat.copyTo(blurredDepthImageMat);
      opencv_imgproc.GaussianBlur(inputDepthImageMat, blurredDepthImageMat, gaussianKernelSize, gaussianSigma.get());

//      blurredDepthImageMat.copyTo(blurredDepthImageMatForDisplay);
      double min = 0.0;
      double max = 65535.0;
//      blurredDepthImageMatForDisplay.copyTo(blurredDepthImageMatForDisplay);
      opencv_core.normalize(blurredDepthImageMat, blurredDepthImageMat, min, max, opencv_core.NORM_MINMAX, -1, opencv_core.noArray());
      blurredDepthImageMatForDisplay.copyTo(blurredDepthImageMatForDisplayRGBA8888);
//      opencv_imgproc.cvtColor(blurredDepthImageMatForDisplay, blurredDepthImageMatForDisplayRGBA8888, opencv_imgproc.COLOR_GRAY2RGBA);
      double alpha = 255.0 / max;
      double beta = 0.0;
//      blurredDepthImageMatForDisplayRGBA8888.copyTo(blurredDepthImageMatForDisplayConvertedRGBA8888);
//      blurredDepthImageMat.convertTo(blurredDepthImageMatForDisplayConvertedRGBA8888, opencv_core.CV_8UC4, alpha, beta);

//      depthByteBufferOfFloats.rewind();
//      inputDepthImageMat.data(eyeDepthMetersBytedecoPointer);
      eyeDepthMetersBytedecoPointer.position(0);
      for (int y = 0; y < imageHeight; y++)
      {
         for (int x = 0; x < imageWidth; x++)
         {
//            float eyeDepth = depthByteBufferOfFloats.getFloat();
//            float eyeDepth = blurredDepthImageMat.ptr(y, x).getFloat();
            float eyeDepth = eyeDepthMetersBytedecoPointer.getFloat();
            eyeDepthMetersBytedecoPointer.position(eyeDepthMetersBytedecoPointer.position() + 4);
//            BytePointer ptr = inputDepthImageMat.ptr(y, x);
//            float eyeDepth3 = ptr.getFloat();
//            System.out.println(eyeDepth + eyeDepth2 + eyeDepth3);
//            float eyeDepth = inputDepthImageMat.ptr(y, x).getFloat();
            if (highestValueSeen < 0 || eyeDepth > highestValueSeen)
               highestValueSeen = eyeDepth;
            if (lowestValueSeen < 0 || eyeDepth < lowestValueSeen)
               lowestValueSeen = eyeDepth;

            float colorRange = highestValueSeen - lowestValueSeen;
            float grayscale = (eyeDepth - lowestValueSeen) / colorRange;
            int flippedY = imageHeight - y;

            blurredDepthPanelPixmap.drawPixel(x, flippedY, Color.rgba8888(grayscale, grayscale, grayscale, 1.0f));
         }
      }

//      blurredDepthPanelPixmap.getPixels().rewind();
//      blurredDepthForDisplayPixmapBytedecoPointer.position(blurredDepthPanelPixmap.getPixels().position());
//      blurredDepthPanelPixmap.setPixels(blurredDepthImageMatForDisplayConvertedRGBA8888.createBuffer()); // FIXME maintain buffer
      blurredDepthPanelTexture.draw(blurredDepthPanelPixmap, 0, 0);
   }

   private void generateRegionsFromDepth(FloatBuffer depthFloatBuffer)
   {
      // timestamp

      // put image into Mat?

//      depthImageBytePointer.putPointerValue(depthFloatBuffer);
//       inputDepthImageMat.


      generatePatchGraph();

      generateSegmentation();
   }

   private void generatePatchGraph()
   {
      uploadParametersToGPU();

      // gaussian blur
      Mat src = null;
      Mat dst = null;
      Size ksize = null;
      double sigmaX = 0.0;
      opencv_imgproc.GaussianBlur(src, dst, ksize, sigmaX);
   }

   private void generateSegmentation()
   {

   }

   private void uploadParametersToGPU()
   {
      int patchHeight = kernelSliderLevel.get();
      int patchWidth = kernelSliderLevel.get();
      int subHeight = inputHeight.get() / patchHeight;
      int subWidth = inputWidth.get() / patchWidth;
      int filterSubHeight = inputHeight.get() / filterKernelSize.get();
      int filterSubWidth = inputWidth.get() / filterKernelSize.get();

      FloatPointer parameters = new FloatPointer(numberOfFloatParameters);
      parameters.put(0, filterDisparityThreshold.get());
      parameters.put(1, mergeAngularThreshold.get());
      parameters.put(2, mergeDistanceThreshold.get());
      parameters.put(3, patchHeight);
      parameters.put(4, patchWidth);
      parameters.put(5, subHeight);
      parameters.put(6, subWidth);
      parameters.put(7, depthFx.get());
      parameters.put(8, depthFy.get());
      parameters.put(9, depthCx.get());
      parameters.put(10, depthCy.get());
      parameters.put(11, filterKernelSize.get());
      parameters.put(12, filterSubHeight);
      parameters.put(13, filterSubWidth);
      parameters.put(14, inputHeight.get());
      parameters.put(15, inputWidth.get());

      openCLManager.enqueueWriteBuffer(parametersBufferObject, parametersBufferSizeInBytes, parameters);
   }

   public ImGuiVideoPanel getBlurredDepthPanel()
   {
      return blurredDepthPanel;
   }
}
