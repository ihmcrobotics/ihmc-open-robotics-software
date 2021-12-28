package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.Color;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import sensor_msgs.Image;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.perception.OpenCLManager;

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
   private final ImDouble gaussianSigma = new ImDouble(20.0);
   private final OpenCLManager openCLManager = new OpenCLManager();
   private int numberOfFloatParameters = 16;
   private _cl_mem parametersBufferObject;
   private long parametersBufferSizeInBytes;
   private FloatPointer parametersNativeCPUPointer;
   private GDXBytedecoImage inputDepthImage;
   private GDXBytedecoImage blurredDepthImage;
   private GDXBytedecoImage blurredNormalizedDepthImage;
   private GDXBytedecoImage blurredNormalizedScaledDepthImage;
   private GDXBytedecoImage blurredNormalizedConvertedDepthImage;
   private ImGuiPanel imguiPanel;
   private GDXCVImagePanel blurredImagePanel;
   private GDXCVImagePanel normalizedImagePanel;
   private GDXCVImagePanel convertedImagePanel;
   private int imageWidth;
   private int imageHeight;
   private float lowestValueSeen = -1.0f;
   private float highestValueSeen = -1.0f;
   private Size gaussianKernelSize;

   public void create(int imageWidth, int imageHeight, ByteBuffer sourceDepthByteBufferOfFloats)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      inputWidth.set(imageWidth);
      inputHeight.set(imageHeight);
      openCLManager.create();

      parametersBufferSizeInBytes = (long) numberOfFloatParameters * Loader.sizeof(FloatPointer.class);
      parametersBufferObject = openCLManager.createBufferObject(parametersBufferSizeInBytes);
      parametersNativeCPUPointer = new FloatPointer(numberOfFloatParameters);

      inputDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1, sourceDepthByteBufferOfFloats);
      blurredDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1);
      blurredNormalizedDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1);
      blurredNormalizedScaledDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC1);
      blurredNormalizedConvertedDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4);
      gaussianKernelSize = new Size();

      imguiPanel = new ImGuiPanel("GPU Planar Region Extraction", this::renderImGuiWidgets);
      blurredImagePanel = new GDXCVImagePanel("Blurred Depth", imageWidth, imageHeight);
      normalizedImagePanel = new GDXCVImagePanel("Normalized Depth", imageWidth, imageHeight);
      convertedImagePanel = new GDXCVImagePanel("Converted Depth", imageWidth, imageHeight);
      imguiPanel.addChild(blurredImagePanel.getVideoPanel());
      imguiPanel.addChild(normalizedImagePanel.getVideoPanel());
      imguiPanel.addChild(convertedImagePanel.getVideoPanel());
   }

   public void processROS1DepthImage(Image image)
   {
      // See us.ihmc.gdx.ui.graphics.live.GDXROS1VideoVisualizer.decodeUsingOpenCV
//      if (inputDepthImageMat == null)
//      {
//         String encoding = image.getEncoding();
//         int cvType = ImageEncodingTools.getCvType(encoding);
//         inputDepthImageMat = new Mat(image.getHeight(), image.getWidth(), cvType);
//      }
//
//      ROSOpenCVTools.backMatWithNettyBuffer(inputDepthImageMat, image.getData());
   }

   public void blurDepthAndRender(ByteBuffer depthByteBufferOfFloats)
   {
      int size = gaussianSize.get() * 2 + 1;
      gaussianKernelSize.width(size);
      gaussianKernelSize.height(size);
      double sigmaX = gaussianSigma.get();
      double sigmaY = sigmaX;
      int borderType = opencv_core.BORDER_DEFAULT;
      opencv_imgproc.GaussianBlur(inputDepthImage.getBytedecoOpenCVMat(),
                                  blurredDepthImage.getBytedecoOpenCVMat(),
                                  gaussianKernelSize,
                                  sigmaX,
                                  sigmaY,
                                  borderType);

      double min = 0.0;
      double max = 1.0;
      int normType = opencv_core.NORM_MINMAX;
      int depthType = -1; // output array has the same type as src
      Mat mask = opencv_core.noArray(); // no operations
      opencv_core.normalize(blurredDepthImage.getBytedecoOpenCVMat(),
                            blurredNormalizedDepthImage.getBytedecoOpenCVMat(),
                            min,
                            max,
                            normType,
                            depthType,
                            mask);

      min = 0.0;
      max = 255.0;
      depthType = opencv_core.CV_8U; // converting 32 bit to 8 bit
      opencv_core.normalize(blurredDepthImage.getBytedecoOpenCVMat(),
                            blurredNormalizedScaledDepthImage.getBytedecoOpenCVMat(),
                            min,
                            max,
                            normType,
                            depthType,
                            mask);

      int destinationChannels = 0; // automatic mode
      opencv_imgproc.cvtColor(blurredNormalizedScaledDepthImage.getBytedecoOpenCVMat(),
                              blurredNormalizedConvertedDepthImage.getBytedecoOpenCVMat(),
                              opencv_imgproc.COLOR_GRAY2RGBA,
                              destinationChannels);
//      double scale = 255.0;
//      double delta = 0.0; // no delta added
//      int resultType = -1; // the output matrix will have the same type as the input
//      blurredNormalizedConvertedDepthImage.getBytedecoOpenCVMat().convertTo(blurredNormalizedConvertedDepthImage.getBytedecoOpenCVMat(),
//                                                                            resultType,
//                                                                            scale,
//                                                                            delta);

      blurredDepthImage.rewind();
      blurredNormalizedDepthImage.rewind();
      blurredNormalizedConvertedDepthImage.rewind();
      blurredNormalizedScaledDepthImage.rewind();
      for (int y = 0; y < imageHeight; y++)
      {
         for (int x = 0; x < imageWidth; x++)
         {
            float eyeDepth = blurredDepthImage.getBackingDirectByteBuffer().getFloat();

            if (highestValueSeen < 0 || eyeDepth > highestValueSeen)
               highestValueSeen = eyeDepth;
            if (lowestValueSeen < 0 || eyeDepth < lowestValueSeen)
               lowestValueSeen = eyeDepth;

            float colorRange = highestValueSeen - lowestValueSeen;
            float grayscale = (eyeDepth - lowestValueSeen) / colorRange;
            int flippedY = imageHeight - y;

            blurredImagePanel.getPixmap().drawPixel(x, flippedY, Color.rgba8888(grayscale, grayscale, grayscale, 1.0f));

            float normalizedDepth = blurredNormalizedDepthImage.getBackingDirectByteBuffer().getFloat();
            normalizedImagePanel.getPixmap().drawPixel(x, flippedY, Color.rgba8888(normalizedDepth, normalizedDepth, normalizedDepth, 1.0f));

            int r = Byte.toUnsignedInt(blurredNormalizedConvertedDepthImage.getBackingDirectByteBuffer().get());
            int g = Byte.toUnsignedInt(blurredNormalizedConvertedDepthImage.getBackingDirectByteBuffer().get());
            int b = Byte.toUnsignedInt(blurredNormalizedConvertedDepthImage.getBackingDirectByteBuffer().get());
            int a = Byte.toUnsignedInt(blurredNormalizedConvertedDepthImage.getBackingDirectByteBuffer().get());
            convertedImagePanel.getPixmap().drawPixel(x, flippedY, r << 24 | g << 16 | b << 8 | a);

//            int gray2 = Byte.toUnsignedInt(blurredNormalizedScaledDepthImage.getBackingDirectByteBuffer().get());
//            convertedImagePanel.getPixmap().drawPixel(x, flippedY, gray2 << 24 | gray2 << 16 | gray2 << 8 | 255);
         }
      }

//      blurredDepthPanelPixmap.getPixels().rewind();
//      blurredDepthPanelPixmap.setPixels(blurredDepthImageMatForDisplayConvertedRGBA8888.createBuffer()); // FIXME maintain buffer
      blurredImagePanel.draw();
      normalizedImagePanel.draw();
      convertedImagePanel.draw();
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

   public void renderImGuiWidgets()
   {
      ImGui.inputInt("Gaussian size", gaussianSize);
      ImGui.inputDouble("Gaussian sigma", gaussianSigma);
   }

   public ImGuiPanel getPanel()
   {
      return imguiPanel;
   }
}
