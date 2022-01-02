package us.ihmc.gdx.perception;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import sensor_msgs.Image;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.perception.OpenCLManager;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

public class GDXGPUPlanarRegionExtraction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
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
   private final ImBoolean useFilteredImage = new ImBoolean(true);
   private final ImInt gaussianSize = new ImInt(6);
   private final ImDouble gaussianSigma = new ImDouble(20.0);
   private GDXBytedecoImage inputFloatDepthImage;
   private GDXBytedecoImage inputScaledFloatDepthImage;
   private GDXBytedecoImage inputU16DepthImage;
   private GDXBytedecoImage blurredDepthImage;
   private GDXBytedecoImage filteredDepthImage;
   private GDXBytedecoImage nxImage;
   private GDXBytedecoImage nyImage;
   private GDXBytedecoImage nzImage;
   private GDXBytedecoImage gxImage;
   private GDXBytedecoImage gyImage;
   private GDXBytedecoImage gzImage;
   private GDXBytedecoImage graphImage;
   private ImGuiPanel imguiPanel;
   private GDXCVImagePanel blurredDepthPanel;
   private GDXCVImagePanel filteredDepthPanel;
   private int imageWidth;
   private int imageHeight;
   private Size gaussianKernelSize;
   private final OpenCLManager openCLManager = new OpenCLManager();
   private final long numberOfFloatParameters = 16;
   private final FloatPointer nativeParameterArray = new FloatPointer(numberOfFloatParameters);
   private _cl_mem parametersBufferObject;
   private _cl_program planarRegionExtractionProgram;
   private _cl_kernel filterKernel;
   private _cl_kernel packKernel;
   private _cl_kernel mergeKernel;
   private int subHeight;
   private int subWidth;
   private int patchHeight;
   private int patchWidth;
   private int filterSubHeight;
   private int filterSubWidth;

   public void create(int imageWidth, int imageHeight, ByteBuffer sourceDepthByteBufferOfFloats, double fx, double fy, double cx, double cy)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      inputWidth.set(imageWidth);
      inputHeight.set(imageHeight);
      depthFx.set((float) fx);
      depthFy.set((float) fy);
      depthCx.set((float) cx);
      depthCy.set((float) cy);

      calculateDetivativeParameters();

      inputFloatDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1, sourceDepthByteBufferOfFloats);
      inputScaledFloatDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1);
      inputU16DepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      blurredDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      filteredDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      nxImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_32FC1);
      nyImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_32FC1);
      nzImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_32FC1);
      gxImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_32FC1);
      gyImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_32FC1);
      gzImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_32FC1);
      graphImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_8UC1);
      gaussianKernelSize = new Size();

      imguiPanel = new ImGuiPanel("GPU Planar Region Extraction", this::renderImGuiWidgets);
      blurredDepthPanel = new GDXCVImagePanel("Blurred Depth", imageWidth, imageHeight);
      filteredDepthPanel = new GDXCVImagePanel("Filtered Depth", imageWidth, imageHeight);
      imguiPanel.addChild(blurredDepthPanel.getVideoPanel());
      imguiPanel.addChild(filteredDepthPanel.getVideoPanel());

      openCLManager.create();
      planarRegionExtractionProgram = openCLManager.loadProgram("PlanarRegionExtraction");
      filterKernel = openCLManager.createKernel(planarRegionExtractionProgram, "filterKernel");
      packKernel = openCLManager.createKernel(planarRegionExtractionProgram, "packKernel");
      mergeKernel = openCLManager.createKernel(planarRegionExtractionProgram, "mergeKernel");


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
      // convert float to unint16
      // multiply by 1000 and cast to int
      double scaleFactor = 1000.0; // convert meters to millimeters
      double delta = 0.0; // no delta added
      int resultType = -1; // the output matrix will have the same type as the input
      inputFloatDepthImage.getBytedecoOpenCVMat().convertTo(inputScaledFloatDepthImage.getBytedecoOpenCVMat(), resultType, scaleFactor, delta);
      scaleFactor = 1.0;
      resultType = opencv_core.CV_16UC1;
      inputScaledFloatDepthImage.getBytedecoOpenCVMat().convertTo(inputU16DepthImage.getBytedecoOpenCVMat(), resultType, scaleFactor, delta);

      int size = gaussianSize.get() * 2 + 1;
      gaussianKernelSize.width(size);
      gaussianKernelSize.height(size);
      double sigmaX = gaussianSigma.get();
      double sigmaY = sigmaX;
      int borderType = opencv_core.BORDER_DEFAULT;
      opencv_imgproc.GaussianBlur(inputU16DepthImage.getBytedecoOpenCVMat(),
                                  blurredDepthImage.getBytedecoOpenCVMat(),
                                  gaussianKernelSize,
                                  sigmaX,
                                  sigmaY,
                                  borderType);

      blurredDepthPanel.drawFloatImage(blurredDepthImage.getBytedecoOpenCVMat());

      inputU16DepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      blurredDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      filteredDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      nxImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      nyImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      nzImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      gxImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      gyImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      gzImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      graphImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      parametersBufferObject = openCLManager.createBufferObject(numberOfFloatParameters * Float.BYTES, nativeParameterArray);

      enqueueWriteParameters();
      _cl_mem inputImage = earlyGaussianBlur.get() ? blurredDepthImage.getOpenCLImageObject() : inputU16DepthImage.getOpenCLImageObject();

      openCLManager.setKernelArgument(filterKernel, 0, inputImage);
      openCLManager.setKernelArgument(filterKernel, 1, filteredDepthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(filterKernel, 2, nxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(filterKernel, 3, parametersBufferObject);

      _cl_mem packKernelInputObject = useFilteredImage.get() ? filteredDepthImage.getOpenCLImageObject() : inputU16DepthImage.getOpenCLImageObject();
      openCLManager.setKernelArgument(packKernel, 0, packKernelInputObject);
      openCLManager.setKernelArgument(packKernel, 1, nxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 2, nyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 3, nzImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 4, gxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 5, gyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 6, gzImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 7, parametersBufferObject);

      openCLManager.setKernelArgument(mergeKernel, 0, nxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 1, nyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 2, nzImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 3, gxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 4, gyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 5, gzImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 6, graphImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 7, parametersBufferObject);

      openCLManager.execute2D(filterKernel, filterSubHeight, filterSubWidth); // TODO: Check X & Y vs height and width
      openCLManager.execute2D(packKernel, subHeight, subWidth);
      openCLManager.execute2D(mergeKernel, subHeight, subWidth);

      openCLManager.enqueueReadImage(filteredDepthImage.getOpenCLImageObject(), imageWidth, imageHeight, filteredDepthImage.getBytedecoByteBufferPointer());

      openCLManager.finish();

      filteredDepthPanel.drawFloatImage(filteredDepthImage.getBytedecoOpenCVMat());
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
      enqueueWriteParameters();

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

   private void enqueueWriteParameters()
   {
      calculateDetivativeParameters();

      nativeParameterArray.put(0, filterDisparityThreshold.get());
      nativeParameterArray.put(1, mergeAngularThreshold.get());
      nativeParameterArray.put(2, mergeDistanceThreshold.get());
      nativeParameterArray.put(3, patchHeight);
      nativeParameterArray.put(4, patchWidth);
      nativeParameterArray.put(5, subHeight);
      nativeParameterArray.put(6, subWidth);
      nativeParameterArray.put(7, depthFx.get());
      nativeParameterArray.put(8, depthFy.get());
      nativeParameterArray.put(9, depthCx.get());
      nativeParameterArray.put(10, depthCy.get());
      nativeParameterArray.put(11, filterKernelSize.get());
      nativeParameterArray.put(12, filterSubHeight);
      nativeParameterArray.put(13, filterSubWidth);
      nativeParameterArray.put(14, inputHeight.get());
      nativeParameterArray.put(15, inputWidth.get());

      openCLManager.enqueueWriteBuffer(parametersBufferObject, nativeParameterArray); // TODO: Necessary?
   }

   private void calculateDetivativeParameters()
   {
      patchHeight = kernelSliderLevel.get();
      patchWidth = kernelSliderLevel.get();
      subHeight = inputHeight.get() / patchHeight;
      subWidth = inputWidth.get() / patchWidth;
      filterSubHeight = inputHeight.get() / filterKernelSize.get();
      filterSubWidth = inputWidth.get() / filterKernelSize.get();
   }

   public void renderImGuiWidgets()
   {
      ImGui.inputInt(labels.get("Input height"), inputHeight);
      ImGui.inputInt(labels.get("Input width"), inputWidth);
      ImGui.inputFloat(labels.get("Depth Fx"), depthFx);
      ImGui.inputFloat(labels.get("Depth Fy"), depthFy);
      ImGui.inputFloat(labels.get("Depth Cx"), depthCx);
      ImGui.inputFloat(labels.get("Depth Cy"), depthCy);
      ImGui.checkbox(labels.get("Early gaussian blur"), earlyGaussianBlur);
      ImGui.inputInt(labels.get("Gaussian size"), gaussianSize);
      ImGui.inputDouble(labels.get("Gaussian sigma"), gaussianSigma);
      ImGui.inputInt(labels.get("Kernel slider level"), kernelSliderLevel);
      ImGui.inputInt(labels.get("Filter kernel size"), filterKernelSize);
      ImGui.checkbox(labels.get("Use filtered image"), useFilteredImage);

   }

   public ImGuiPanel getPanel()
   {
      return imguiPanel;
   }
}
