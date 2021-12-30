package us.ihmc.gdx.perception;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
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
   private GDXBytedecoImage inputDepthImage;
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
   private _cl_mem clInputDepthImageObject;
   private _cl_mem clFilteredDepthImageObject;
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
   private _cl_mem nxBufferObject;
   private _cl_mem nyBufferObject;
   private _cl_mem nzBufferObject;
   private _cl_mem gxBufferObject;
   private _cl_mem gyBufferObject;
   private _cl_mem gzBufferObject;
   private _cl_mem graphBufferObject;

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

      inputDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1, sourceDepthByteBufferOfFloats);
      blurredDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1);
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

//      clInputDepthImageObject = openCLManager.createImage(OpenCL.CL_MEM_READ_ONLY, imageWidth, imageHeight, inputDepthImage.getBytedecoByteBufferPointer());
      clInputDepthImageObject = openCLManager.createImage(OpenCL.CL_MEM_READ_ONLY, imageWidth, imageHeight, null);
      clFilteredDepthImageObject
            = openCLManager.createBufferObject((long) imageWidth * imageHeight * Short.BYTES, filteredDepthImage.getBytedecoByteBufferPointer());
      nxBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight * Float.BYTES, nxImage.getBytedecoByteBufferPointer());
      nyBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight * Float.BYTES, nyImage.getBytedecoByteBufferPointer());
      nzBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight * Float.BYTES, nzImage.getBytedecoByteBufferPointer());
      gxBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight * Float.BYTES, gxImage.getBytedecoByteBufferPointer());
      gyBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight * Float.BYTES, gyImage.getBytedecoByteBufferPointer());
      gzBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight * Float.BYTES, gzImage.getBytedecoByteBufferPointer());
      graphBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight, graphImage.getBytedecoByteBufferPointer());
      parametersBufferObject = openCLManager.createBufferObject(numberOfFloatParameters * Float.BYTES, nativeParameterArray);
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

      blurredDepthPanel.drawFloatImage(blurredDepthImage.getBytedecoOpenCVMat());

      enqueueWriteParameters();
      BytePointer inputImage = earlyGaussianBlur.get() ? blurredDepthImage.getBytedecoByteBufferPointer() : inputDepthImage.getBytedecoByteBufferPointer();
      openCLManager.enqueueWriteBuffer(clInputDepthImageObject, inputImage);

      openCLManager.setKernelArgument(filterKernel, 0, clInputDepthImageObject);
      openCLManager.setKernelArgument(filterKernel, 1, clFilteredDepthImageObject);
      openCLManager.setKernelArgument(filterKernel, 2, nxBufferObject);
      openCLManager.setKernelArgument(filterKernel, 3, parametersBufferObject);

      _cl_mem packKernelInputObject = useFilteredImage.get() ? clFilteredDepthImageObject : clInputDepthImageObject;
      openCLManager.setKernelArgument(packKernel, 0, packKernelInputObject);
      openCLManager.setKernelArgument(packKernel, 1, nxBufferObject);
      openCLManager.setKernelArgument(packKernel, 2, nyBufferObject);
      openCLManager.setKernelArgument(packKernel, 3, nzBufferObject);
      openCLManager.setKernelArgument(packKernel, 4, gxBufferObject);
      openCLManager.setKernelArgument(packKernel, 5, gyBufferObject);
      openCLManager.setKernelArgument(packKernel, 6, gzBufferObject);
      openCLManager.setKernelArgument(packKernel, 7, parametersBufferObject);

      openCLManager.setKernelArgument(mergeKernel, 0, nxBufferObject);
      openCLManager.setKernelArgument(mergeKernel, 1, nyBufferObject);
      openCLManager.setKernelArgument(mergeKernel, 2, nzBufferObject);
      openCLManager.setKernelArgument(mergeKernel, 3, gxBufferObject);
      openCLManager.setKernelArgument(mergeKernel, 4, gyBufferObject);
      openCLManager.setKernelArgument(mergeKernel, 5, gzBufferObject);
      openCLManager.setKernelArgument(mergeKernel, 6, graphBufferObject);
      openCLManager.setKernelArgument(mergeKernel, 7, parametersBufferObject);

      openCLManager.execute2D(filterKernel, filterSubHeight, filterSubWidth);
      openCLManager.execute2D(packKernel, subHeight, subWidth);
      openCLManager.execute2D(mergeKernel, subHeight, subWidth);

      openCLManager.enqueueReadBuffer(clFilteredDepthImageObject, filteredDepthImage.getBytedecoByteBufferPointer());

      openCLManager.finish();

      filteredDepthPanel.drawFloatImage(filteredDepthImage.getBytedecoOpenCVMat());

      filteredDepthPanel.draw();
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

      openCLManager.enqueueWriteBuffer(parametersBufferObject, nativeParameterArray);
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
