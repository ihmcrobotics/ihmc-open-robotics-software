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
   private final ImBoolean filterSelected = new ImBoolean(true);
   private final ImInt gaussianSize = new ImInt(6);
   private final ImDouble gaussianSigma = new ImDouble(20.0);
   private GDXBytedecoImage inputDepthImage;
   private GDXBytedecoImage blurredDepthImage;
   private ImGuiPanel imguiPanel;
   private GDXCVImagePanel blurredDepthPanel;
   private GDXCVImagePanel nextStepPanel;
   private int imageWidth;
   private int imageHeight;
   private Size gaussianKernelSize;
   private final OpenCLManager openCLManager = new OpenCLManager();
   private long numberOfFloatParameters = 16;
   private _cl_mem parametersBufferObject;
   private _cl_mem clInputDepthImageObject;
   private _cl_mem clFilteredDepthImageObject;
   private long parametersBufferSizeInBytes;
   private FloatPointer parametersNativeCPUPointer;
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

   public void create(int imageWidth, int imageHeight, ByteBuffer sourceDepthByteBufferOfFloats)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      inputWidth.set(imageWidth);
      inputHeight.set(imageHeight);

      inputDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1, sourceDepthByteBufferOfFloats);
      blurredDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1);
      gaussianKernelSize = new Size();

      imguiPanel = new ImGuiPanel("GPU Planar Region Extraction", this::renderImGuiWidgets);
      blurredDepthPanel = new GDXCVImagePanel("Blurred Depth", imageWidth, imageHeight);
      nextStepPanel = new GDXCVImagePanel("Next Step", imageWidth, imageHeight);
      imguiPanel.addChild(blurredDepthPanel.getVideoPanel());
      imguiPanel.addChild(nextStepPanel.getVideoPanel());

      openCLManager.create();
      planarRegionExtractionProgram = openCLManager.loadProgram("PlanarRegionExtraction");
      filterKernel = openCLManager.createKernel(planarRegionExtractionProgram, "filterKernel");
      packKernel = openCLManager.createKernel(planarRegionExtractionProgram, "packKernel");
      mergeKernel = openCLManager.createKernel(planarRegionExtractionProgram, "mergeKernel");

      parametersBufferObject = openCLManager.createBufferObject(numberOfFloatParameters * Float.BYTES);
      parametersNativeCPUPointer = new FloatPointer(numberOfFloatParameters);

      clInputDepthImageObject = openCLManager.createBufferObject((long) imageWidth * imageHeight * Float.BYTES);
      clFilteredDepthImageObject = openCLManager.createBufferObject((long) imageWidth * imageHeight * Float.BYTES);
      nxBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight * Float.BYTES);
      nyBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight * Float.BYTES);
      nzBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight * Float.BYTES);
      gxBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight * Float.BYTES);
      gyBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight * Float.BYTES);
      gzBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight * Float.BYTES);
      graphBufferObject = openCLManager.createBufferObject((long) subWidth * subHeight);


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

      blurredDepthPanel.draw32FImage(blurredDepthImage.getBytedecoOpenCVMat());

      uploadParametersToGPU();
      openCLManager.enqueueWriteBuffer(clInputDepthImageObject, inputDepthImage.getBytedecoByteBufferPointer());

      openCLManager.setKernelArgument(filterKernel, 0, clInputDepthImageObject);
      openCLManager.setKernelArgument(filterKernel, 1, clFilteredDepthImageObject);
      openCLManager.setKernelArgument(filterKernel, 2, nxBufferObject);
      openCLManager.setKernelArgument(filterKernel, 3, parametersBufferObject);

      _cl_mem packKernelInputObject = filterSelected.get() ? clFilteredDepthImageObject : clInputDepthImageObject;
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

      openCLManager.execute2D(filterKernel, subHeight, subWidth);


      nextStepPanel.draw();
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
      patchHeight = kernelSliderLevel.get();
      patchWidth = kernelSliderLevel.get();
      subHeight = inputHeight.get() / patchHeight;
      subWidth = inputWidth.get() / patchWidth;
      filterSubHeight = inputHeight.get() / filterKernelSize.get();
      filterSubWidth = inputWidth.get() / filterKernelSize.get();

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

      openCLManager.enqueueWriteBuffer(parametersBufferObject, parameters);
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
