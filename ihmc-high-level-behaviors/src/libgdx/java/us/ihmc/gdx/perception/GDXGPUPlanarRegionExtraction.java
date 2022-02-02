package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.*;
import org.ejml.data.BMatrixRMaj;
import org.ejml.data.DMatrixRMaj;
import sensor_msgs.Image;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.robotEnvironmentAwareness.geometry.*;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.perception.ProjectionTools;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class GDXGPUPlanarRegionExtraction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private final ImFloat mergeDistanceThreshold = new ImFloat(0.016f);
   private final ImFloat mergeAngularThreshold = new ImFloat(0.82f);
   private final ImFloat filterDisparityThreshold = new ImFloat(2000);
   private final ImInt desiredPatchSize = new ImInt(16);
   private final ImInt patchSize = new ImInt(desiredPatchSize.get());
   private final ImInt deadPixelFilterPatchSize = new ImInt(4);
   private final ImFloat focalLengthXPixels = new ImFloat(0);
   private final ImFloat focalLengthYPixels = new ImFloat(0);
   private final ImFloat principalOffsetXPixels = new ImFloat(0);
   private final ImFloat principalOffsetYPixels = new ImFloat(0);
   private final ImBoolean earlyGaussianBlur = new ImBoolean(true);
   private final ImBoolean useFilteredImage = new ImBoolean(true);
   private final ImBoolean useSVDNormals = new ImBoolean(true);
   private final ImInt svdReductionFactor = new ImInt(20);
   private final ImInt gaussianSize = new ImInt(6);
   private final ImFloat gaussianSigma = new ImFloat(4.74f);
   private final ImInt searchDepthLimit = new ImInt(37000);
   private final ImInt regionMinPatches = new ImInt(37);
   private final ImInt boundaryMinPatches = new ImInt(20);
   private final ImBoolean drawPatches = new ImBoolean(true);
   private final ImBoolean drawBoundaries = new ImBoolean(true);
   private final ImBoolean render3DPlanarRegions = new ImBoolean(true);
   private final ImBoolean render3DBoundaries = new ImBoolean(true);
   private final ImBoolean render3DGrownBoundaries = new ImBoolean(true);
   private final ImFloat regionGrowthFactor = new ImFloat(0.051f);
   private final ImFloat edgeLengthTresholdSlider = new ImFloat(0.224f);
   private final ImFloat triangulationToleranceSlider = new ImFloat(0.0f);
   private final ImInt maxNumberOfIterationsSlider = new ImInt(5000);
   private final ImBoolean allowSplittingConcaveHullChecked = new ImBoolean(false);
   private final ImBoolean removeAllTrianglesWithTwoBorderEdgesChecked = new ImBoolean(false);
   private final ImFloat concaveHullThresholdSlider = new ImFloat(0.15f);
   private final ImFloat shallowAngleThresholdSlider = new ImFloat((float) Math.toRadians(1.0));
   private final ImFloat peakAngleThresholdSlider = new ImFloat((float) Math.toRadians(170.0));
   private final ImFloat lengthThresholdSlider = new ImFloat(0.05f);
   private final ImFloat depthThresholdSlider = new ImFloat(0.10f);
   private final ImInt minNumberOfNodesSlider = new ImInt(10);
   private final ImBoolean cutNarrowPassageChecked = new ImBoolean(true);
   private ImGuiPlot numberOfPlanarRegionsPlot;
   private ImGuiPlot regionMaxSearchDepthPlot;
   private ImGuiPlot numberOfBoundaryVerticesPlot;
   private ImGuiPlot boundaryMaxSearchDepthPlot;
   private ImGuiPlot svdDurationPlot;
   private ImGuiPlot wholeAlgorithmDurationPlot;
   private ImGuiPlot gpuDurationPlot;
   private ImGuiPlot depthFirstSearchDurationPlot;
   private ImGuiPlot planarRegionsSegmentationDurationPlot;
   private final Stopwatch wholeAlgorithmDurationStopwatch = new Stopwatch();
   private final Stopwatch gpuDurationStopwatch = new Stopwatch();
   private final Stopwatch depthFirstSearchDurationStopwatch = new Stopwatch();
   private final Stopwatch planarRegionsSegmentationDurationStopwatch = new Stopwatch();
   private GDXBytedecoImage inputFloatDepthImage;
   private GDXBytedecoImage inputScaledFloatDepthImage;
   private GDXBytedecoImage inputU16DepthImage;
   private GDXBytedecoImage blurredDepthImage;
   private GDXBytedecoImage filteredDepthImage;
   private GDXBytedecoImage nxImage;
   private GDXBytedecoImage nyImage;
   private GDXBytedecoImage nzImage;
   private GDXBytedecoImage cxImage;
   private GDXBytedecoImage cyImage;
   private GDXBytedecoImage czImage;
   private GDXBytedecoImage graphImage;
   private ImGuiPanel imguiPanel;
   private GDXCVImagePanel blurredDepthPanel;
   private GDXCVImagePanel filteredDepthPanel;
   private GDXCVImagePanel nxImagePanel;
   private GDXCVImagePanel nyImagePanel;
   private GDXCVImagePanel nzImagePanel;
   private GDXCVImagePanel gxImagePanel;
   private GDXCVImagePanel gyImagePanel;
   private GDXCVImagePanel gzImagePanel;
   private BMatrixRMaj regionVisitedMatrix;
   private BMatrixRMaj boundaryVisitedMatrix;
   private BMatrixRMaj boundaryMatrix;
   private DMatrixRMaj regionMatrix;
   private GDXCVImagePanel debugExtractionPanel;
   private boolean patchSizeChanged = false;
   private int numberOfRegionPatches = 0;
   private int regionMaxSearchDepth = 0;
   private int boundaryMaxSearchDepth = 0;
   private int numberOfBoundaryPatchesInWholeImage = 0;
   private double maxSVDSolveTime = Double.NaN;
   private final int[] adjacentY = {-1, 0, 1, 1, 1, 0, -1, -1};
   private final int[] adjacentX = {-1, -1, -1, 0, 1, 1, 1, 0};
   private final RecyclingArrayList<GDXGPUPlanarRegion> planarRegions = new RecyclingArrayList<>(GDXGPUPlanarRegion::new);
   private final Comparator<GDXGPURegionRing> boundaryIndexComparator = Comparator.comparingInt(regionRing -> regionRing.getBoundaryIndices().size());
   private int imageWidth;
   private int imageHeight;
   private Size gaussianKernelSize;
   private final OpenCLManager openCLManager = new OpenCLManager();
   private OpenCLFloatBuffer parametersBuffer;
   private _cl_program planarRegionExtractionProgram;
   private _cl_kernel filterKernel;
   private _cl_kernel packKernel;
   private _cl_kernel mergeKernel;
   private int patchImageHeight;
   private int patchImageWidth;
   private int patchHeight;
   private int patchWidth;
   private int filterPatchImageHeight;
   private int filterPatchImageWidth;
   private final Mat BLACK_OPAQUE_RGBA8888 = new Mat((byte) 0, (byte) 0, (byte) 0, (byte) 255);
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private GDXPlanarRegionsGraphic planarRegionsGraphic;
   private final PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private GDXPointCloudRenderer boundaryPointCloud;
   private boolean firstRun = true;

   public void create(int imageWidth, int imageHeight, ByteBuffer sourceDepthByteBufferOfFloats, double fx, double fy, double cx, double cy)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      focalLengthXPixels.set((float) fx);
      focalLengthYPixels.set((float) fy);
      principalOffsetXPixels.set((float) cx);
      principalOffsetYPixels.set((float) cy);

      parametersBuffer = new OpenCLFloatBuffer(16);
      calculateDerivativeParameters();
      inputFloatDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1, sourceDepthByteBufferOfFloats);
      inputScaledFloatDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1);
      inputU16DepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      blurredDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      filteredDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      nxImage = new GDXBytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      nyImage = new GDXBytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      nzImage = new GDXBytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      cxImage = new GDXBytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      cyImage = new GDXBytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      czImage = new GDXBytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      graphImage = new GDXBytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_8UC1);
      gaussianKernelSize = new Size();

      imguiPanel = new ImGuiPanel("GPU Planar Region Extraction", this::renderImGuiWidgets);
      blurredDepthPanel = new GDXCVImagePanel("Blurred Depth", imageWidth, imageHeight);
      filteredDepthPanel = new GDXCVImagePanel("Filtered Depth", imageWidth, imageHeight);
      nxImagePanel = new GDXCVImagePanel("Nx Image", patchImageWidth, patchImageHeight);
      nyImagePanel = new GDXCVImagePanel("Ny Image", patchImageWidth, patchImageHeight);
      nzImagePanel = new GDXCVImagePanel("Nz Image", patchImageWidth, patchImageHeight);
      gxImagePanel = new GDXCVImagePanel("Gx Image", patchImageWidth, patchImageHeight);
      gyImagePanel = new GDXCVImagePanel("Gy Image", patchImageWidth, patchImageHeight);
      gzImagePanel = new GDXCVImagePanel("Gz Image", patchImageWidth, patchImageHeight);
      debugExtractionPanel = new GDXCVImagePanel("Planar Region Extraction Image", patchImageWidth, patchImageHeight);
      imguiPanel.addChild(blurredDepthPanel.getVideoPanel());
      imguiPanel.addChild(filteredDepthPanel.getVideoPanel());
      imguiPanel.addChild(nxImagePanel.getVideoPanel());
      imguiPanel.addChild(nyImagePanel.getVideoPanel());
      imguiPanel.addChild(nzImagePanel.getVideoPanel());
      imguiPanel.addChild(gxImagePanel.getVideoPanel());
      imguiPanel.addChild(gyImagePanel.getVideoPanel());
      imguiPanel.addChild(gzImagePanel.getVideoPanel());
      imguiPanel.addChild(debugExtractionPanel.getVideoPanel());

      openCLManager.create();
      planarRegionExtractionProgram = openCLManager.loadProgram("PlanarRegionExtraction");
      filterKernel = openCLManager.createKernel(planarRegionExtractionProgram, "filterKernel");
      packKernel = openCLManager.createKernel(planarRegionExtractionProgram, "packKernel");
      mergeKernel = openCLManager.createKernel(planarRegionExtractionProgram, "mergeKernel");

      regionVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      regionMatrix = new DMatrixRMaj(patchImageHeight, patchImageWidth);

      numberOfPlanarRegionsPlot = new ImGuiPlot(labels.get("Number of planar regions"), 1000, 300, 50);
      regionMaxSearchDepthPlot = new ImGuiPlot(labels.get("Regions max search depth"), 1000, 300, 50);
      numberOfBoundaryVerticesPlot = new ImGuiPlot(labels.get("Number of boundary vertices"), 1000, 300, 50);
      boundaryMaxSearchDepthPlot = new ImGuiPlot(labels.get("Boundary max search depth"), 1000, 300, 50);
      svdDurationPlot = new ImGuiPlot(labels.get("SVD duration"), 1000, 300, 50);
      wholeAlgorithmDurationPlot = new ImGuiPlot(labels.get("Whole algorithm duration"), 1000, 300, 50);
      gpuDurationPlot = new ImGuiPlot(labels.get("GPU processing duration"), 1000, 300, 50);
      depthFirstSearchDurationPlot = new ImGuiPlot(labels.get("Depth first searching duration"), 1000, 300, 50);
      planarRegionsSegmentationDurationPlot = new ImGuiPlot(labels.get("Planar region segmentation duration"), 1000, 300, 50);

      planarRegionsGraphic = new GDXPlanarRegionsGraphic();
      boundaryPointCloud = new GDXPointCloudRenderer();
      boundaryPointCloud.create(2000000);
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

   public void extractPlanarRegions(ReferenceFrame cameraFrame)
   {
      if (!enabled.get())
         return;

      wholeAlgorithmDurationStopwatch.start();
      gpuDurationStopwatch.start();

      calculateDerivativeParameters();

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

      parametersBuffer.getBytedecoFloatBufferPointer().put(0, filterDisparityThreshold.get());
      parametersBuffer.getBytedecoFloatBufferPointer().put(1, mergeAngularThreshold.get());
      parametersBuffer.getBytedecoFloatBufferPointer().put(2, mergeDistanceThreshold.get());
      parametersBuffer.getBytedecoFloatBufferPointer().put(3, patchHeight);
      parametersBuffer.getBytedecoFloatBufferPointer().put(4, patchWidth);
      parametersBuffer.getBytedecoFloatBufferPointer().put(5, patchImageHeight);
      parametersBuffer.getBytedecoFloatBufferPointer().put(6, patchImageWidth);
      parametersBuffer.getBytedecoFloatBufferPointer().put(7, focalLengthXPixels.get());
      parametersBuffer.getBytedecoFloatBufferPointer().put(8, focalLengthYPixels.get());
      parametersBuffer.getBytedecoFloatBufferPointer().put(9, principalOffsetXPixels.get());
      parametersBuffer.getBytedecoFloatBufferPointer().put(10, principalOffsetYPixels.get());
      parametersBuffer.getBytedecoFloatBufferPointer().put(11, deadPixelFilterPatchSize.get());
      parametersBuffer.getBytedecoFloatBufferPointer().put(12, filterPatchImageHeight);
      parametersBuffer.getBytedecoFloatBufferPointer().put(13, filterPatchImageWidth);
      parametersBuffer.getBytedecoFloatBufferPointer().put(14, imageHeight);
      parametersBuffer.getBytedecoFloatBufferPointer().put(15, imageWidth);

      if (patchSizeChanged)
      {
         patchSizeChanged = false;
         nxImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         nyImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         nzImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         cxImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         cyImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         czImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         graphImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         nxImagePanel.resize(patchImageWidth, patchImageHeight, openCLManager);
         nyImagePanel.resize(patchImageWidth, patchImageHeight, openCLManager);
         nzImagePanel.resize(patchImageWidth, patchImageHeight, openCLManager);
         gxImagePanel.resize(patchImageWidth, patchImageHeight, openCLManager);
         gyImagePanel.resize(patchImageWidth, patchImageHeight, openCLManager);
         gzImagePanel.resize(patchImageWidth, patchImageHeight, openCLManager);
         debugExtractionPanel.resize(patchImageWidth, patchImageHeight, openCLManager);
         regionVisitedMatrix.reshape(patchImageHeight, patchImageWidth);
         boundaryVisitedMatrix.reshape(patchImageHeight, patchImageWidth);
         boundaryMatrix.reshape(patchImageHeight, patchImageWidth);
         regionMatrix.reshape(patchImageHeight, patchImageWidth);
      }
      if (firstRun)
      {
         firstRun = false;
         inputU16DepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
         blurredDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
         filteredDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         nxImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         nyImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         nzImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         cxImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         cyImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         czImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         graphImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         parametersBuffer.createOpenCLBufferObject(openCLManager);
      }
      else
      {
         inputU16DepthImage.writeOpenCLImage(openCLManager);
         blurredDepthImage.writeOpenCLImage(openCLManager);
         parametersBuffer.writeOpenCLBufferObject(openCLManager);
      }

      _cl_mem inputImage = earlyGaussianBlur.get() ? blurredDepthImage.getOpenCLImageObject() : inputU16DepthImage.getOpenCLImageObject();

      openCLManager.setKernelArgument(filterKernel, 0, inputImage);
      openCLManager.setKernelArgument(filterKernel, 1, filteredDepthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(filterKernel, 2, nxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(filterKernel, 3, parametersBuffer.getOpenCLBufferObject());

      _cl_mem packKernelInputObject = useFilteredImage.get() ? filteredDepthImage.getOpenCLImageObject() : inputU16DepthImage.getOpenCLImageObject();
      openCLManager.setKernelArgument(packKernel, 0, packKernelInputObject);
      openCLManager.setKernelArgument(packKernel, 1, nxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 2, nyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 3, nzImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 4, cxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 5, cyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 6, czImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 7, parametersBuffer.getOpenCLBufferObject());

      openCLManager.setKernelArgument(mergeKernel, 0, nxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 1, nyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 2, nzImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 3, cxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 4, cyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 5, czImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 6, graphImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 7, parametersBuffer.getOpenCLBufferObject());

      openCLManager.execute2D(filterKernel, filterPatchImageHeight, filterPatchImageWidth); // X & Y vs height and width are flipped in this kernel code
      openCLManager.execute2D(packKernel, patchImageHeight, patchImageWidth);
      openCLManager.execute2D(mergeKernel, patchImageHeight, patchImageWidth);

      openCLManager.enqueueReadImage(filteredDepthImage.getOpenCLImageObject(), imageWidth, imageHeight, filteredDepthImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(nxImage.getOpenCLImageObject(), patchImageWidth, patchImageHeight, nxImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(nyImage.getOpenCLImageObject(), patchImageWidth, patchImageHeight, nyImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(nzImage.getOpenCLImageObject(), patchImageWidth, patchImageHeight, nzImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(cxImage.getOpenCLImageObject(), patchImageWidth, patchImageHeight, cxImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(cyImage.getOpenCLImageObject(), patchImageWidth, patchImageHeight, cyImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(czImage.getOpenCLImageObject(), patchImageWidth, patchImageHeight, czImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(graphImage.getOpenCLImageObject(), patchImageWidth, patchImageHeight, graphImage.getBytedecoByteBufferPointer());

      openCLManager.finish();
      gpuDurationStopwatch.suspend();

      if (debugExtractionPanel.getVideoPanel().getIsShowing().get() && (drawPatches.get() || drawBoundaries.get()))
         debugExtractionPanel.getBytedecoImage().getBytedecoOpenCVMat().setTo(BLACK_OPAQUE_RGBA8888);

      depthFirstSearchDurationStopwatch.start();
      findRegions();
      findBoundariesAndHoles();
      growRegionBoundaries();
      depthFirstSearchDurationStopwatch.suspend();

      planarRegionsSegmentationDurationStopwatch.start();
      computePlanarRegions(cameraFrame);
      planarRegionsSegmentationDurationStopwatch.suspend();
      wholeAlgorithmDurationStopwatch.suspend();

      render2DPanels();
      renderPlanarRegions();
      renderBoundaryPoints(cameraFrame);
   }

   private void findRegions()
   {
      int planarRegionIslandIndex = 0;
      regionMaxSearchDepth = 0;
      planarRegions.clear();
      regionVisitedMatrix.zero();
      boundaryMatrix.zero();
      regionMatrix.zero();
      maxSVDSolveTime = 0.0;
      for (int row = 0; row < patchImageHeight; row++)
      {
         for (int column = 0; column < patchImageWidth; column++)
         {
            int boundaryConnectionsEncodedAsOnes = Byte.toUnsignedInt(graphImage.getBytedecoOpenCVMat().ptr(row, column).get());
            if (!regionVisitedMatrix.get(row, column) && boundaryConnectionsEncodedAsOnes == 255) // all ones; fully connected
            {
               numberOfRegionPatches = 0; // also number of patches traversed
               GDXGPUPlanarRegion planarRegion = planarRegions.add();
               planarRegion.reset(planarRegionIslandIndex);
               regionsDepthFirstSearch(row, column, planarRegionIslandIndex, planarRegion, 1);
               if (numberOfRegionPatches >= regionMinPatches.get())
               {
                  ++planarRegionIslandIndex;
                  planarRegion.update(useSVDNormals.get(), svdReductionFactor.get());
                  if (planarRegion.getSVDDuration() > maxSVDSolveTime)
                     maxSVDSolveTime = planarRegion.getSVDDuration();

                  if (debugExtractionPanel.getVideoPanel().getIsShowing().get() && drawPatches.get())
                  {
                     for (Point2D regionIndex : planarRegion.getRegionIndices())
                     {
                        int x = (int) regionIndex.getX();
                        int y = (int) regionIndex.getY();
                        int r = (planarRegionIslandIndex + 1) * 312 % 255;
                        int g = (planarRegionIslandIndex + 1) * 123 % 255;
                        int b = (planarRegionIslandIndex + 1) * 231 % 255;
                        BytePointer pixel = debugExtractionPanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(y, x);
                        pixel.put(0, (byte) r);
                        pixel.put(1, (byte) g);
                        pixel.put(2, (byte) b);
                     }
                  }
               }
               else
               {
                  planarRegions.remove(planarRegions.size() - 1);
               }
               if (numberOfRegionPatches > regionMaxSearchDepth)
                  regionMaxSearchDepth = numberOfRegionPatches;
            }
         }
      }
   }

   private void regionsDepthFirstSearch(int row, int column, int planarRegionIslandIndex, GDXGPUPlanarRegion planarRegion, int searchDepth)
   {
      if (regionVisitedMatrix.get(row, column) || searchDepth > searchDepthLimit.get())
         return;

      if (searchDepth > regionMaxSearchDepth)
         regionMaxSearchDepth = searchDepth;

      ++numberOfRegionPatches;
      regionVisitedMatrix.set(row, column, true);
      regionMatrix.set(row, column, planarRegionIslandIndex);
      // kernel coordinates is in left-handed frame, so lets flip it to IHMC Z up
      float ny = -nxImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float nz = nyImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float nx = nzImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float cy = -cxImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float cz = cyImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float cx = czImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      planarRegion.addRegionPatch(row, column, nx, ny, nz, cx, cy, cz);

      int count = 0;
      for (int i = 0; i < 8; i++)
      {
         if (row + adjacentY[i] < patchImageHeight - 1 && row + adjacentY[i] > 1 && column + adjacentX[i] < patchImageWidth - 1 && column + adjacentX[i] > 1)
         {
            int boundaryConnectionsEncodedAsOnes
                  = Byte.toUnsignedInt(graphImage.getBytedecoOpenCVMat().ptr((row + adjacentY[i]), (column + adjacentX[i])).get());
            if (boundaryConnectionsEncodedAsOnes == 255) // all ones; fully connected
            {
               ++count;
               regionsDepthFirstSearch(row + adjacentY[i], column + adjacentX[i], planarRegionIslandIndex, planarRegion, searchDepth + 1);
            }
         }
      }
      if (count != 8)
      {
         boundaryMatrix.set(row, column, true);
         planarRegion.getBorderIndices().add().set(column, row);
      }
   }

   private void findBoundariesAndHoles()
   {
      boundaryVisitedMatrix.zero();
      numberOfBoundaryPatchesInWholeImage = 0;
      boundaryMaxSearchDepth = 0;
      planarRegions.parallelStream().forEach(planarRegion ->
      {
         int leafPatchIndex = 0;
         int regionRingIndex = 0;
         for (Point2D leafPatch : planarRegion.getBorderIndices())
         {
            GDXGPURegionRing regionRing = planarRegion.getRegionRings().add();
            regionRing.reset();
            int numberOfBoundaryPatches = boundaryDepthFirstSearch((int) leafPatch.getY(),
                                                                   (int) leafPatch.getX(),
                                                                   planarRegion.getId(),
                                                                   regionRing,
                                                                   leafPatchIndex,
                                                                   1);
            if (numberOfBoundaryPatches >= boundaryMinPatches.get())
            {
               if (debugExtractionPanel.getVideoPanel().getIsShowing().get() && drawBoundaries.get())
               {
                  for (Vector2D boundaryIndex : regionRing.getBoundaryIndices())
                  {
                     int x = (int) boundaryIndex.getX();
                     int y = (int) boundaryIndex.getY();
                     int r = (regionRingIndex + 1) * 130 % 255;
                     int g = (regionRingIndex + 1) * 227 % 255;
                     int b = (regionRingIndex + 1) * 332 % 255;
                     BytePointer pixel = debugExtractionPanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(y, x);
                     pixel.put(0, (byte) r);
                     pixel.put(1, (byte) g);
                     pixel.put(2, (byte) b);
                  }
               }
               ++regionRingIndex;
            }
            else
            {
               planarRegion.getRegionRings().remove(planarRegion.getRegionRings().size() - 1);
            }
            ++leafPatchIndex;
         }
         planarRegion.getRegionRings().sort(boundaryIndexComparator);
      });
   }

   private int boundaryDepthFirstSearch(int row, int column, int planarRegionId, GDXGPURegionRing regionRing, int leafPatchIndex, int searchDepth)
   {
      if (boundaryVisitedMatrix.get(row, column) || searchDepth > searchDepthLimit.get())
         return 0;

      if (searchDepth > boundaryMaxSearchDepth)
         boundaryMaxSearchDepth = searchDepth;

      ++numberOfBoundaryPatchesInWholeImage;
      boundaryVisitedMatrix.set(row, column, true);
      regionRing.getBoundaryIndices().add().set(column, row);

      int numberOfBoundaryPatches = 1;
      for (int i = 0; i < 8; i++)
      {
         if (row + adjacentY[i] < patchImageHeight - 1
          && row + adjacentY[i] > 1
          && column + adjacentX[i] < patchImageWidth - 1
          && column + adjacentX[i] > 1
          && boundaryMatrix.get(row + adjacentY[i], column + adjacentX[i])
          && planarRegionId == regionMatrix.get(row + adjacentY[i], column + adjacentX[i]))
         {
            numberOfBoundaryPatches += boundaryDepthFirstSearch(row + adjacentY[i],
                                                                column + adjacentX[i],
                                                                planarRegionId,
                                                                regionRing,
                                                                leafPatchIndex,
                                                                searchDepth + 1);
         }
      }
      return numberOfBoundaryPatches;
   }

   private void growRegionBoundaries()
   {
      planarRegions.forEach(planarRegion ->
      {
         if (!planarRegion.getRegionRings().isEmpty())
         {
            GDXGPURegionRing firstRing = planarRegion.getRegionRings().get(0);
            for (Vector2D boundaryIndex : firstRing.getBoundaryIndices())
            {
               // kernel coordinates is in left-handed frame, so lets flip it to IHMC Z up
               float vertexX = czImage.getBytedecoOpenCVMat().ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX()).getFloat();
               float vertexY = -cxImage.getBytedecoOpenCVMat().ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX()).getFloat();
               float vertexZ = cyImage.getBytedecoOpenCVMat().ptr((int) boundaryIndex.getY(), (int) boundaryIndex.getX()).getFloat();
               Vector3D boundaryVertex = planarRegion.getBoundaryVertices().add();
               boundaryVertex.set(vertexX, vertexY, vertexZ);
               boundaryVertex.sub(planarRegion.getCenter());
               boundaryVertex.normalize();
               boundaryVertex.scale(regionGrowthFactor.get());
               boundaryVertex.add(vertexX, vertexY, vertexZ);
            }
         }
      });
   }

   private void computePlanarRegions(ReferenceFrame cameraFrame)
   {
      List<List<PlanarRegion>> listOfListsOfRegions = planarRegions.parallelStream()
         .filter(gpuPlanarRegion -> gpuPlanarRegion.getBoundaryVertices().size() >= polygonizerParameters.getMinNumberOfNodes())
         .map(gpuPlanarRegion ->
         {
            List<PlanarRegion> planarRegions = new ArrayList<>();
            try
            {
               FrameQuaternion orientation = new FrameQuaternion();
               orientation.setIncludingFrame(cameraFrame, EuclidGeometryTools.axisAngleFromZUpToVector3D(gpuPlanarRegion.getNormal()));
               orientation.changeFrame(ReferenceFrame.getWorldFrame());

               // First compute the set of concave hulls for this region
               FramePoint3D origin = new FramePoint3D(cameraFrame, gpuPlanarRegion.getCenter());
               origin.changeFrame(ReferenceFrame.getWorldFrame());

               List<Point2D> pointCloudInPlane = gpuPlanarRegion.getBoundaryVertices().stream()
                  .map(boundaryVertex ->
                  {
                     FramePoint3D framePoint3D = new FramePoint3D(cameraFrame, boundaryVertex);
                     framePoint3D.changeFrame(ReferenceFrame.getWorldFrame());
                     return PolygonizerTools.toPointInPlane(framePoint3D, origin, orientation);
                  })
                  .filter(point2D -> Double.isFinite(point2D.getX()) && Double.isFinite(point2D.getY()))
                  .collect(Collectors.toList());
               List<LineSegment2D> intersections = new ArrayList<>();
//                     = intersections.stream()
//                  .map(intersection -> PolygonizerTools.toLineSegmentInPlane(lineSegmentInWorld, origin, orientation))
//                  .collect(Collectors.toList());
               ConcaveHullCollection concaveHullCollection = SimpleConcaveHullFactory.createConcaveHullCollection(pointCloudInPlane,
                                                                                                                  intersections,
                                                                                                                  concaveHullFactoryParameters);

               // Apply some simple filtering to reduce the number of vertices and hopefully the number of convex polygons.
               double shallowAngleThreshold = polygonizerParameters.getShallowAngleThreshold();
               double peakAngleThreshold = polygonizerParameters.getPeakAngleThreshold();
               double lengthThreshold = polygonizerParameters.getLengthThreshold();

               ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullCollection);
               ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHullCollection);
               if (polygonizerParameters.getCutNarrowPassage())
                  concaveHullCollection = ConcaveHullPruningFilteringTools.concaveHullNarrowPassageCutter(lengthThreshold, concaveHullCollection);

               int hullCounter = 0;
               int regionId = gpuPlanarRegion.getId();

               for (ConcaveHull concaveHull : concaveHullCollection)
               {
                  if (concaveHull.isEmpty())
                     continue;

                  // Decompose the concave hulls into convex polygons
                  double depthThreshold = polygonizerParameters.getDepthThreshold();
                  List<ConvexPolygon2D> decomposedPolygons = new ArrayList<>();
                  ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHull, depthThreshold, decomposedPolygons);

                  // Pack the data in PlanarRegion
                  FramePose3D regionPose = new FramePose3D();
                  regionPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), origin, orientation);
                  RigidBodyTransform tempTransform = new RigidBodyTransform();
                  regionPose.get(tempTransform);
                  PlanarRegion planarRegion = new PlanarRegion(tempTransform,
                                                               concaveHull.getConcaveHullVertices(),
                                                               decomposedPolygons);
                  planarRegion.setRegionId(regionId);
                  planarRegions.add(planarRegion);

                  hullCounter++;
                  regionId = 31 * regionId + hullCounter;
               }
            }
            catch (RuntimeException e)
            {
               e.printStackTrace();
            }
            return planarRegions;
         })
         .collect(Collectors.toList());
      planarRegionsList.clear();
      for (List<PlanarRegion> planarRegions : listOfListsOfRegions)
      {
         planarRegionsList.addPlanarRegions(planarRegions);
      }
   }

   private void render2DPanels()
   {
      blurredDepthPanel.drawFloatImage(blurredDepthImage.getBytedecoOpenCVMat());
      filteredDepthPanel.drawFloatImage(filteredDepthImage.getBytedecoOpenCVMat());
      nxImagePanel.drawFloatImage(nxImage.getBytedecoOpenCVMat());
      nyImagePanel.drawFloatImage(nyImage.getBytedecoOpenCVMat());
      nzImagePanel.drawFloatImage(nzImage.getBytedecoOpenCVMat());
      gxImagePanel.drawFloatImage(cxImage.getBytedecoOpenCVMat());
      gyImagePanel.drawFloatImage(cyImage.getBytedecoOpenCVMat());
      gzImagePanel.drawFloatImage(czImage.getBytedecoOpenCVMat());
      debugExtractionPanel.draw();
   }

   /** FIXME: This method filled with allocations. */
   private void renderPlanarRegions()
   {
      if (!render3DPlanarRegions.get())
         return;

      planarRegionsGraphic.generateMeshes(planarRegionsList);
      planarRegionsGraphic.update();
   }

   private void renderBoundaryPoints(ReferenceFrame cameraFrame)
   {
      if (render3DBoundaries.get() || render3DGrownBoundaries.get())
      {
         boundaryPointCloud.prepareVertexBufferForAddingPoints();
         for (GDXGPUPlanarRegion planarRegion : planarRegions)
         {
            if (render3DBoundaries.get())
            {
               if (!planarRegion.getRegionRings().isEmpty())
               {
                  GDXGPURegionRing firstRing = planarRegion.getRegionRings().get(0);
                  for (Vector2D boundaryIndex : firstRing.getBoundaryIndices())
                  {
                     int column = (int) boundaryIndex.getX() * patchWidth;
                     int row = (int) boundaryIndex.getY() * patchHeight;
                     float z = inputFloatDepthImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
                     tempFramePoint.setIncludingFrame(cameraFrame, column, row, z);
                     ProjectionTools.projectDepthPixelToIHMCZUp3D(tempFramePoint,
                                                                  principalOffsetXPixels.get(),
                                                                  principalOffsetYPixels.get(),
                                                                  focalLengthXPixels.get(),
                                                                  focalLengthYPixels.get());
                     tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
                     boundaryPointCloud.putVertex(tempFramePoint);
                  }
               }
            }

            if (render3DGrownBoundaries.get())
            {
               for (Vector3D boundaryVertex : planarRegion.getBoundaryVertices())
               {
                  tempFramePoint.setIncludingFrame(cameraFrame, boundaryVertex);
                  tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
                  boundaryPointCloud.putVertex(tempFramePoint);
               }
            }
         }

         boundaryPointCloud.updateMeshFastest();
      }
   }

   private void calculateDerivativeParameters()
   {
      patchHeight = patchSize.get();
      patchWidth = patchSize.get();
      patchImageHeight = imageHeight / patchHeight;
      patchImageWidth = imageWidth / patchWidth;
      filterPatchImageHeight = imageHeight / deadPixelFilterPatchSize.get();
      filterPatchImageWidth = imageWidth / deadPixelFilterPatchSize.get();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Input image dimensions: " + imageWidth + " x " + imageHeight);
      ImGui.checkbox(labels.get("Enabled"), enabled);
      wholeAlgorithmDurationPlot.render(wholeAlgorithmDurationStopwatch.totalElapsed());
      gpuDurationPlot.render(gpuDurationStopwatch.totalElapsed());
      depthFirstSearchDurationPlot.render(depthFirstSearchDurationStopwatch.totalElapsed());
      planarRegionsSegmentationDurationPlot.render(planarRegionsSegmentationDurationStopwatch.totalElapsed());
      numberOfPlanarRegionsPlot.render((float) planarRegions.size());
      regionMaxSearchDepthPlot.render((float) regionMaxSearchDepth);
      numberOfBoundaryVerticesPlot.render((float) numberOfBoundaryPatchesInWholeImage);
      boundaryMaxSearchDepthPlot.render((float) boundaryMaxSearchDepth);

      ImGui.checkbox(labels.get("Early gaussian blur"), earlyGaussianBlur);
      ImGui.sliderInt(labels.get("Gaussian size"), gaussianSize.getData(), 1, 20);
      ImGui.sliderFloat(labels.get("Gaussian sigma"), gaussianSigma.getData(), 0.23f, 10.0f);
      if (ImGui.sliderInt(labels.get("Patch size"), desiredPatchSize.getData(), 2, 20))
      {
         if (desiredPatchSize.get() != patchSize.get()
          && imageWidth % desiredPatchSize.get() == 0
          && imageHeight % desiredPatchSize.get() == 0)
         {
            patchSize.set(desiredPatchSize.get());
            patchSizeChanged = true;
         }
      }
      ImGui.sliderInt(labels.get("Dead pixel filter patch size"), deadPixelFilterPatchSize.getData(), 1, 20);
      ImGui.checkbox(labels.get("Use filtered image"), useFilteredImage);
      ImGui.sliderFloat(labels.get("Merge distance threshold"), mergeDistanceThreshold.getData(), 0.0f, 0.1f);
      ImGui.sliderFloat(labels.get("Merge angular threshold"), mergeAngularThreshold.getData(), 0.0f, 1.0f);
      ImGui.sliderInt(labels.get("Search depth limit"), searchDepthLimit.getData(), 1, 50000);
      ImGui.sliderInt(labels.get("Region min patches"), regionMinPatches.getData(), 1, 1000);
      ImGui.sliderInt(labels.get("Boundary min patches"), boundaryMinPatches.getData(), 1, 1000);
      ImGui.sliderFloat(labels.get("Region growth factor"), regionGrowthFactor.getData(), 0.005f, 0.1f);
      ImGui.checkbox(labels.get("Use SVD normals"), useSVDNormals);
      ImGui.sliderInt(labels.get("SVD reduction factor"), svdReductionFactor.getData(), 1, 100);
      svdDurationPlot.render((float) maxSVDSolveTime);
      ImGui.checkbox(labels.get("Draw patches"), drawPatches);
      ImGui.checkbox(labels.get("Draw boundaries"), drawBoundaries);
      ImGui.checkbox(labels.get("Render 3D planar regions"), render3DPlanarRegions);
      ImGui.checkbox(labels.get("Render 3D boundaries"), render3DBoundaries);
      ImGui.checkbox(labels.get("Render 3D grown boundaries"), render3DGrownBoundaries);
      ImGui.sliderFloat(labels.get("Focal length X (px)"), focalLengthXPixels.getData(), -1000.0f, 1000.0f);
      ImGui.sliderFloat(labels.get("Focal length Y (px)"), focalLengthYPixels.getData(), -1000.0f, 1000.0f);
      ImGui.sliderFloat(labels.get("Principal offset X (px)"), principalOffsetXPixels.getData(), -imageWidth, imageWidth);
      ImGui.sliderFloat(labels.get("Principal offset Y (px)"), principalOffsetYPixels.getData(), -imageHeight, imageHeight);

      ImGui.sliderFloat("Edge Length Threshold", edgeLengthTresholdSlider.getData(), 0, 0.5f);
      concaveHullFactoryParameters.setEdgeLengthThreshold(edgeLengthTresholdSlider.get());
      ImGui.sliderFloat("Triangulation Tolerance", triangulationToleranceSlider.getData(), 0, 0.3f);
      concaveHullFactoryParameters.setTriangulationTolerance(triangulationToleranceSlider.get());
      ImGui.sliderInt("Max Number of Iterations", maxNumberOfIterationsSlider.getData(), 2000, 6000);
      concaveHullFactoryParameters.setMaxNumberOfIterations(maxNumberOfIterationsSlider.get());
      ImGui.checkbox("Remove Degenerate Triangles", removeAllTrianglesWithTwoBorderEdgesChecked);
      concaveHullFactoryParameters.setRemoveAllTrianglesWithTwoBorderEdges(removeAllTrianglesWithTwoBorderEdgesChecked.get());
      ImGui.checkbox("Split Concave Hull", allowSplittingConcaveHullChecked);
      concaveHullFactoryParameters.setAllowSplittingConcaveHull(allowSplittingConcaveHullChecked.get());

      ImGui.sliderFloat("Concave Hull Threshold", concaveHullThresholdSlider.getData(), 0, 1);
      polygonizerParameters.setConcaveHullThreshold(concaveHullThresholdSlider.get());
      ImGui.sliderFloat("Shallow Angle Threshold", shallowAngleThresholdSlider.getData(), 0, 2.0f * (float) Math.PI);
      polygonizerParameters.setShallowAngleThreshold(shallowAngleThresholdSlider.get());
      ImGui.sliderFloat("Peak Angle Threshold", peakAngleThresholdSlider.getData(), 0, 2.0f * (float) Math.PI);
      polygonizerParameters.setPeakAngleThreshold(peakAngleThresholdSlider.get());
      ImGui.sliderFloat("Length Threshold", lengthThresholdSlider.getData(), 0, 1);
      polygonizerParameters.setLengthThreshold(lengthThresholdSlider.get());
      ImGui.sliderFloat("Depth Threshold", depthThresholdSlider.getData(), 0, 1);
      polygonizerParameters.setDepthThreshold(depthThresholdSlider.get());
      ImGui.sliderInt("Min Number of Nodes", minNumberOfNodesSlider.getData(), 0, 100);
      polygonizerParameters.setMinNumberOfNodes(minNumberOfNodesSlider.get());
      ImGui.checkbox("Cut Narrow Passage", cutNarrowPassageChecked);
      polygonizerParameters.setCutNarrowPassage(cutNarrowPassageChecked.get());
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (render3DPlanarRegions.get())
         planarRegionsGraphic.getRenderables(renderables, pool);
      if (render3DGrownBoundaries.get() || render3DBoundaries.get())
         boundaryPointCloud.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      openCLManager.destroy();
      // TODO: Destroy the rest
   }

   public ImGuiPanel getPanel()
   {
      return imguiPanel;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public ImBoolean getEnabled()
   {
      return enabled;
   }

   public ImBoolean getDrawBoundaries()
   {
      return drawBoundaries;
   }

   public ImBoolean getDrawPatches()
   {
      return drawPatches;
   }

   public ImBoolean getRender3DPlanarRegions()
   {
      return render3DPlanarRegions;
   }

   public ImBoolean getRender3DBoundaries()
   {
      return render3DBoundaries;
   }

   public ImBoolean getRender3DGrownBoundaries()
   {
      return render3DGrownBoundaries;
   }
}
