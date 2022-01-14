package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
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
import org.bytedeco.opencv.opencv_core.*;
import org.ejml.data.BMatrixRMaj;
import org.ejml.data.DMatrixRMaj;
import sensor_msgs.Image;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.gdx.GDXPointCloudRenderer;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
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
   private final ImFloat mergeDistanceThreshold = new ImFloat(0.016f);
   private final ImFloat mergeAngularThreshold = new ImFloat(0.82f);
   private final ImFloat filterDisparityThreshold = new ImFloat(2000);
   private final ImInt inputHeight = new ImInt(0);
   private final ImInt inputWidth = new ImInt(0);
   private final ImInt patchSize = new ImInt(4);
   private final ImInt deadPixelFilterPatchSize = new ImInt(4);
   private final ImFloat focalLengthXPixels = new ImFloat(0);
   private final ImFloat focalLengthYPixels = new ImFloat(0);
   private final ImFloat principalOffsetXPixels = new ImFloat(0);
   private final ImFloat principalOffsetYPixels = new ImFloat(0);
   private final ImBoolean earlyGaussianBlur = new ImBoolean(true);
   private final ImBoolean useFilteredImage = new ImBoolean(true);
   private final ImInt gaussianSize = new ImInt(6);
   private final ImInt gaussianSigma = new ImInt(20);
   private final ImInt searchDepthLimit = new ImInt(10000);
   private final ImInt regionMinPatches = new ImInt(20);
   private final ImInt regionBoundaryDiff = new ImInt(20);
   private final ImBoolean drawPatches = new ImBoolean(true);
   private final ImBoolean drawBoundaries = new ImBoolean(true);
   private final ImBoolean render3DPlanarRegions = new ImBoolean(false);
   private final ImBoolean render3DBoundaries = new ImBoolean(true);
   private final ImBoolean render3DGrownBoundaries = new ImBoolean(true);
   private final ImDouble regionGrowthFactor = new ImDouble(0.01);
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
   private MatVector outputChannelVector;
   private GDXBytedecoImage regionOutputImage;
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
   private final Point cvCenterPoint = new Point();
   private final Scalar cvBGRColorScalar = new Scalar();
   private int depthOfRegionsSearch = 0;
   private int regionMaxSearchDepth = 0;
   private int depthOfBoundariesSearch = 0;
   private int boundaryMaxSearchDepth = 0;
   private int numberOfBoundaryIndices = 0;
   private final int[] adjacentY = {-1, 0, 1, 1, 1, 0, -1, -1};
   private final int[] adjacentX = {-1, -1, -1, 0, 1, 1, 1, 0};
   private final RecyclingArrayList<GDXGPUPlanarRegion> planarRegions = new RecyclingArrayList<>(GDXGPUPlanarRegion::new);
   private final Comparator<GDXGPURegionRing> boundaryVertexComparator = Comparator.comparingInt(regionRing -> regionRing.getBoundaryVertices().size());
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
   private final Mat BLACK_OPAQUE_RGBA8888 = new Mat((byte) 0, (byte) 0, (byte) 0, (byte) 255);
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final FramePose3D regionPose = new FramePose3D();
   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FrameQuaternion orientation = new FrameQuaternion();
   private GDXPlanarRegionsGraphic planarRegionsGraphic;
   private final PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private GDXPointCloudRenderer boundaryPointCloud;
   private RecyclingArrayList<Point3D32> pointsToRender = new RecyclingArrayList<>(Point3D32::new);

   public void create(int imageWidth, int imageHeight, ByteBuffer sourceDepthByteBufferOfFloats, double fx, double fy, double cx, double cy)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      inputWidth.set(imageWidth);
      inputHeight.set(imageHeight);
      focalLengthXPixels.set((float) fx);
      focalLengthYPixels.set((float) fy);
      principalOffsetXPixels.set((float) cx);
      principalOffsetYPixels.set((float) cy);

      calculateDetivativeParameters();

      inputFloatDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1, sourceDepthByteBufferOfFloats);
      inputScaledFloatDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1);
      inputU16DepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      blurredDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      filteredDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      nxImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_32FC1);
      nyImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_32FC1);
      nzImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_32FC1);
      cxImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_32FC1);
      cyImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_32FC1);
      czImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_32FC1);
      outputChannelVector = new MatVector(nxImage.getBytedecoOpenCVMat(),
                                          nyImage.getBytedecoOpenCVMat(),
                                          nxImage.getBytedecoOpenCVMat(),
                                          cxImage.getBytedecoOpenCVMat(),
                                          cyImage.getBytedecoOpenCVMat(),
                                          czImage.getBytedecoOpenCVMat());
      graphImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_8UC1);
      regionOutputImage = new GDXBytedecoImage(subWidth, subHeight, opencv_core.CV_32FC(6));
      gaussianKernelSize = new Size();

      imguiPanel = new ImGuiPanel("GPU Planar Region Extraction", this::renderImGuiWidgets);
      blurredDepthPanel = new GDXCVImagePanel("Blurred Depth", imageWidth, imageHeight);
      filteredDepthPanel = new GDXCVImagePanel("Filtered Depth", imageWidth, imageHeight);
      nxImagePanel = new GDXCVImagePanel("Nx Image", subWidth, subHeight);
      nyImagePanel = new GDXCVImagePanel("Ny Image", subWidth, subHeight);
      nzImagePanel = new GDXCVImagePanel("Nz Image", subWidth, subHeight);
      gxImagePanel = new GDXCVImagePanel("Gx Image", subWidth, subHeight);
      gyImagePanel = new GDXCVImagePanel("Gy Image", subWidth, subHeight);
      gzImagePanel = new GDXCVImagePanel("Gz Image", subWidth, subHeight);
      debugExtractionPanel = new GDXCVImagePanel("Planar Region Extraction Image", imageWidth, imageHeight);
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

      regionVisitedMatrix = new BMatrixRMaj(subHeight, subWidth);
      boundaryVisitedMatrix = new BMatrixRMaj(subHeight, subWidth);
      boundaryMatrix = new BMatrixRMaj(subHeight, subWidth);
      regionMatrix = new DMatrixRMaj(subHeight, subWidth);

      numberOfPlanarRegionsPlot = new ImGuiPlot(labels.get("Number of planar regions"), 1000, 300, 50);
      regionMaxSearchDepthPlot = new ImGuiPlot(labels.get("Regions max search depth"), 1000, 300, 50);
      numberOfBoundaryVerticesPlot = new ImGuiPlot(labels.get("Number of boundary vertices"), 1000, 300, 50);
      boundaryMaxSearchDepthPlot = new ImGuiPlot(labels.get("Boundary max search depth"), 1000, 300, 50);

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

   public void extractPlanarRegions()
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
      cxImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      cyImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      czImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
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
      openCLManager.setKernelArgument(packKernel, 4, cxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 5, cyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 6, czImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(packKernel, 7, parametersBufferObject);

      openCLManager.setKernelArgument(mergeKernel, 0, nxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 1, nyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 2, nzImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 3, cxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 4, cyImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 5, czImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 6, graphImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(mergeKernel, 7, parametersBufferObject);

      openCLManager.execute2D(filterKernel, filterSubHeight, filterSubWidth); // TODO: Check X & Y vs height and width
      openCLManager.execute2D(packKernel, subHeight, subWidth);
      openCLManager.execute2D(mergeKernel, subHeight, subWidth);

      openCLManager.enqueueReadImage(filteredDepthImage.getOpenCLImageObject(), imageWidth, imageHeight, filteredDepthImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(nxImage.getOpenCLImageObject(), subWidth, subHeight, nxImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(nyImage.getOpenCLImageObject(), subWidth, subHeight, nyImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(nzImage.getOpenCLImageObject(), subWidth, subHeight, nzImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(cxImage.getOpenCLImageObject(), subWidth, subHeight, cxImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(cyImage.getOpenCLImageObject(), subWidth, subHeight, cyImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(czImage.getOpenCLImageObject(), subWidth, subHeight, czImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(graphImage.getOpenCLImageObject(), subWidth, subHeight, graphImage.getBytedecoByteBufferPointer());

      openCLManager.finish();

      filteredDepthPanel.drawFloatImage(filteredDepthImage.getBytedecoOpenCVMat());
      nxImagePanel.drawFloatImage(nxImage.getBytedecoOpenCVMat());
      nyImagePanel.drawFloatImage(nyImage.getBytedecoOpenCVMat());
      nzImagePanel.drawFloatImage(nzImage.getBytedecoOpenCVMat());
      gxImagePanel.drawFloatImage(cxImage.getBytedecoOpenCVMat());
      gyImagePanel.drawFloatImage(cyImage.getBytedecoOpenCVMat());
      gzImagePanel.drawFloatImage(czImage.getBytedecoOpenCVMat());

      opencv_core.merge(outputChannelVector, regionOutputImage.getBytedecoOpenCVMat());

      debugExtractionPanel.getBytedecoImage().getBytedecoOpenCVMat().setTo(BLACK_OPAQUE_RGBA8888);

      findRegions();
      findBoundariesAndHoles();
      growRegionBoundaries();

      debugExtractionPanel.draw();
   }

   private void findRegions()
   {
      int planarRegionIslandIndex = 0;
      regionMaxSearchDepth = 0;
      planarRegions.clear();
      regionVisitedMatrix.zero();
      boundaryMatrix.zero();
      regionMatrix.zero();
      for (int row = 0; row < subHeight; row++)
      {
         for (int column = 0; column < subWidth; column++)
         {
            int patch = Byte.toUnsignedInt(graphImage.getBytedecoOpenCVMat().ptr(row, column).get());
            if (!regionVisitedMatrix.get(row, column) && patch == 255)
            {
               depthOfRegionsSearch = 0; // also number of patches traversed
               GDXGPUPlanarRegion planarRegion = planarRegions.add();
               planarRegion.reset(planarRegionIslandIndex);
               depthFirstSearch(row, column, planarRegionIslandIndex, planarRegion);
               if (depthOfRegionsSearch > regionMinPatches.get()
                && depthOfRegionsSearch - planarRegion.getBoundaryVertices().size() > regionBoundaryDiff.get())
               {
                  ++planarRegionIslandIndex;
               }
               else
               {
                  planarRegions.remove(planarRegions.size() - 1);
               }
               if (depthOfRegionsSearch > regionMaxSearchDepth)
                  regionMaxSearchDepth = depthOfRegionsSearch;
            }
         }
      }
   }

   private void depthFirstSearch(int row, int column, int planarRegionIslandIndex, GDXGPUPlanarRegion planarRegion)
   {
      if (regionVisitedMatrix.get(row, column) || depthOfRegionsSearch > searchDepthLimit.get())
         return;

      ++depthOfRegionsSearch;
      regionVisitedMatrix.set(row, column, true);
      regionMatrix.set(row, column, planarRegionIslandIndex);
//      BytePointer patchPointer = regionOutputImage.getBytedecoOpenCVMat().ptr(row, column); // TODO: Is this faster?
//      float nx = patchPointer.getFloat(0);
//      float ny = patchPointer.getFloat(1);
//      float nz = patchPointer.getFloat(2);
//      float cx = patchPointer.getFloat(3);
//      float cy = patchPointer.getFloat(4);
//      float cz = patchPointer.getFloat(5);
      float nx = nxImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float ny = nyImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float nz = nzImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float cx = cxImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float cy = cyImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      float cz = czImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
      planarRegion.addPatch(nx, ny, nz, cx, cy, cz);
//
      if (drawPatches.get())
      {
         int x = column * patchHeight;
         int y = row * patchWidth;
         cvCenterPoint.x(x);
         cvCenterPoint.y(y);
         int radius = 2;
         int r = (planarRegionIslandIndex + 1) * 312 % 255;
         int g = (planarRegionIslandIndex + 1) * 123 % 255;
         int b = (planarRegionIslandIndex + 1) * 231 % 255;
         cvBGRColorScalar.put(b, g, r, 255);
         opencv_imgproc.circle(debugExtractionPanel.getBytedecoImage().getBytedecoOpenCVMat(), cvCenterPoint, radius, cvBGRColorScalar);
      }

      int count = 0;
      for (int i = 0; i < 8; i++)
      {
         if (row + adjacentY[i] < subHeight - 1 && row + adjacentY[i] > 1 && column + adjacentX[i] < subWidth - 1 && column + adjacentX[i] > 1)
         {
            int newPatch = Byte.toUnsignedInt(graphImage.getBytedecoOpenCVMat().ptr((row + adjacentY[i]), (column + adjacentX[i])).get());
            if (newPatch == 255)
            {
               ++count;
               depthFirstSearch(row + adjacentY[i], column + adjacentX[i], planarRegionIslandIndex, planarRegion);
            }
         }
      }
      if (count != 8)
      {
         boundaryMatrix.set(row, column, true);
         planarRegion.insertLeafPatch(row, column);
      }
   }

   private void findBoundariesAndHoles()
   {
      boundaryMaxSearchDepth = 0;
      numberOfBoundaryIndices = 0;
      boundaryVisitedMatrix.zero();
      planarRegions.parallelStream().forEach(planarRegion ->
      {
         int regionRingIndex = 0;
         for (Point2D leafPatch : planarRegion.getLeafPatches())
         {
            depthOfBoundariesSearch = 0;
            GDXGPURegionRing regionRing = planarRegion.getRegionRings().add();
            regionRing.reset(regionRingIndex);
            boundaryDepthFirstSearch((int) leafPatch.getX(), (int) leafPatch.getY(), planarRegion.getId(), regionRingIndex, regionRing);
            if (depthOfBoundariesSearch > 3)
            {
               ++regionRingIndex;
            }
            else
            {
               planarRegion.getRegionRings().remove(planarRegion.getRegionRings().size() - 1);
            }
            if (depthOfBoundariesSearch > boundaryMaxSearchDepth)
               boundaryMaxSearchDepth = depthOfBoundariesSearch;
         }
         planarRegion.getRegionRings().sort(boundaryVertexComparator);
      });
   }

   private void boundaryDepthFirstSearch(int row, int column, int planarRegionId, int regionRingIndex, GDXGPURegionRing regionRing)
   {
      if (boundaryVisitedMatrix.get(row, column) || depthOfBoundariesSearch > searchDepthLimit.get())
         return;

      ++depthOfBoundariesSearch;
      boundaryVisitedMatrix.set(row, column, true);
      regionRing.getBoundaryIndices().add().set(row, column);
      ++numberOfBoundaryIndices;

      if (drawBoundaries.get())
      {
         int x = column * patchHeight;
         int y = row * patchWidth;
         cvCenterPoint.x(x);
         cvCenterPoint.y(y);
         int radius = 2;
         int r = (regionRingIndex + 1) * 130 % 255;
         int g = (regionRingIndex + 1) * 227 % 255;
         int b = (regionRingIndex + 1) * 332 % 255;
         cvBGRColorScalar.put(b, g, r, 255);
         opencv_imgproc.circle(debugExtractionPanel.getBytedecoImage().getBytedecoOpenCVMat(), cvCenterPoint, radius, cvBGRColorScalar);
      }

      for (int i = 0; i < 8; i++)
      {
         if (row + adjacentY[i] < subHeight - 1 && row + adjacentY[i] > 1 && column + adjacentX[i] < subWidth - 1 && column + adjacentX[i] > 1)
         {
            if (boundaryMatrix.get(row + adjacentY[i], column + adjacentX[i]) && planarRegionId == regionMatrix.get(row + adjacentY[i], column + adjacentX[i]))
            {
               boundaryDepthFirstSearch(row + adjacentY[i], column + adjacentX[i], planarRegionId, regionRingIndex, regionRing);
            }
         }
      }
   }

   private void growRegionBoundaries()
   {
      planarRegions.parallelStream().forEach(planarRegion ->
      {
         if (!planarRegion.getRegionRings().isEmpty())
         {
            GDXGPURegionRing firstRing = planarRegion.getRegionRings().get(0);
            for (Vector2D boundaryIndex : firstRing.getBoundaryIndices())
            {
//               BytePointer patchPointer = regionOutputImage.getBytedecoOpenCVMat().ptr((int) boundaryIndex.getX(), (int) boundaryIndex.getY());
               float vertexX = cxImage.getBytedecoOpenCVMat().ptr((int) boundaryIndex.getX(), (int) boundaryIndex.getY()).getFloat();
               float vertexY = cyImage.getBytedecoOpenCVMat().ptr((int) boundaryIndex.getX(), (int) boundaryIndex.getY()).getFloat();
               float vertexZ = czImage.getBytedecoOpenCVMat().ptr((int) boundaryIndex.getX(), (int) boundaryIndex.getY()).getFloat();
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

   /** FIXME: This method filled with allocations. */
   public void renderPlanarRegions(ReferenceFrame cameraFrame)
   {
      if (!render3DPlanarRegions.get())
         return;

      List<List<PlanarRegion>> listOfListsOfRegions = planarRegions.parallelStream()
         .filter(gpuPlanarRegion -> gpuPlanarRegion.getBoundaryVertices().size() >= polygonizerParameters.getMinNumberOfNodes())
         .map(gpuPlanarRegion ->
         {
            List<PlanarRegion> planarRegions = new ArrayList<>();
            try
            {
               orientation.setYawPitchRollIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
//               orientation.changeFrame(cmosFrame);
               Vector3D32 normal = gpuPlanarRegion.getNormal();
//               orientation.set(EuclidGeometryTools.axisAngleFromZUpToVector3D(normal));

               // First compute the set of concave hulls for this region
               Point3D32 origin = gpuPlanarRegion.getCenter();
               List<Point2D> pointCloudInPlane = gpuPlanarRegion.getBoundaryVertices().stream()
                  .map(boundaryVertex -> PolygonizerTools.toPointInPlane(new Point3D(boundaryVertex), origin, orientation))
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

//               ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHullCollection);
//               ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHullCollection);
//               if (polygonizerParameters.getCutNarrowPassage())
//                  concaveHullCollection = ConcaveHullPruningFilteringTools.concaveHullNarrowPassageCutter(lengthThreshold, concaveHullCollection);

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
//                  tempTransform.set(orientation, origin);
//                  regionPose.setIncludingFrame(cmosFrame, origin, orientation);
                  regionPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), origin, orientation);
//                  regionPose.changeFrame(ReferenceFrame.getWorldFrame());
                  regionPose.getOrientation().setYawPitchRoll(0.0, 0.0, 0.0);
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
//               e.printStackTrace();
            }
            return planarRegions;
         })
         .collect(Collectors.toList());
      planarRegionsList.clear();
      for (List<PlanarRegion> planarRegions : listOfListsOfRegions)
      {
         planarRegionsList.addPlanarRegions(planarRegions);
      }
//      tempTransform.setIdentity();
//      tempTransform.appendPitchRotation(Math.PI / 2.0);
//      tempTransform.appendYawRotation(-Math.PI / 2.0);
//      planarRegionsList.applyTransform(tempTransform);
//      planarRegionsList.applyTransform(cameraFrame.getTransformToWorldFrame());

      planarRegionsGraphic.generateMeshes(planarRegionsList);
      planarRegionsGraphic.update();
   }

   public void renderBoundaryPoints(ReferenceFrame cameraFrame, Matrix4 invProjectionView)
   {
      pointsToRender.clear();
      for (GDXGPUPlanarRegion planarRegion : planarRegions)
      {
         if (render3DBoundaries.get())
         {
            if (!planarRegion.getRegionRings().isEmpty())
            {
               GDXGPURegionRing firstRing = planarRegion.getRegionRings().get(0);
               for (Vector2D boundaryIndex : firstRing.getBoundaryIndices())
               {
                  int column = (int) boundaryIndex.getY() * patchWidth;
                  int row = (int) boundaryIndex.getX() * patchHeight;
                  float z = inputFloatDepthImage.getBytedecoOpenCVMat().ptr(row, column).getFloat();
                  tempFramePoint.setIncludingFrame(cameraFrame, column, row, z);
                  ProjectionTools.projectDepthPixelToIHMCZUp3D(tempFramePoint,
                                                               principalOffsetXPixels.get(),
                                                               principalOffsetYPixels.get(),
                                                               focalLengthXPixels.get(),
                                                               focalLengthYPixels.get());
                  tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
                  pointsToRender.add().set(tempFramePoint);
               }
            }
         }

         if (render3DGrownBoundaries.get())
         {
            for (Vector3D boundaryVertex : planarRegion.getBoundaryVertices())
            {
               tempFramePoint.setIncludingFrame(cameraFrame, boundaryVertex);
               ProjectionTools.projectDepthPixelToIHMCZUp3D(tempFramePoint,
                                                            principalOffsetXPixels.get(),
                                                            principalOffsetYPixels.get(),
                                                            focalLengthXPixels.get(),
                                                            focalLengthYPixels.get());
               tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
               pointsToRender.add().set(tempFramePoint);
            }
         }
      }

      boundaryPointCloud.setPointsToRender(pointsToRender, Color.WHITE);
      boundaryPointCloud.updateMesh();
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
      nativeParameterArray.put(7, focalLengthXPixels.get());
      nativeParameterArray.put(8, focalLengthYPixels.get());
      nativeParameterArray.put(9, principalOffsetXPixels.get());
      nativeParameterArray.put(10, principalOffsetYPixels.get());
      nativeParameterArray.put(11, deadPixelFilterPatchSize.get());
      nativeParameterArray.put(12, filterSubHeight);
      nativeParameterArray.put(13, filterSubWidth);
      nativeParameterArray.put(14, inputHeight.get());
      nativeParameterArray.put(15, inputWidth.get());

      openCLManager.enqueueWriteBuffer(parametersBufferObject, nativeParameterArray); // TODO: Necessary?
   }

   private void calculateDetivativeParameters()
   {
      patchHeight = patchSize.get();
      patchWidth = patchSize.get();
      subHeight = inputHeight.get() / patchHeight;
      subWidth = inputWidth.get() / patchWidth;
      filterSubHeight = inputHeight.get() / deadPixelFilterPatchSize.get();
      filterSubWidth = inputWidth.get() / deadPixelFilterPatchSize.get();
   }

   public void renderImGuiWidgets()
   {
      numberOfPlanarRegionsPlot.render((float) planarRegions.size());
      regionMaxSearchDepthPlot.render((float) regionMaxSearchDepth);
      numberOfBoundaryVerticesPlot.render((float) numberOfBoundaryIndices);
      boundaryMaxSearchDepthPlot.render((float) boundaryMaxSearchDepth);

      ImGui.checkbox(labels.get("Early gaussian blur"), earlyGaussianBlur);
      ImGui.sliderInt(labels.get("Gaussian size"), gaussianSize.getData(), 1, 6);
      ImGui.sliderInt(labels.get("Gaussian sigma"), gaussianSigma.getData(), 1, 30);
      ImGui.sliderInt(labels.get("Patch size"), patchSize.getData(), 1, 20);
      ImGui.sliderInt(labels.get("Dead pixel filter patch size"), deadPixelFilterPatchSize.getData(), 1, 20);
      ImGui.checkbox(labels.get("Use filtered image"), useFilteredImage);
      ImGui.sliderFloat(labels.get("Merge distance threshold"), mergeDistanceThreshold.getData(), 0.0f, 0.1f);
      ImGui.sliderFloat(labels.get("Merge angular threshold"), mergeAngularThreshold.getData(), 0.0f, 1.0f);
      ImGui.sliderInt(labels.get("Search depth limit"), searchDepthLimit.getData(), 1, 50000);
      ImGui.sliderInt(labels.get("Region min patches"), regionMinPatches.getData(), 1, 30);
      ImGui.sliderInt(labels.get("Region boundary diff"), regionBoundaryDiff.getData(), 1, 30);
      ImGui.checkbox(labels.get("Draw patches"), drawPatches);
      ImGui.checkbox(labels.get("Draw boundaries"), drawBoundaries);
      ImGui.checkbox(labels.get("Render 3D planar regions"), render3DPlanarRegions);
      ImGui.checkbox(labels.get("Render 3D boundaries"), render3DBoundaries);
      ImGui.checkbox(labels.get("Render 3D grown boundaries"), render3DGrownBoundaries);
      ImGui.inputDouble(labels.get("Region growth factor"), regionGrowthFactor);
      ImGui.text("Input height: " + inputHeight);
      ImGui.text("Input width: " + inputWidth);
      ImGui.sliderFloat(labels.get("Focal length X (px)"), focalLengthXPixels.getData(), -1000.0f, 1000.0f);
      ImGui.sliderFloat(labels.get("Focal length Y (px)"), focalLengthYPixels.getData(), -1000.0f, 1000.0f);
      ImGui.sliderFloat(labels.get("Principal offset X (px)"), principalOffsetXPixels.getData(), -imageWidth, imageWidth);
      ImGui.sliderFloat(labels.get("Principal offset Y (px)"), principalOffsetYPixels.getData(), -imageHeight, imageHeight);

      ImGui.sliderFloat("Edge Length Threshold", edgeLengthTresholdSlider.getData(), 0, 0.5f);
      concaveHullFactoryParameters.setEdgeLengthThreshold(edgeLengthTresholdSlider.get());
      ImGui.sliderFloat("Triangulation Tolerance", triangulationToleranceSlider.getData(), 0, 1);
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
      planarRegionsGraphic.getRenderables(renderables, pool);
      boundaryPointCloud.getRenderables(renderables, pool);
   }

   public ImGuiPanel getPanel()
   {
      return imguiPanel;
   }
}
