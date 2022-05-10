package us.ihmc.avatar.gpuPlanarRegions;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Size;
import org.ejml.data.BMatrixRMaj;
import org.ejml.data.DMatrixRMaj;
import sensor_msgs.Image;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.LinearTransform3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.robotEnvironmentAwareness.geometry.*;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import static us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters.*;

public class GPUPlanarRegionExtraction
{
   private final GPUPlanarRegionExtractionParameters parameters;
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters;
   private final PolygonizerParameters polygonizerParameters;

   private BytedecoImage inputFloatDepthImage;
   private BytedecoImage inputScaledFloatDepthImage;
   private BytedecoImage inputU16DepthImage;
   private BytedecoImage blurredDepthImage;
   private BytedecoImage filteredDepthImage;
   private BytedecoImage nxImage;
   private BytedecoImage nyImage;
   private BytedecoImage nzImage;
   private BytedecoImage cxImage;
   private BytedecoImage cyImage;
   private BytedecoImage czImage;
   private BytedecoImage graphImage;
   private BMatrixRMaj regionVisitedMatrix;
   private BMatrixRMaj boundaryVisitedMatrix;
   private BMatrixRMaj boundaryMatrix;
   private DMatrixRMaj regionMatrix;
   private boolean patchSizeChanged = false;
   private int numberOfRegionPatches = 0;
   private int regionMaxSearchDepth = 0;
   private int boundaryMaxSearchDepth = 0;
   private int numberOfBoundaryPatchesInWholeImage = 0;
   private double maxSVDSolveTime = Double.NaN;
   private final int[] adjacentY = {-1, 0, 1, 1, 1, 0, -1, -1};
   private final int[] adjacentX = {-1, -1, -1, 0, 1, 1, 1, 0};
   private final RecyclingArrayList<GPUPlanarRegion> gpuPlanarRegions = new RecyclingArrayList<>(GPUPlanarRegion::new);
   private final Comparator<GPURegionRing> boundaryIndexComparator = Comparator.comparingInt(regionRing -> regionRing.getBoundaryIndices().size());
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

   private final PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private final GPUPlanarRegionIsland tempIsland = new GPUPlanarRegionIsland();
   private boolean firstRun = true;

   public GPUPlanarRegionExtraction()
   {
      this(new GPUPlanarRegionExtractionParameters(), new PolygonizerParameters(), new ConcaveHullFactoryParameters());
   }

   public GPUPlanarRegionExtraction(GPUPlanarRegionExtractionParameters parameters,
                                    PolygonizerParameters polygonizerParameters,
                                    ConcaveHullFactoryParameters concaveHullFactoryParameters)
   {
      this.parameters = parameters;
      this.polygonizerParameters = polygonizerParameters;
      this.concaveHullFactoryParameters = concaveHullFactoryParameters;
   }

   public void create(int imageWidth, int imageHeight, ByteBuffer sourceDepthByteBufferOfFloats, double fx, double fy, double cx, double cy)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      parameters.set(focalLengthXPixels, fx);
      parameters.set(focalLengthYPixels, fy);
      parameters.set(principalOffsetXPixels, cx);
      parameters.set(principalOffsetYPixels, cy);

      parametersBuffer = new OpenCLFloatBuffer(16);
      calculateDerivativeParameters();
      inputFloatDepthImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1, sourceDepthByteBufferOfFloats);
      inputScaledFloatDepthImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1);
      inputU16DepthImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      blurredDepthImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      filteredDepthImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1);
      nxImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      nyImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      nzImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      cxImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      cyImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      czImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_32FC1);
      graphImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_8UC1);
      gaussianKernelSize = new Size();

      openCLManager.create();
      planarRegionExtractionProgram = openCLManager.loadProgram("PlanarRegionExtraction");
      filterKernel = openCLManager.createKernel(planarRegionExtractionProgram, "filterKernel");
      packKernel = openCLManager.createKernel(planarRegionExtractionProgram, "packKernel");
      mergeKernel = openCLManager.createKernel(planarRegionExtractionProgram, "mergeKernel");

      regionVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryVisitedMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      boundaryMatrix = new BMatrixRMaj(patchImageHeight, patchImageWidth);
      regionMatrix = new DMatrixRMaj(patchImageHeight, patchImageWidth);
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

   public void extractPlanarRegions(ReferenceFrame cameraFrame, Runnable onPatchSizeChanged)
   {
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

      // Flip so the Y+ goes up instead of down.
      opencv_core.flip(inputU16DepthImage.getBytedecoOpenCVMat(), inputU16DepthImage.getBytedecoOpenCVMat(), BytedecoOpenCVTools.FLIP_Y);

      int size = parameters.getGaussianSize() * 2 + 1;
      gaussianKernelSize.width(size);
      gaussianKernelSize.height(size);
      double sigmaX = parameters.getGaussianSigma();
      double sigmaY = sigmaX;
      int borderType = opencv_core.BORDER_DEFAULT;
      opencv_imgproc.GaussianBlur(inputU16DepthImage.getBytedecoOpenCVMat(),
                                  blurredDepthImage.getBytedecoOpenCVMat(),
                                  gaussianKernelSize,
                                  sigmaX,
                                  sigmaY,
                                  borderType);

      parametersBuffer.getBytedecoFloatBufferPointer().put(0, (float) parameters.getFilterDisparityThreshold());
      parametersBuffer.getBytedecoFloatBufferPointer().put(1, (float) parameters.getMergeAngularThreshold());
      parametersBuffer.getBytedecoFloatBufferPointer().put(2, (float) parameters.getMergeDistanceThreshold());
      parametersBuffer.getBytedecoFloatBufferPointer().put(3, patchHeight);
      parametersBuffer.getBytedecoFloatBufferPointer().put(4, patchWidth);
      parametersBuffer.getBytedecoFloatBufferPointer().put(5, patchImageHeight);
      parametersBuffer.getBytedecoFloatBufferPointer().put(6, patchImageWidth);
      parametersBuffer.getBytedecoFloatBufferPointer().put(7, (float) parameters.getFocalLengthXPixels());
      parametersBuffer.getBytedecoFloatBufferPointer().put(8, (float) parameters.getFocalLengthYPixels());
      parametersBuffer.getBytedecoFloatBufferPointer().put(9, (float) parameters.getPrincipalOffsetXPixels());
      parametersBuffer.getBytedecoFloatBufferPointer().put(10, (float) parameters.getPrincipalOffsetYPixels());
      parametersBuffer.getBytedecoFloatBufferPointer().put(11, parameters.getDeadPixelFilterPatchSize());
      parametersBuffer.getBytedecoFloatBufferPointer().put(12, filterPatchImageHeight);
      parametersBuffer.getBytedecoFloatBufferPointer().put(13, filterPatchImageWidth);
      parametersBuffer.getBytedecoFloatBufferPointer().put(14, imageHeight);
      parametersBuffer.getBytedecoFloatBufferPointer().put(15, imageWidth);

      if (patchSizeChanged)
      {
         patchSizeChanged = false;
         LogTools.info("Resizing patch image to {}x{}", patchImageWidth, patchImageHeight);
         nxImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         nyImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         nzImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         cxImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         cyImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         czImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         graphImage.resize(patchImageWidth, patchImageHeight, openCLManager, null);
         if (onPatchSizeChanged != null)
            onPatchSizeChanged.run();
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

      _cl_mem inputImage = parameters.getEarlyGaussianBlur() ? blurredDepthImage.getOpenCLImageObject() : inputU16DepthImage.getOpenCLImageObject();

      openCLManager.setKernelArgument(filterKernel, 0, inputImage);
      openCLManager.setKernelArgument(filterKernel, 1, filteredDepthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(filterKernel, 2, nxImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(filterKernel, 3, parametersBuffer.getOpenCLBufferObject());

      _cl_mem packKernelInputObject = parameters.getUseFilteredImage() ? filteredDepthImage.getOpenCLImageObject() : inputU16DepthImage.getOpenCLImageObject();
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
   }

   public void findRegions(Consumer<GPUPlanarRegionIsland> forDrawingDebugPanel)
   {
      int planarRegionIslandIndex = 0;
      regionMaxSearchDepth = 0;
      gpuPlanarRegions.clear();
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
               GPUPlanarRegion planarRegion = gpuPlanarRegions.add();
               planarRegion.reset(planarRegionIslandIndex);
               regionsDepthFirstSearch(row, column, planarRegionIslandIndex, planarRegion, 1);
               if (numberOfRegionPatches >= parameters.getRegionMinPatches())
               {
                  ++planarRegionIslandIndex;
                  planarRegion.update(parameters.getUseSVDNormals(), parameters.getSVDReductionFactor());
                  if (planarRegion.getSVDDuration() > maxSVDSolveTime)
                     maxSVDSolveTime = planarRegion.getSVDDuration();

                  tempIsland.planarRegion = planarRegion;
                  tempIsland.planarRegionIslandIndex = planarRegionIslandIndex;
                  if (forDrawingDebugPanel != null)
                     forDrawingDebugPanel.accept(tempIsland);
               }
               else
               {
                  gpuPlanarRegions.remove(gpuPlanarRegions.size() - 1);
               }
               if (numberOfRegionPatches > regionMaxSearchDepth)
                  regionMaxSearchDepth = numberOfRegionPatches;
            }
         }
      }
   }

   private void regionsDepthFirstSearch(int row, int column, int planarRegionIslandIndex, GPUPlanarRegion planarRegion, int searchDepth)
   {
      if (regionVisitedMatrix.get(row, column) || searchDepth > parameters.getSearchDepthLimit())
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

   public void findBoundariesAndHoles(Consumer<GPURegionRing> forDrawingDebugPanel)
   {
      boundaryVisitedMatrix.zero();
      boundaryMaxSearchDepth = 0;
      gpuPlanarRegions.parallelStream().forEach(planarRegion ->
      {
         int leafPatchIndex = 0;
         int regionRingIndex = 0;
         for (Point2D leafPatch : planarRegion.getBorderIndices())
         {
            GPURegionRing regionRing = planarRegion.getRegionRings().add();
            regionRing.reset();
            regionRing.setIndex(regionRingIndex);
            int numberOfBoundaryPatches = boundaryDepthFirstSearch((int) leafPatch.getY(),
                                                                   (int) leafPatch.getX(),
                                                                   planarRegion.getId(),
                                                                   regionRing,
                                                                   leafPatchIndex,
                                                                   1);
            if (numberOfBoundaryPatches >= parameters.getBoundaryMinPatches())
            {
               if (forDrawingDebugPanel != null)
                  forDrawingDebugPanel.accept(regionRing);
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

   private int boundaryDepthFirstSearch(int row, int column, int planarRegionId, GPURegionRing regionRing, int leafPatchIndex, int searchDepth)
   {
      if (boundaryVisitedMatrix.get(row, column) || searchDepth > parameters.getSearchDepthLimit())
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

   public void growRegionBoundaries()
   {
      gpuPlanarRegions.forEach(planarRegion ->
      {
         if (!planarRegion.getRegionRings().isEmpty())
         {
            GPURegionRing firstRing = planarRegion.getRegionRings().get(0);
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
               boundaryVertex.scale(parameters.getRegionGrowthFactor());
               boundaryVertex.add(vertexX, vertexY, vertexZ);
            }
         }
      });
   }

   public void computePlanarRegions(ReferenceFrame cameraFrame)
   {
      List<List<PlanarRegion>> listOfListsOfRegions = gpuPlanarRegions.parallelStream()
      .filter(gpuPlanarRegion -> gpuPlanarRegion.getBoundaryVertices().size() >= polygonizerParameters.getMinNumberOfNodes())
      .map(gpuPlanarRegion ->
      {
         List<PlanarRegion> planarRegions = new ArrayList<>();
         try
         {
            // Going through LinearTransform3D first prevents NotARotationMatrix exceptions.
            LinearTransform3D linearTransform3D = new LinearTransform3D(EuclidGeometryTools.axisAngleFromZUpToVector3D(gpuPlanarRegion.getNormal()));
            linearTransform3D.normalize();
            FrameQuaternion orientation = new FrameQuaternion();
            orientation.setIncludingFrame(cameraFrame, linearTransform3D.getAsQuaternion());
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

   private void calculateDerivativeParameters()
   {
      int previousPatchHeight = patchHeight;
      int previousPatchWidth = patchWidth;
      int previousPatchImageHeight = patchImageHeight;
      int previousPatchImageWidth = patchImageWidth;
      int previousFilterPatchImageHeight = filterPatchImageHeight;
      int previousFilterPatchImageWidth = filterPatchImageWidth;

      patchHeight = parameters.getPatchSize();
      patchWidth = parameters.getPatchSize();
      patchImageHeight = imageHeight / patchHeight;
      patchImageWidth = imageWidth / patchWidth;
      filterPatchImageHeight = imageHeight / parameters.getDeadPixelFilterPatchSize();
      filterPatchImageWidth = imageWidth / parameters.getDeadPixelFilterPatchSize();

      int newPatchHeight = patchHeight;
      int newPatchWidth = patchWidth;
      int newPatchImageHeight = patchImageHeight;
      int newPatchImageWidth = patchImageWidth;
      int newFilterPatchImageHeight = filterPatchImageHeight;
      int newFilterPatchImageWidth = filterPatchImageWidth;

      boolean changed = previousPatchHeight != newPatchHeight;
      changed |= previousPatchWidth != newPatchWidth;
      changed |= previousPatchImageHeight != newPatchImageHeight;
      changed |= previousPatchImageWidth != newPatchImageWidth;
      changed |= previousFilterPatchImageHeight != newFilterPatchImageHeight;
      changed |= previousFilterPatchImageWidth != newFilterPatchImageWidth;

      if (changed)
      {
         LogTools.info("Updated patch sizes:");
         LogTools.info("newPatchHeight: {} -> {}", previousPatchHeight, newPatchHeight);
         LogTools.info("newPatchWidth: {} -> {}", previousPatchWidth, newPatchWidth);
         LogTools.info("newPatchImageHeight: {} -> {}", previousPatchImageHeight, newPatchImageHeight);
         LogTools.info("newPatchImageWidth: {} -> {}", previousPatchImageWidth, newPatchImageWidth);
         LogTools.info("newFilterPatchImageHeight: {} -> {}", previousFilterPatchImageHeight, newFilterPatchImageHeight);
         LogTools.info("newFilterPatchImageWidth: {} -> {}", previousFilterPatchImageWidth, newFilterPatchImageWidth);
      }
   }

   public void destroy()
   {
      openCLManager.destroy();
      // TODO: Destroy the rest
   }

   public void setPatchSizeChanged(boolean patchSizeChanged)
   {
      this.patchSizeChanged = patchSizeChanged;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public int getPatchImageWidth()
   {
      return patchImageWidth;
   }

   public int getPatchImageHeight()
   {
      return patchImageHeight;
   }

   public OpenCLManager getOpenCLManager()
   {
      return openCLManager;
   }

   public int getNumberOfBoundaryPatchesInWholeImage()
   {
      return numberOfBoundaryPatchesInWholeImage;
   }

   public BytedecoImage getBlurredDepthImage()
   {
      return blurredDepthImage;
   }

   public BytedecoImage getFilteredDepthImage()
   {
      return filteredDepthImage;
   }

   public BytedecoImage getNxImage()
   {
      return nxImage;
   }

   public BytedecoImage getNyImage()
   {
      return nyImage;
   }

   public BytedecoImage getNzImage()
   {
      return nzImage;
   }

   public BytedecoImage getCxImage()
   {
      return cxImage;
   }

   public BytedecoImage getCyImage()
   {
      return cyImage;
   }

   public BytedecoImage getCzImage()
   {
      return czImage;
   }

   public RecyclingArrayList<GPUPlanarRegion> getGPUPlanarRegions()
   {
      return gpuPlanarRegions;
   }

   public int getPatchWidth()
   {
      return patchWidth;
   }

   public int getPatchHeight()
   {
      return patchHeight;
   }

   public BytedecoImage getInputFloatDepthImage()
   {
      return inputFloatDepthImage;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public GPUPlanarRegionExtractionParameters getParameters()
   {
      return parameters;
   }

   public ConcaveHullFactoryParameters getConcaveHullFactoryParameters()
   {
      return concaveHullFactoryParameters;
   }

   public PolygonizerParameters getPolygonizerParameters()
   {
      return polygonizerParameters;
   }

   public int getRegionMaxSearchDepth()
   {
      return regionMaxSearchDepth;
   }

   public int getBoundaryMaxSearchDepth()
   {
      return boundaryMaxSearchDepth;
   }

   public double getMaxSVDSolveTime()
   {
      return maxSVDSolveTime;
   }
}
