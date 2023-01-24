package us.ihmc.perception.rapidRegions;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.concurrent.ScheduledExecutorService;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;

public class RapidRegionsDebutOutputGenerator
{
   private boolean enabled = false;
   private boolean showPointCloud = true;

   private Mat debugImage;
   private Scalar internalColor = new Scalar(0, 0, 255, 0);
   private Scalar boundaryColor = new Scalar(255, 255, 255, 0);
   private final RecyclingArrayList<Point3D32> debugPoints = new RecyclingArrayList<>(Point3D32::new);
   private final RecyclingArrayList<UnitVector3D> debugNormals = new RecyclingArrayList<>(UnitVector3D::new);

   public void create(int height, int width)
   {
      debugImage = new Mat(height, width, opencv_core.CV_8UC4);
   }

   public void drawRegionInternalPatches(GPUPlanarRegionIsland island, int patchHeight, int patchWidth)
   {
      if (!enabled)
         return;

      for (Point2D regionIndex : island.planarRegion.getRegionIndices())
      {
         int x = (int) regionIndex.getX();
         int y = (int) regionIndex.getY();
         int r = (island.planarRegionIslandIndex + 1) * 312 % 255;
         int g = (island.planarRegionIslandIndex + 1) * 123 % 255;
         int b = (island.planarRegionIslandIndex + 1) * 231 % 255;
         BytePointer pixel = debugImage.ptr(y * patchHeight, x * patchWidth);
         pixel.put(0, (byte) r);
         pixel.put(1, (byte) g);
         pixel.put(2, (byte) b);
      }
   }

   public void drawRegionRing(GPURegionRing regionRing, int patchHeight, int patchWidth)
   {
      if (!enabled)
         return;

      for (Vector2D boundaryIndex : regionRing.getBoundaryIndices())
      {
         int x = (int) boundaryIndex.getX();
         int y = (int) boundaryIndex.getY();
         int r = (regionRing.getIndex() + 1) * 130 % 255;
         int g = (regionRing.getIndex() + 1) * 227 % 255;
         int b = (regionRing.getIndex() + 1) * 332 % 255;
         BytePointer pixel = debugImage.ptr(y * patchHeight, x * patchWidth);
         pixel.put(0, (byte) r);
         pixel.put(1, (byte) g);
         pixel.put(2, (byte) b);
      }
   }

   public void constructPointCloud(FloatBuffer buffer, int numberOfPoints)
   {
      if (!enabled)
         return;

      debugPoints.clear();
      for (int i = 0; i < numberOfPoints; i++)
      {
         float cx = buffer.get(i * 3);
         float cy = buffer.get(i * 3 + 1);
         float cz = buffer.get(i * 3 + 2);

         //LogTools.info("Point To Render: {} {} {}", cx, cy, cz);

         Point3D32 point = new Point3D32(cx, cy, cz);

         if (point.norm() > 0.1f)
         {
            debugPoints.add().set(cx, cy, cz);
            //LogTools.info("Point To Render: {} {} {}", cx, cy, cz);
         }
      }
   }

   public void constructPointCloud(BytedecoImage cxImage, BytedecoImage cyImage, BytedecoImage czImage)
   {
      if (!enabled)
         return;

      FloatBuffer cxBuffer = cxImage.getBackingDirectByteBuffer().asFloatBuffer();
      FloatBuffer cyBuffer = cyImage.getBackingDirectByteBuffer().asFloatBuffer();
      FloatBuffer czBuffer = czImage.getBackingDirectByteBuffer().asFloatBuffer();

      int rows = cxImage.getImageHeight();
      int cols = cxImage.getImageWidth();

      for (int y = 0; y < rows; y++)
      {
         for (int x = 0; x < cols; x++)
         {
            float cx = cxBuffer.get(y * cols + x);
            float cy = cyBuffer.get(y * cols + x);
            float cz = czBuffer.get(y * cols + x);

            //LogTools.info(String.format("Centroid: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f", cx, cy, cz, cv_cx, cv_cy, cv_cz));
            if (!(cx == 0.0f && cy == 0.0f && cz == 0.0f))
            {
               debugPoints.add().set(cx, cy, cz);
            }
         }
      }
   }

   public void constructCentroidSurfelCloud(BytedecoImage cxImage,
                                            BytedecoImage cyImage,
                                            BytedecoImage czImage,
                                            BytedecoImage nxImage,
                                            BytedecoImage nyImage,
                                            BytedecoImage nzImage)
   {

      if (!enabled)
         return;

      FloatBuffer cxBuffer = cxImage.getBackingDirectByteBuffer().asFloatBuffer();
      FloatBuffer cyBuffer = cyImage.getBackingDirectByteBuffer().asFloatBuffer();
      FloatBuffer czBuffer = czImage.getBackingDirectByteBuffer().asFloatBuffer();

      FloatBuffer nxBuffer = nxImage.getBackingDirectByteBuffer().asFloatBuffer();
      FloatBuffer nyBuffer = nyImage.getBackingDirectByteBuffer().asFloatBuffer();
      FloatBuffer nzBuffer = nzImage.getBackingDirectByteBuffer().asFloatBuffer();

      int rows = cxImage.getImageHeight();
      int cols = cxImage.getImageWidth();

      for (int y = 0; y < rows; y++)
      {
         for (int x = 0; x < cols; x++)
         {
            float cx = cxBuffer.get(y * cols + x);
            float cy = cyBuffer.get(y * cols + x);
            float cz = czBuffer.get(y * cols + x);

            float nx = cxBuffer.get(y * cols + x);
            float ny = cyBuffer.get(y * cols + x);
            float nz = czBuffer.get(y * cols + x);

            if (!(cx == 0.0f && cy == 0.0f && cz == 0.0f))
            {
               debugPoints.add().set(cx, cy, cz);
               debugNormals.add().set(nx, ny, nz);
            }
         }
      }
   }

   public void printPatchGraph(BytedecoImage patchGraph)
   {
      if (!enabled)
         return;

      for (int i = 0; i < patchGraph.getImageHeight(); i++)
      {
         for (int j = 0; j < patchGraph.getImageWidth(); j++)
         {
            int value = patchGraph.getCharDirect(i, j);
            if (value == 255)
               System.out.print("o");
            else if (value > 0)
               System.out.print("+");
            else
               System.out.print(".");
         }
         System.out.println();
      }
   }

   public void drawInternalNode(int id, int patchRow, int patchCol, int patchHeight, int patchWidth)
   {
      if (!enabled)
         return;

      drawNode(patchRow, patchCol, patchHeight, patchWidth, getColor(id));
   }

   public void drawBoundaryNode(int id, int patchRow, int patchCol, int patchHeight, int patchWidth)
   {
      if (!enabled)
         return;

      drawNode(patchRow, patchCol, patchHeight, patchWidth, boundaryColor);
   }

   private void drawNode(int patchRow, int patchCol, int patchHeight, int patchWidth, Scalar color)
   {
      if (!enabled)
         return;

      opencv_imgproc.circle(debugImage, new Point(patchRow * patchHeight, patchCol * patchWidth), 2, color, -1, -1, 0);
   }

   private Scalar getColor(int id)
   {
      internalColor.red((id + 1) * 231 % 255);
      internalColor.green((id + 1) * 123 % 255);
      internalColor.blue((id + 1) * 312 % 255);
      return internalColor;
   }

   public void clearDebugImage()
   {
      debugImage.put(new Scalar(0, 0, 0, 0));
   }

   public void update(Mat inputDepthImage, PatchFeatureGrid patchFeatureGrid, BytedecoImage patchGraph, FloatBuffer floatBuffer)
   {
      if (!enabled)
         return;

//      printPatchGraph(patchGraph);

      if(showPointCloud)
      {
         constructPointCloud(floatBuffer, inputDepthImage.rows() * inputDepthImage.cols());
      }
//      constructPointCloud(patchFeatureGrid.getCxImage(), patchFeatureGrid.getCyImage(), patchFeatureGrid.getCzImage());

      //      constructCentroidSurfelCloud(patchFeatureGrid.getCxImage(), patchFeatureGrid.getCyImage(), patchFeatureGrid.getCzImage(), patchFeatureGrid.getNxImage(),
//                                   patchFeatureGrid.getNyImage(), patchFeatureGrid.getNzImage());
//      displayInputDepth(inputDepthImage, 1);
      showDebugImage(1);
   }

   public void displayInputDepth(Mat depth, int delay)
   {
      if (!enabled)
         return;

      Mat depthDisplay = new Mat();
      BytedecoOpenCVTools.clampTo8BitUnsignedChar(depth, depthDisplay, 0.0, 255.0);
      BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(depthDisplay, depthDisplay);

      imshow("Depth", depthDisplay);
      int code = waitKeyEx(delay);
      if (code == 113)
      {
         System.exit(0);
      }
   }

   public void showDebugImage(int delay)
   {
      if (!enabled)
         return;

      BytedecoOpenCVTools.display("Debug Output", debugImage, delay);
   }

   public Mat getDebugImage()
   {
      return debugImage;
   }

   public void transformPoints(RigidBodyTransform transform)
   {
      if (!enabled)
         return;

      for(Point3D32 point : debugPoints)
      {
         point.applyTransform(transform);
      }
   }

   public RecyclingArrayList<Point3D32> getDebugPoints()
   {
      return debugPoints;
   }

   public RecyclingArrayList<UnitVector3D> getDebugNormals()
   {
      return debugNormals;
   }

   public boolean isEnabled()
   {
      return enabled;
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   ///* A one-time method to convert depth map to renderable pointcloud. */
   //public void submitToPointCloudRenderer(int width, int height, BytedecoImage bytedecoImage, OpenCLManager openCLManager)
   //{
   //
   //   openCLProgram = openCLManager.loadProgram("OusterPointCloudVisualizer");
   //   unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "imageToPointCloud");
   //
   //   bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
   //
   //   totalNumberOfPoints = height * width;
   //   pointCloudRenderer.create(totalNumberOfPoints);
   //
   //   pointCloudVertexBuffer = new OpenCLFloatBuffer(totalNumberOfPoints * RDXPointCloudRenderer.FLOATS_PER_VERTEX, pointCloudRenderer.getVertexBuffer());
   //   pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);
   //   LogTools.info("Allocated new buffers. {} points.", totalNumberOfPoints);
   //
   //   // TODO: Create tuners for these
   //   double verticalFieldOfView = Math.PI / 2.0;
   //   double horizontalFieldOfView = 2.0 * Math.PI;
   //
   //   parametersOpenCLFloatBuffer.setParameter((float) horizontalFieldOfView);
   //   parametersOpenCLFloatBuffer.setParameter((float) verticalFieldOfView);
   //   parametersOpenCLFloatBuffer.setParameter(sensorTransformToWorld.getTranslation().getX32());
   //   parametersOpenCLFloatBuffer.setParameter(sensorTransformToWorld.getTranslation().getY32());
   //   parametersOpenCLFloatBuffer.setParameter(sensorTransformToWorld.getTranslation().getZ32());
   //   parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM00());
   //   parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM01());
   //   parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM02());
   //   parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM10());
   //   parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM11());
   //   parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM12());
   //   parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM20());
   //   parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM21());
   //   parametersOpenCLFloatBuffer.setParameter((float) sensorTransformToWorld.getRotation().getM22());
   //   parametersOpenCLFloatBuffer.setParameter(width);
   //   parametersOpenCLFloatBuffer.setParameter(height);
   //   parametersOpenCLFloatBuffer.setParameter(0.01f);
   //
   //   parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
   //   bytedecoImage.writeOpenCLImage(openCLManager);
   //   pointCloudRenderer.updateMeshFastestBeforeKernel();
   //   pointCloudVertexBuffer.syncWithBackingBuffer();
   //
   //   openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
   //   openCLManager.setKernelArgument(unpackPointCloudKernel, 1, bytedecoImage.getOpenCLImageObject());
   //   openCLManager.setKernelArgument(unpackPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
   //   openCLManager.execute2D(unpackPointCloudKernel, width, height);
   //
   //   pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);
   //   pointCloudRenderer.updateMeshFastestAfterKernel();
   //}

   //// TODO: Complete this for visualizing the patch centroids and normals
   //private RDXModelInstance constructSurfelMesh()
   //{
   //   RecyclingArrayList<Point3D32> debugPoints = rapidPlanarRegionsExtractor.getDebugger().getDebugPoints();
   //
   //   ModelBuilder modelBuilder = new ModelBuilder();
   //   modelBuilder.begin();
   //
   //   RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
   //
   //   ArrayList<Point3DReadOnly> points = new ArrayList<>();
   //   for (int i = 0; i < debugPoints.size(); i++)
   //   {
   //      points.clear();
   //
   //      points.add(debugPoints.get(i));
   //
   //      Point3D32 point = debugPoints.get(i);
   //      meshBuilder.addPolygon(points, new Color(0.5f, 0.6f, 0.0f, 1.0f)); // dark red
   //   }
   //
   //   Mesh mesh = meshBuilder.generateMesh();
   //   MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL20.GL_TRIANGLES);
   //   Material material = new Material();
   //   Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
   //   material.set(TextureAttribute.createDiffuse(paletteTexture));
   //   material.set(ColorAttribute.createDiffuse(com.badlogic.gdx.graphics.Color.WHITE));
   //   modelBuilder.part(meshPart, material);
   //
   //   Model model = modelBuilder.end();
   //
   //   RDXModelInstance modelInstance = new RDXModelInstance(model);
   //   return modelInstance;
   //}

   public void setShowPointCloud(boolean showPointCloud)
   {
      this.showPointCloud = showPointCloud;
   }
}
