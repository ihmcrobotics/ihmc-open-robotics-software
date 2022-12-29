package us.ihmc.avatar.gpuPlanarRegions;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;

import java.nio.FloatBuffer;

public class RapidRegionsDebutOutputGenerator
{
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

   public void constructCentroidPointCloud(FloatBuffer buffer, int numberOfPoints)
   {
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

   public void constructCentroidPointCloud(BytedecoImage cxImage, BytedecoImage cyImage, BytedecoImage czImage)
   {
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
      for(int i = 0; i<patchGraph.getImageHeight(); i++)
      {
         for(int j = 0; j<patchGraph.getImageWidth(); j++)
         {
            int value = patchGraph.getCharDirect(i, j);
            if(value == 255)
               System.out.print("o");
            else if(value > 0)
               System.out.print("+");
            else
               System.out.print(".");
         }
         System.out.println();
      }
   }

   public void drawInternalNode(int id, int patchRow, int patchCol, int patchHeight, int patchWidth)
   {
      drawNode(patchRow, patchCol, patchHeight, patchWidth, getColor(id));
   }

   public void drawBoundaryNode(int id, int patchRow, int patchCol, int patchHeight, int patchWidth)
   {
      drawNode(patchRow, patchCol, patchHeight, patchWidth, boundaryColor);
   }

   private void drawNode(int patchRow, int patchCol, int patchHeight, int patchWidth, Scalar color)
   {
      opencv_imgproc.circle(debugImage, new Point(patchRow * patchHeight, patchCol * patchWidth), 2, color, -1, -1, 0);
   }

   private Scalar getColor(int id)
   {
      internalColor.red((id + 1) * 231 % 255);
      internalColor.green((id + 1) * 123 % 255);
      internalColor.blue((id + 1) * 312 % 255);
      return internalColor;
   }

   public void showDebugImage()
   {
      BytedecoOpenCVTools.display("Debug Output", debugImage, 0);
   }

   public Mat getDebugImage()
   {
      return debugImage;
   }

   public RecyclingArrayList<Point3D32> getDebugPoints()
   {
      return debugPoints;
   }

   public RecyclingArrayList<UnitVector3D> getDebugNormals()
   {
      return debugNormals;
   }
}
