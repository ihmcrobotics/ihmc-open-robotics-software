package us.ihmc.avatar.gpuPlanarRegions;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;

import java.nio.FloatBuffer;

public class RapidRegionsDebutOutputGenerator
{
   private Mat debugImage;
   private final RecyclingArrayList<Point3D32> debugPoints = new RecyclingArrayList<>(Point3D32::new);
   private final RecyclingArrayList<UnitVector3D> debugNormals = new RecyclingArrayList<>(UnitVector3D::new);

   public void create(int width, int height)
   {
      debugImage = new Mat(width, height, opencv_core.CV_8UC4);
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
      for (int y = 0; y < cxImage.getImageHeight(); y++)
      {
         for (int x = 0; x < cxImage.getImageWidth(); x++)
         {
            float cx = cxImage.getFloat(y, x);
            float cy = cyImage.getFloat(y, x);
            float cz = czImage.getFloat(y, x);

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
      for (int y = 0; y < cxImage.getImageHeight(); y++)
      {
         for (int x = 0; x < cxImage.getImageWidth(); x++)
         {
            float cx = cxImage.getFloat(y, x);
            float cy = cyImage.getFloat(y, x);
            float cz = czImage.getFloat(y, x);
            float nx = nxImage.getFloat(y, x);
            float ny = nyImage.getFloat(y, x);
            float nz = nzImage.getFloat(y, x);

            if (!(cx == 0.0f && cy == 0.0f && cz == 0.0f))
            {
               debugPoints.add().set(cx, cy, cz);
               debugNormals.add().set(nx, ny, nz);
            }
         }
      }
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
