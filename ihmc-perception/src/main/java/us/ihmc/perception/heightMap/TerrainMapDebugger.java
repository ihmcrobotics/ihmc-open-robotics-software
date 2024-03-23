package us.ihmc.perception.heightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.perception.gpuHeightMap.HeatMapGenerator;
import us.ihmc.perception.tools.PerceptionDebugTools;

public class TerrainMapDebugger
{
   private int offsetX = 0;
   private int offsetY = 0;

   private int scaleFactor;
   private int height;
   private int width;
   private int scaledHeight;
   private int scaledWidth;

   private final HeatMapGenerator contactHeatMapGenerator = new HeatMapGenerator();

   private final Mat heightMapImage;
   private final Mat stacked;
   private final Mat heightMapColorImage;
   private final Mat contactHeatMapColorImage;
   private Mat top;
   private Mat bottom;
   private Mat contactHeatMapImage;


   public TerrainMapDebugger(int height, int width, int scaleFactor)
   {
      this.height = height;
      this.width = width;
      this.scaleFactor = scaleFactor;
      this.scaledHeight = scaleFactor * height;
      this.scaledWidth = scaleFactor * width;

      heightMapImage = new Mat(scaledHeight, scaledWidth, opencv_core.CV_8UC3);
      stacked = new Mat(scaledHeight * 2, scaledWidth, opencv_core.CV_8UC4);
      heightMapColorImage = new Mat(scaledHeight, scaledWidth, opencv_core.CV_8UC4);
      contactHeatMapColorImage = new Mat(scaledHeight, scaledWidth, opencv_core.CV_8UC4);

      top = stacked.apply(new org.bytedeco.opencv.opencv_core.Rect(0, 0, heightMapColorImage.cols(), heightMapColorImage.rows()));
      bottom = stacked.apply(new org.bytedeco.opencv.opencv_core.Rect(0,
                                                    heightMapColorImage.rows(),
                                                    contactHeatMapColorImage.cols(),
                                                    contactHeatMapColorImage.rows()));
   }

   public void setOffsets(int offsetX, int offsetY)
   {
      this.offsetX = offsetX;
      this.offsetY = offsetY;
   }

   public Mat getHeightMapImage()
   {
      return heightMapImage;
   }

   public Mat getStacked()
   {
      return stacked;
   }

   public Mat getHeightMapColorImage()
   {
      return heightMapColorImage;
   }

   public Mat getContactHeatMapColorImage()
   {
      return contactHeatMapColorImage;
   }

   public void setContactHeatMapImage(Mat contactHeatMapImage)
   {
      this.contactHeatMapImage = contactHeatMapImage;
   }

   public Mat getContactHeatMapImage()
   {
      return contactHeatMapImage;
   }

   public void refresh(TerrainMapData terrainMapData)
   {
      this.offsetX = (int) (terrainMapData.getSensorOrigin().getX() * 50.0f);
      this.offsetY = (int) (terrainMapData.getSensorOrigin().getY() * 50.0f);

      PerceptionDebugTools.convertDepthCopyToColor(terrainMapData.getHeightMap().clone(), heightMapImage);
      opencv_imgproc.cvtColor(heightMapImage, heightMapColorImage, opencv_imgproc.COLOR_BGR2BGRA);
      opencv_imgproc.resize(heightMapColorImage, heightMapColorImage, new Size(scaledWidth, scaledHeight));

      if (terrainMapData.getContactMap() != null)
      {
         this.contactHeatMapImage = contactHeatMapGenerator.generateHeatMap(terrainMapData.getContactMap().clone());
         opencv_imgproc.resize(contactHeatMapImage, contactHeatMapColorImage, new Size(scaledWidth, scaledHeight));
      }
   }



   public void display(int delay)
   {
      heightMapColorImage.copyTo(top);
      contactHeatMapColorImage.copyTo(bottom);

      // make the stacked image brighter
      opencv_core.convertScaleAbs(stacked, stacked, 1.5, 0);

      PerceptionDebugTools.display("Display", stacked, delay, 1500);
   }

   public int getOffsetX()
   {
      return offsetX;
   }

   public int getOffsetY()
   {
      return offsetY;
   }

   public int getHeight()
   {
      return height;
   }

   public int getWidth()
   {
      return width;
   }

   public int getScaleFactor()
   {
      return scaleFactor;
   }
}
